/*
 * PD Buddy Firmware Library - USB Power Delivery for everyone
 * Copyright 2017-2018 Clayton G. Hobbs
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "protocol_tx.h"

#include <pd.h>
#include "priorities.h"
#include "policy_engine.h"
#include "protocol_rx.h"
#include "fusb302b.h"


/*
 * Protocol TX machine states
 *
 * Because the PHY can automatically send retries, the Check_RetryCounter state
 * has been removed, transitions relating to it are modified appropriately, and
 * we don't even keep a RetryCounter.
 */
enum protocol_tx_state {
    PRLTxPHYReset,
    PRLTxWaitMessage,
    PRLTxReset,
    PRLTxConstructMessage,
    PRLTxWaitResponse,
    PRLTxMatchMessageID,
    PRLTxTransmissionError,
    PRLTxMessageSent,
    PRLTxDiscardMessage
};


/*
 * PRL_Tx_PHY_Layer_Reset state
 */
static enum protocol_tx_state protocol_tx_phy_reset(struct pdb_config *cfg)
{
    /* Reset the PHY */
    fusb_reset(&cfg->fusb);

    /* If a message was pending when we got here, tell the policy engine that
     * we failed to send it */
    if (cfg->prl._tx_message != NULL) {
        /* Tell the policy engine that we failed */
        chEvtSignal(cfg->pe.thread, PDB_EVT_PE_TX_ERR);
        /* Finish failing to send the message */
        cfg->prl._tx_message = NULL;
    }

    /* Wait for a message request */
    return PRLTxWaitMessage;
}

/*
 * PRL_Tx_Wait_for_Message_Request state
 */
static enum protocol_tx_state protocol_tx_wait_message(struct pdb_config *cfg)
{
    /* Wait for an event */
    eventmask_t evt = chEvtWaitAny(PDB_EVT_PRLTX_RESET | PDB_EVT_PRLTX_DISCARD
            | PDB_EVT_PRLTX_MSG_TX);

    if (evt & PDB_EVT_PRLTX_RESET) {
        return PRLTxPHYReset;
    }
    if (evt & PDB_EVT_PRLTX_DISCARD) {
        return PRLTxDiscardMessage;
    }

    /* If the policy engine is trying to send a message */
    if (evt & PDB_EVT_PRLTX_MSG_TX) {
        /* Get the message */
        chMBFetchTimeout(&cfg->prl.tx_mailbox, (msg_t *) &cfg->prl._tx_message, TIME_IMMEDIATE);
        /* If it's a Soft_Reset, reset the TX layer first */
        if (PD_MSGTYPE_GET(cfg->prl._tx_message) == PD_MSGTYPE_SOFT_RESET
                && PD_NUMOBJ_GET(cfg->prl._tx_message) == 0) {
            return PRLTxReset;
        /* Otherwise, just send the message */
        } else {
            return PRLTxConstructMessage;
        }
    }

    /* Silence the compiler warning */
    return PRLTxDiscardMessage;
}

static enum protocol_tx_state protocol_tx_reset(struct pdb_config *cfg)
{
    /* Clear MessageIDCounter */
    cfg->prl._tx_messageidcounter = 0;

    /* Tell the Protocol RX thread to reset */
    chEvtSignal(cfg->prl.rx_thread, PDB_EVT_PRLRX_RESET);
    chThdYield();

    return PRLTxConstructMessage;
}

/*
 * PRL_Tx_Construct_Message state
 */
static enum protocol_tx_state protocol_tx_construct_message(struct pdb_config *cfg)
{
    /* Make sure nobody wants us to reset */
    eventmask_t evt = chEvtGetAndClearEvents(PDB_EVT_PRLTX_RESET | PDB_EVT_PRLTX_DISCARD);

    if (evt & PDB_EVT_PRLTX_RESET) {
        return PRLTxPHYReset;
    }
    if (evt & PDB_EVT_PRLTX_DISCARD) {
        return PRLTxDiscardMessage;
    }

    /* Set the correct MessageID in the message */
    cfg->prl._tx_message->hdr &= ~PD_HDR_MESSAGEID;
    cfg->prl._tx_message->hdr |= (cfg->prl._tx_messageidcounter % 8) << PD_HDR_MESSAGEID_SHIFT;

    /* PD 3.0 collision avoidance */
    if ((cfg->pe.hdr_template & PD_HDR_SPECREV) == PD_SPECREV_3_0) {
        /* If we're starting an AMS, wait for permission to transmit */
        evt = chEvtGetAndClearEvents(PDB_EVT_PRLTX_START_AMS);
        if (evt & PDB_EVT_PRLTX_START_AMS) {
            while (fusb_get_typec_current(&cfg->fusb) != fusb_sink_tx_ok) {
                chThdSleepMilliseconds(1);
            }
        }
    }

    /* Send the message to the PHY */
    fusb_send_message(&cfg->fusb, cfg->prl._tx_message);

    return PRLTxWaitResponse;
}

/*
 * PRL_Tx_Wait_for_PHY_Response state
 */
static enum protocol_tx_state protocol_tx_wait_response(struct pdb_config *cfg)
{
    (void) cfg;
    /* Wait for an event.  There is no need to run CRCReceiveTimer, since the
     * FUSB302B handles that as part of its retry mechanism. */
    eventmask_t evt = chEvtWaitAny(PDB_EVT_PRLTX_RESET | PDB_EVT_PRLTX_DISCARD
            | PDB_EVT_PRLTX_I_TXSENT | PDB_EVT_PRLTX_I_RETRYFAIL);

    if (evt & PDB_EVT_PRLTX_RESET) {
        return PRLTxPHYReset;
    }
    if (evt & PDB_EVT_PRLTX_DISCARD) {
        return PRLTxDiscardMessage;
    }

    /* If the message was sent successfully */
    if (evt & PDB_EVT_PRLTX_I_TXSENT) {
        return PRLTxMatchMessageID;
    }
    /* If the message failed to be sent */
    if (evt & PDB_EVT_PRLTX_I_RETRYFAIL) {
        return PRLTxTransmissionError;
    }

    /* Silence the compiler warning */
    return PRLTxDiscardMessage;
}

/*
 * PRL_Tx_Match_MessageID state
 */
static enum protocol_tx_state protocol_tx_match_messageid(struct pdb_config *cfg)
{
    union pd_msg goodcrc;

    /* Read the GoodCRC */
    fusb_read_message(&cfg->fusb, &goodcrc);

    /* Check that the message is correct */
    if (PD_MSGTYPE_GET(&goodcrc) == PD_MSGTYPE_GOODCRC
            && PD_NUMOBJ_GET(&goodcrc) == 0
            && PD_MESSAGEID_GET(&goodcrc) == cfg->prl._tx_messageidcounter) {
        return PRLTxMessageSent;
    } else {
        return PRLTxTransmissionError;
    }
}

static enum protocol_tx_state protocol_tx_transmission_error(struct pdb_config *cfg)
{
    /* Increment MessageIDCounter */
    cfg->prl._tx_messageidcounter = (cfg->prl._tx_messageidcounter + 1) % 8;

    /* Tell the policy engine that we failed */
    chEvtSignal(cfg->pe.thread, PDB_EVT_PE_TX_ERR);

    cfg->prl._tx_message = NULL;
    return PRLTxWaitMessage;
}

static enum protocol_tx_state protocol_tx_message_sent(struct pdb_config *cfg)
{
    /* Increment MessageIDCounter */
    cfg->prl._tx_messageidcounter = (cfg->prl._tx_messageidcounter + 1) % 8;

    /* Tell the policy engine that we succeeded */
    chEvtSignal(cfg->pe.thread, PDB_EVT_PE_TX_DONE);

    cfg->prl._tx_message = NULL;
    return PRLTxWaitMessage;
}

static enum protocol_tx_state protocol_tx_discard_message(struct pdb_config *cfg)
{
    /* If we were working on sending a message, increment MessageIDCounter */
    if (cfg->prl._tx_message != NULL) {
        cfg->prl._tx_messageidcounter = (cfg->prl._tx_messageidcounter + 1) % 8;
    }

    return PRLTxPHYReset;
}

/*
 * Protocol layer TX state machine thread
 */
static THD_FUNCTION(ProtocolTX, vcfg) {
    struct pdb_config *cfg = vcfg;

    enum protocol_tx_state state = PRLTxPHYReset;

    /* Initialize the mailbox */
    chMBObjectInit(&cfg->prl.tx_mailbox, cfg->prl._tx_mailbox_queue, PDB_MSG_POOL_SIZE);

    while (true) {
        switch (state) {
            case PRLTxPHYReset:
                state = protocol_tx_phy_reset(cfg);
                break;
            case PRLTxWaitMessage:
                state = protocol_tx_wait_message(cfg);
                break;
            case PRLTxReset:
                state = protocol_tx_reset(cfg);
                break;
            case PRLTxConstructMessage:
                state = protocol_tx_construct_message(cfg);
                break;
            case PRLTxWaitResponse:
                state = protocol_tx_wait_response(cfg);
                break;
            case PRLTxMatchMessageID:
                state = protocol_tx_match_messageid(cfg);
                break;
            case PRLTxTransmissionError:
                state = protocol_tx_transmission_error(cfg);
                break;
            case PRLTxMessageSent:
                state = protocol_tx_message_sent(cfg);
                break;
            case PRLTxDiscardMessage:
                state = protocol_tx_discard_message(cfg);
                break;
            default:
                /* This is an error.  It really shouldn't happen.  We might
                 * want to handle it anyway, though. */
                break;
        }
    }
}

void pdb_prltx_run(struct pdb_config *cfg)
{
    cfg->prl.tx_thread = chThdCreateStatic(cfg->prl._tx_wa,
            sizeof(cfg->prl._tx_wa), PDB_PRIO_PRL, ProtocolTX, cfg);
}
