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

#include "pd.h"
#include "policy_engine.h"
#include "protocol_rx.h"
#include "fusb302b.h"

#include "pt.h"
#include "pt-evt.h"
#include "pt-queue.h"


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
static PT_THREAD(protocol_tx_phy_reset(struct pt *pt, struct pdb_config *cfg, enum protocol_tx_state *res))
{
    PT_BEGIN(pt);
    /* Reset the PHY */
    fusb_reset(&cfg->fusb);

    /* If a message was pending when we got here, tell the policy engine that
     * we failed to send it */
    if (cfg->prl._tx_message != NULL) {
        /* Tell the policy engine that we failed */
        cfg->pe.events |= PDB_EVT_PE_TX_ERR;
        /* Finish failing to send the message */
        cfg->prl._tx_message = NULL;
    }

    /* Wait for a message request */
    *res = PRLTxWaitMessage;
    PT_END(pt);
}

/*
 * PRL_Tx_Wait_for_Message_Request state
 */
static PT_THREAD(protocol_tx_wait_message(struct pt *pt, struct pdb_config *cfg, enum protocol_tx_state *res))
{
    PT_BEGIN(pt);
    /* Wait for an event */
    static uint32_t evt;
    PT_EVT_WAIT(pt, &cfg->prl.tx_events, PDB_EVT_PRLTX_RESET | PDB_EVT_PRLTX_DISCARD | PDB_EVT_PRLTX_MSG_TX, &evt);

    if (evt & PDB_EVT_PRLTX_RESET) {
        *res = PRLTxPHYReset;
        PT_EXIT(pt);
    }
    if (evt & PDB_EVT_PRLTX_DISCARD) {
        *res = PRLTxDiscardMessage;
        PT_EXIT(pt);
    }

    /* If the policy engine is trying to send a message */
    if (evt & PDB_EVT_PRLTX_MSG_TX) {
        /* Get the message */
        cfg->prl._tx_message = pt_queue_pop(&cfg->prl.tx_mailbox);
        /* If it's a Soft_Reset, reset the TX layer first */
        if (PD_MSGTYPE_GET(cfg->prl._tx_message) == PD_MSGTYPE_SOFT_RESET
                && PD_NUMOBJ_GET(cfg->prl._tx_message) == 0) {
            *res = PRLTxReset;
            PT_EXIT(pt);
        /* Otherwise, just send the message */
        } else {
            *res = PRLTxConstructMessage;
            PT_EXIT(pt);
        }
    }

    /* Silence the compiler warning */
    *res = PRLTxDiscardMessage;
    PT_END(pt);
}

static PT_THREAD(protocol_tx_reset(struct pt *pt, struct pdb_config *cfg, enum protocol_tx_state *res))
{
    PT_BEGIN(pt);
    /* Clear MessageIDCounter */
    cfg->prl._tx_messageidcounter = 0;

    /* Tell the Protocol RX thread to reset */
    cfg->prl.rx_events |= PDB_EVT_PRLRX_RESET;
    PT_YIELD(pt);

    *res = PRLTxConstructMessage;
    PT_END(pt);
}

/*
 * PRL_Tx_Construct_Message state
 */
static PT_THREAD(protocol_tx_construct_message(struct pt *pt, struct pdb_config *cfg, enum protocol_tx_state *res))
{
    PT_BEGIN(pt);
    /* Make sure nobody wants us to reset */
    static uint32_t evt;
    evt = PT_EVT_GETANDCLEAR(&cfg->prl.tx_events, PDB_EVT_PRLTX_RESET | PDB_EVT_PRLTX_DISCARD);

    if (evt & PDB_EVT_PRLTX_RESET) {
        *res = PRLTxPHYReset;
        PT_EXIT(pt);
    }
    if (evt & PDB_EVT_PRLTX_DISCARD) {
        *res = PRLTxDiscardMessage;
        PT_EXIT(pt);
    }

    /* Set the correct MessageID in the message */
    cfg->prl._tx_message->hdr &= ~PD_HDR_MESSAGEID;
    cfg->prl._tx_message->hdr |= (cfg->prl._tx_messageidcounter % 8) << PD_HDR_MESSAGEID_SHIFT;

    /* PD 3.0 collision avoidance */
    if ((cfg->pe.hdr_template & PD_HDR_SPECREV) == PD_SPECREV_3_0) {
        /* If we're starting an AMS, wait for permission to transmit */
        evt = PT_EVT_GETANDCLEAR(&cfg->prl.tx_events, PDB_EVT_PRLTX_START_AMS);
        if (evt & PDB_EVT_PRLTX_START_AMS) {
            while (fusb_get_typec_current(&cfg->fusb) != fusb_sink_tx_ok) {
                PT_YIELD(pt);
            }
        }
    }

    /* Send the message to the PHY */
    fusb_send_message(&cfg->fusb, cfg->prl._tx_message);

    *res = PRLTxWaitResponse;
    PT_END(pt);
}

/*
 * PRL_Tx_Wait_for_PHY_Response state
 */
static PT_THREAD(protocol_tx_wait_response(struct pt *pt, struct pdb_config *cfg, enum protocol_tx_state *res))
{
    PT_BEGIN(pt);
    (void) cfg;
    /* Wait for an event.  There is no need to run CRCReceiveTimer, since the
     * FUSB302B handles that as part of its retry mechanism. */
    static uint32_t evt;
    PT_EVT_WAIT(pt, &cfg->prl.tx_events, PDB_EVT_PRLTX_RESET | PDB_EVT_PRLTX_DISCARD
            | PDB_EVT_PRLTX_I_TXSENT | PDB_EVT_PRLTX_I_RETRYFAIL, &evt);

    if (evt & PDB_EVT_PRLTX_RESET) {
        *res = PRLTxPHYReset;
        PT_EXIT(pt);
    }
    if (evt & PDB_EVT_PRLTX_DISCARD) {
        *res = PRLTxDiscardMessage;
        PT_EXIT(pt);
    }

    /* If the message was sent successfully */
    if (evt & PDB_EVT_PRLTX_I_TXSENT) {
        *res = PRLTxMatchMessageID;
        PT_EXIT(pt);
    }
    /* If the message failed to be sent */
    if (evt & PDB_EVT_PRLTX_I_RETRYFAIL) {
        *res = PRLTxTransmissionError;
        PT_EXIT(pt);
    }

    /* Silence the compiler warning */
    *res = PRLTxDiscardMessage;
    PT_END(pt);
}

/*
 * PRL_Tx_Match_MessageID state
 */
static PT_THREAD(protocol_tx_match_messageid(struct pt *pt, struct pdb_config *cfg, enum protocol_tx_state *res))
{
    PT_BEGIN(pt);
    union pd_msg goodcrc;

    /* Read the GoodCRC */
    fusb_read_message(&cfg->fusb, &goodcrc);

    /* Check that the message is correct */
    if (PD_MSGTYPE_GET(&goodcrc) == PD_MSGTYPE_GOODCRC
            && PD_NUMOBJ_GET(&goodcrc) == 0
            && PD_MESSAGEID_GET(&goodcrc) == cfg->prl._tx_messageidcounter) {
        *res = PRLTxMessageSent;
        PT_EXIT(pt);
    } else {
        *res = PRLTxTransmissionError;
        PT_EXIT(pt);
    }
    PT_END(pt);
}

static PT_THREAD(protocol_tx_transmission_error(struct pt *pt, struct pdb_config *cfg, enum protocol_tx_state *res))
{
    PT_BEGIN(pt);
    /* Increment MessageIDCounter */
    cfg->prl._tx_messageidcounter = (cfg->prl._tx_messageidcounter + 1) % 8;

    /* Tell the policy engine that we failed */
    cfg->pe.events |= PDB_EVT_PE_TX_ERR;

    cfg->prl._tx_message = NULL;
    *res = PRLTxWaitMessage;
    PT_END(pt);
}

static PT_THREAD(protocol_tx_message_sent(struct pt *pt, struct pdb_config *cfg, enum protocol_tx_state *res))
{
    PT_BEGIN(pt);
    /* Increment MessageIDCounter */
    cfg->prl._tx_messageidcounter = (cfg->prl._tx_messageidcounter + 1) % 8;

    /* Tell the policy engine that we succeeded */
    cfg->pe.events |= PDB_EVT_PE_TX_DONE;

    cfg->prl._tx_message = NULL;
    *res = PRLTxWaitMessage;
    PT_END(pt);
}

static PT_THREAD(protocol_tx_discard_message(struct pt *pt, struct pdb_config *cfg, enum protocol_tx_state *res))
{
    PT_BEGIN(pt);
    /* If we were working on sending a message, increment MessageIDCounter */
    if (cfg->prl._tx_message != NULL) {
        cfg->prl._tx_messageidcounter = (cfg->prl._tx_messageidcounter + 1) % 8;
    }

    *res = PRLTxPHYReset;
    PT_END(pt);
}

/*
 * Protocol layer TX state machine thread
 */
static PT_THREAD(ProtocolTX(struct pt *pt, struct pdb_config *cfg))
{
    PT_BEGIN(pt);
    static enum protocol_tx_state state = PRLTxPHYReset;
    static struct pt child;

    /* Initialize the mailbox */
    cfg->prl.tx_mailbox.r = 0;
    cfg->prl.tx_mailbox.w = 0;

    while (true) {
        switch (state) {
            case PRLTxPHYReset:
                PT_SPAWN(pt, &child, protocol_tx_phy_reset(&child, cfg, &state));
                break;
            case PRLTxWaitMessage:
                PT_SPAWN(pt, &child, protocol_tx_wait_message(&child, cfg, &state));
                break;
            case PRLTxReset:
                PT_SPAWN(pt, &child, protocol_tx_reset(&child, cfg, &state));
                break;
            case PRLTxConstructMessage:
                PT_SPAWN(pt, &child, protocol_tx_construct_message(&child, cfg, &state));
                break;
            case PRLTxWaitResponse:
                PT_SPAWN(pt, &child, protocol_tx_wait_response(&child, cfg, &state));
                break;
            case PRLTxMatchMessageID:
                PT_SPAWN(pt, &child, protocol_tx_match_messageid(&child, cfg, &state));
                break;
            case PRLTxTransmissionError:
                PT_SPAWN(pt, &child, protocol_tx_transmission_error(&child, cfg, &state));
                break;
            case PRLTxMessageSent:
                PT_SPAWN(pt, &child, protocol_tx_message_sent(&child, cfg, &state));
                break;
            case PRLTxDiscardMessage:
                PT_SPAWN(pt, &child, protocol_tx_discard_message(&child, cfg, &state));
                break;
            default:
                /* This is an error.  It really shouldn't happen.  We might
                 * want to handle it anyway, though. */
                break;
        }
    }
    PT_END(pt);
}

void pdb_prltx_run(struct pdb_config *cfg)
{
    (void)PT_SCHEDULE(ProtocolTX(&cfg->prl.tx_thread, cfg));
}
