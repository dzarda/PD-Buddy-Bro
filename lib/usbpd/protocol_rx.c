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

#include "protocol_rx.h"

#include <stdlib.h>

#include <pd.h>
#include "priorities.h"
#include "policy_engine.h"
#include "protocol_tx.h"
#include "fusb302b.h"

#include "pt.h"
#include "pt-evt.h"

/*
 * Protocol RX machine states
 *
 * There is no Send_GoodCRC state because the PHY sends the GoodCRC for us.
 * All transitions that would go to that state instead go to Check_MessageID.
 */
enum protocol_rx_state {
    PRLRxWaitPHY,
    PRLRxReset,
    PRLRxCheckMessageID,
    PRLRxStoreMessageID
};

/*
 * PRL_Rx_Wait_for_PHY_Message state
 */

static PT_THREAD(protocol_rx_wait_phy(struct pt *pt, struct pdb_config *cfg, enum protocol_rx_state *res))
{
    PT_BEGIN(pt);
    /* Wait for an event */
    static uint32_t evt;
    PT_EVT_WAIT(pt, &cfg->pe.events, UINT32_MAX, &evt);

    /* If we got a reset event, reset */
    if (evt & PDB_EVT_PRLRX_RESET) {
        *res = PRLRxWaitPHY;
        PT_EXIT(pt);
    }
    /* If we got an I_GCRCSENT event, read the message and decide what to do */
    if (evt & PDB_EVT_PRLRX_I_GCRCSENT) {
        /* Get a buffer to read the message into.  Guaranteed to not fail
         * because we have a big enough pool and are careful. */
        cfg->prl._rx_message = chPoolAlloc(&pdb_msg_pool);
        /* Read the message */
        fusb_read_message(&cfg->fusb, cfg->prl._rx_message);
        /* If it's a Soft_Reset, go to the soft reset state */
        if (PD_MSGTYPE_GET(cfg->prl._rx_message) == PD_MSGTYPE_SOFT_RESET
                && PD_NUMOBJ_GET(cfg->prl._rx_message) == 0) {
            *res = PRLRxReset;
            PT_EXIT(pt);
        /* Otherwise, check the message ID */
        } else {
            *res = PRLRxCheckMessageID;
            PT_EXIT(pt);
        }
    }

    /* We shouldn't ever get here.  This just silence the compiler warning. */
    *res = PRLRxWaitPHY;
    PT_END(pt);
}

/*
 * PRL_Rx_Layer_Reset_for_Receive state
 */
static PT_THREAD(protocol_rx_reset(struct pt *pt, struct pdb_config *cfg, enum protocol_rx_state *res))
{
    PT_BEGIN(pt);
    /* Reset MessageIDCounter */
    cfg->prl._tx_messageidcounter = 0;

    /* Clear stored MessageID */
    cfg->prl._rx_messageid = -1;

    /* TX transitions to its reset state */
    cfg->prl.tx_events |= PDB_EVT_PRLTX_RESET;
    PT_YIELD(pt);

    /* If we got a RESET signal, reset the machine */
    if (PT_EVT_GETANDCLEAR(&cfg->prl.rx_events, PDB_EVT_PRLRX_RESET) != 0) {
        chPoolFree(&pdb_msg_pool, cfg->prl._rx_message);
        cfg->prl._rx_message = NULL;
        *res = PRLRxWaitPHY;
        PT_EXIT(pt);
    }

    /* Go to the Check_MessageID state */
    *res = PRLRxCheckMessageID;
    PT_END(pt);
}

/*
 * PRL_Rx_Check_MessageID state
 */
static PT_THREAD(protocol_rx_check_messageid(struct pt *pt, struct pdb_config *cfg, enum protocol_rx_state *res))
{
    PT_BEGIN(pt);
    /* If we got a RESET signal, reset the machine */
    if (PT_EVT_GETANDCLEAR(&cfg->prl.rx_events, PDB_EVT_PRLRX_RESET) != 0) {
        chPoolFree(&pdb_msg_pool, cfg->prl._rx_message);
        cfg->prl._rx_message = NULL;
        *res = PRLRxWaitPHY;
        PT_EXIT(pt);
    }

    /* If the message has the stored ID, we've seen this message before.  Free
     * it and don't pass it to the policy engine. */
    if (PD_MESSAGEID_GET(cfg->prl._rx_message) == cfg->prl._rx_messageid) {
        chPoolFree(&pdb_msg_pool, cfg->prl._rx_message);
        cfg->prl._rx_message = NULL;
        *res = PRLRxWaitPHY;
        PT_EXIT(pt);
    /* Otherwise, there's either no stored ID or this message has an ID we
     * haven't just seen.  Transition to the Store_MessageID state. */
    } else {
        *res = PRLRxStoreMessageID;
        PT_EXIT(pt);
    }
    PT_END(pt);
}

/*
 * PRL_Rx_Store_MessageID state
 */
static PT_THREAD(protocol_rx_store_messageid(struct pt *pt, struct pdb_config *cfg, enum protocol_rx_state *res))
{
    PT_BEGIN(pt);
    /* Tell ProtocolTX to discard the message being transmitted */
    cfg->prl.tx_events |= PDB_EVT_PRLTX_DISCARD;
    PT_YIELD(pt);

    /* Update the stored MessageID */
    cfg->prl._rx_messageid = PD_MESSAGEID_GET(cfg->prl._rx_message);

    /* Pass the message to the policy engine. */
    chMBPostTimeout(&cfg->pe.mailbox, (msg_t) cfg->prl._rx_message, TIME_IMMEDIATE);
    cfg->pe.events |= PDB_EVT_PE_MSG_RX;

    /* Don't check if we got a RESET because we'd do nothing different. */

    *res = PRLRxWaitPHY;
    PT_END(pt);
}

/*
 * Protocol layer RX state machine thread
 */
static PT_THREAD(ProtocolRX(struct pt *pt, struct pdb_config *cfg))
{
    PT_BEGIN(pt);
    static enum protocol_rx_state state = PRLRxWaitPHY;
    static struct pt child;

    while (true) {
        switch (state) {
            case PRLRxWaitPHY:
                PT_SPAWN(pt, &child, protocol_rx_wait_phy(&child, cfg, &state));
                break;
            case PRLRxReset:
                PT_SPAWN(pt, &child, protocol_rx_reset(&child, cfg, &state));
                break;
            case PRLRxCheckMessageID:
                PT_SPAWN(pt, &child, protocol_rx_check_messageid(&child, cfg, &state));
                break;
            case PRLRxStoreMessageID:
                PT_SPAWN(pt, &child, protocol_rx_store_messageid(&child, cfg, &state));
                break;
            default:
                /* This is an error.  It really shouldn't happen.  We might
                 * want to handle it anyway, though. */
                break;
        }
    }
    PT_END(pt);
}

void pdb_prlrx_run(struct pdb_config *cfg)
{
    (void)PT_SCHEDULE(ProtocolRX(&cfg->prl.rx_thread, cfg));
}
