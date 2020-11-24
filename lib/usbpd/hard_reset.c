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

#include "hard_reset.h"

#include "pd.h"
#include "policy_engine.h"
#include "protocol_rx.h"
#include "protocol_tx.h"
#include "fusb302b.h"

#include "pt.h"
#include "pt-evt.h"


/*
 * Hard Reset machine states
 */
enum hardrst_state {
    PRLHRResetLayer,
    PRLHRIndicateHardReset,
    PRLHRRequestHardReset,
    PRLHRWaitPHY,
    PRLHRHardResetRequested,
    PRLHRWaitPE,
    PRLHRComplete
};

/*
 * PRL_HR_Reset_Layer state
 */
static PT_THREAD(hardrst_reset_layer(struct pt *pt, struct pdb_config *cfg, enum hardrst_state *res))
{
    PT_BEGIN(pt);
    /* First, wait for the signal to run a hard reset. */
    static uint32_t evt;
    PT_EVT_WAIT(pt, &cfg->prl.hardrst_events, PDB_EVT_HARDRST_RESET | PDB_EVT_HARDRST_I_HARDRST, &evt);

    /* Reset the stored message IDs */
    cfg->prl._rx_messageid = 0;
    cfg->prl._tx_messageidcounter = 0;

    /* Reset the Protocol RX machine */
    cfg->prl.rx_events |= PDB_EVT_PRLRX_RESET;
    PT_YIELD(pt);

    /* Reset the Protocol TX machine */
    cfg->prl.tx_events |= PDB_EVT_PRLTX_RESET;
    PT_YIELD(pt);

    /* Continue the process based on what event started the reset. */
    if (evt & PDB_EVT_HARDRST_RESET) {
        /* Policy Engine started the reset. */
        *res = PRLHRRequestHardReset;
    } else {
        /* PHY started the reset */
        *res = PRLHRIndicateHardReset;
    }
    PT_END(pt);
}

static PT_THREAD(hardrst_indicate_hard_reset(struct pt *pt, struct pdb_config *cfg, enum hardrst_state *res))
{
    PT_BEGIN(pt);
    /* Tell the PE that we're doing a hard reset */
    cfg->pe.events |= PDB_EVT_PE_RESET;

    *res = PRLHRWaitPE;
    PT_END(pt);
}

static PT_THREAD(hardrst_request_hard_reset(struct pt *pt, struct pdb_config *cfg, enum hardrst_state *res))
{
    PT_BEGIN(pt);
    (void) cfg;
    /* Tell the PHY to send a hard reset */
    fusb_send_hardrst(&cfg->fusb);

    *res = PRLHRWaitPHY;
    PT_END(pt);
}

static PT_THREAD(hardrst_wait_phy(struct pt *pt, struct pdb_config *cfg, enum hardrst_state *res))
{
    PT_BEGIN(pt);
    (void) cfg;
    /* Wait for the PHY to tell us that it's done sending the hard reset */
    static uint32_t evt;
    PT_EVT_WAIT_TO(pt, &cfg->prl.hardrst_events, PDB_EVT_HARDRST_I_HARDSENT, PD_T_HARD_RESET_COMPLETE, &evt);
    cfg->pe.events |= PDB_EVT_PE_RESET;

    /* Move on no matter what made us stop waiting. */
    *res = PRLHRHardResetRequested;
    PT_END(pt);
}

static PT_THREAD(hardrst_hard_reset_requested(struct pt *pt, struct pdb_config *cfg, enum hardrst_state *res))
{
    PT_BEGIN(pt);
    /* Tell the PE that the hard reset was sent */
    cfg->pe.events |= PDB_EVT_PE_HARD_SENT;

    *res = PRLHRWaitPE;
    PT_END(pt);
}

static PT_THREAD(hardrst_wait_pe(struct pt *pt, struct pdb_config *cfg, enum hardrst_state *res))
{
    PT_BEGIN(pt);
    (void) cfg;
    /* Wait for the PE to tell us that it's done */
    static uint32_t evt;
    PT_EVT_WAIT(pt, &cfg->prl.hardrst_events, PDB_EVT_HARDRST_DONE, &evt);

    *res = PRLHRComplete;
    PT_END(pt);
}

static PT_THREAD(hardrst_complete(struct pt *pt, struct pdb_config *cfg, enum hardrst_state *res))
{
    PT_BEGIN(pt);
    (void) cfg;
    /* I'm not aware of anything we have to tell the FUSB302B, so just finish
     * the reset routine. */
    *res = PRLHRResetLayer;
    PT_END(pt);
}

/*
 * Hard Reset state machine thread
 */
static PT_THREAD(HardReset(struct pt *pt, struct pdb_config *cfg))
{
    PT_BEGIN(pt);
    static enum hardrst_state state = PRLHRResetLayer;
    static struct pt child;

    while (true) {
        switch (state) {
            case PRLHRResetLayer:
                PT_SPAWN(pt, &child, hardrst_reset_layer(&child, cfg, &state));
                break;
            case PRLHRIndicateHardReset:
                PT_SPAWN(pt, &child, hardrst_indicate_hard_reset(&child, cfg, &state));
                break;
            case PRLHRRequestHardReset:
                PT_SPAWN(pt, &child, hardrst_request_hard_reset(&child, cfg, &state));
                break;
            case PRLHRWaitPHY:
                PT_SPAWN(pt, &child, hardrst_wait_phy(&child, cfg, &state));
                break;
            case PRLHRHardResetRequested:
                PT_SPAWN(pt, &child, hardrst_hard_reset_requested(&child, cfg, &state));
                break;
            case PRLHRWaitPE:
                PT_SPAWN(pt, &child, hardrst_wait_pe(&child, cfg, &state));
                break;
            case PRLHRComplete:
                PT_SPAWN(pt, &child, hardrst_complete(&child, cfg, &state));
                break;
            default:
                /* This is an error.  It really shouldn't happen.  We might
                 * want to handle it anyway, though. */
                break;
        }
    }
    PT_END(pt);
}

void pdb_hardrst_run(struct pdb_config *cfg)
{
    (void)PT_SCHEDULE(HardReset(&cfg->prl.hardrst_thread, cfg));
}
