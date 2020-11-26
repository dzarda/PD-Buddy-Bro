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

#include <pd.h>
#include "priorities.h"
#include "policy_engine.h"
#include "protocol_rx.h"
#include "protocol_tx.h"
#include "fusb302b.h"


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
static enum hardrst_state hardrst_reset_layer(struct pdb_config *cfg)
{
    /* First, wait for the signal to run a hard reset. */
    eventmask_t evt = chEvtWaitAny(PDB_EVT_HARDRST_RESET
            | PDB_EVT_HARDRST_I_HARDRST);

    /* Reset the stored message IDs */
    cfg->prl._rx_messageid = 0;
    cfg->prl._tx_messageidcounter = 0;

    /* Reset the Protocol RX machine */
    chEvtSignal(cfg->prl.rx_thread, PDB_EVT_PRLRX_RESET);
    chThdYield();

    /* Reset the Protocol TX machine */
    chEvtSignal(cfg->prl.tx_thread, PDB_EVT_PRLTX_RESET);
    chThdYield();

    /* Continue the process based on what event started the reset. */
    if (evt & PDB_EVT_HARDRST_RESET) {
        /* Policy Engine started the reset. */
        return PRLHRRequestHardReset;
    } else {
        /* PHY started the reset */
        return PRLHRIndicateHardReset;
    }
}

static enum hardrst_state hardrst_indicate_hard_reset(struct pdb_config *cfg)
{
    /* Tell the PE that we're doing a hard reset */
    chEvtSignal(cfg->pe.thread, PDB_EVT_PE_RESET);

    return PRLHRWaitPE;
}

static enum hardrst_state hardrst_request_hard_reset(struct pdb_config *cfg)
{
    (void) cfg;
    /* Tell the PHY to send a hard reset */
    fusb_send_hardrst(&cfg->fusb);

    return PRLHRWaitPHY;
}

static enum hardrst_state hardrst_wait_phy(struct pdb_config *cfg)
{
    (void) cfg;
    /* Wait for the PHY to tell us that it's done sending the hard reset */
    chEvtWaitAnyTimeout(PDB_EVT_HARDRST_I_HARDSENT, PD_T_HARD_RESET_COMPLETE);

    /* Move on no matter what made us stop waiting. */
    return PRLHRHardResetRequested;
}

static enum hardrst_state hardrst_hard_reset_requested(struct pdb_config *cfg)
{
    /* Tell the PE that the hard reset was sent */
    chEvtSignal(cfg->pe.thread, PDB_EVT_PE_HARD_SENT);

    return PRLHRWaitPE;
}

static enum hardrst_state hardrst_wait_pe(struct pdb_config *cfg)
{
    (void) cfg;
    /* Wait for the PE to tell us that it's done */
    chEvtWaitAny(PDB_EVT_HARDRST_DONE);

    return PRLHRComplete;
}

static enum hardrst_state hardrst_complete(struct pdb_config *cfg)
{
    (void) cfg;
    /* I'm not aware of anything we have to tell the FUSB302B, so just finish
     * the reset routine. */
    return PRLHRResetLayer;
}

/*
 * Hard Reset state machine thread
 */
static THD_FUNCTION(HardReset, cfg) {
    enum hardrst_state state = PRLHRResetLayer;

    while (true) {
        switch (state) {
            case PRLHRResetLayer:
                state = hardrst_reset_layer(cfg);
                break;
            case PRLHRIndicateHardReset:
                state = hardrst_indicate_hard_reset(cfg);
                break;
            case PRLHRRequestHardReset:
                state = hardrst_request_hard_reset(cfg);
                break;
            case PRLHRWaitPHY:
                state = hardrst_wait_phy(cfg);
                break;
            case PRLHRHardResetRequested:
                state = hardrst_hard_reset_requested(cfg);
                break;
            case PRLHRWaitPE:
                state = hardrst_wait_pe(cfg);
                break;
            case PRLHRComplete:
                state = hardrst_complete(cfg);
                break;
            default:
                /* This is an error.  It really shouldn't happen.  We might
                 * want to handle it anyway, though. */
                break;
        }
    }
}

void pdb_hardrst_run(struct pdb_config *cfg)
{
    cfg->prl.hardrst_thread = chThdCreateStatic(cfg->prl._hardrst_wa,
            sizeof(cfg->prl._hardrst_wa), PDB_PRIO_PRL, HardReset, cfg);
}
