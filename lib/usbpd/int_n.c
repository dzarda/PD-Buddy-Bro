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

#include "int_n.h"

#include <ch.h>
#include <hal.h>

#include <pdb.h>
#include "priorities.h"
#include "fusb302b.h"
#include "protocol_rx.h"
#include "protocol_tx.h"
#include "hard_reset.h"
#include "policy_engine.h"


/*
 * INT_N polling thread
 */
static THD_FUNCTION(IntNPoll, vcfg) {
    struct pdb_config *cfg = vcfg;

    union fusb_status status;
    eventmask_t events;

    while (true) {
        /* If the INT_N line is low */
        if (palReadLine(cfg->fusb.int_n) == PAL_LOW) {
            /* Read the FUSB302B status and interrupt registers */
            fusb_get_status(&cfg->fusb, &status);

            /* If the I_GCRCSENT flag is set, tell the Protocol RX thread */
            if (status.interruptb & FUSB_INTERRUPTB_I_GCRCSENT) {
                chEvtSignal(cfg->prl.rx_thread, PDB_EVT_PRLRX_I_GCRCSENT);
            }

            /* If the I_TXSENT or I_RETRYFAIL flag is set, tell the Protocol TX
             * thread */
            events = 0;
            if (status.interrupta & FUSB_INTERRUPTA_I_RETRYFAIL) {
                events |= PDB_EVT_PRLTX_I_RETRYFAIL;
            }
            if (status.interrupta & FUSB_INTERRUPTA_I_TXSENT) {
                events |= PDB_EVT_PRLTX_I_TXSENT;
            }
            chEvtSignal(cfg->prl.tx_thread, events);

            /* If the I_HARDRST or I_HARDSENT flag is set, tell the Hard Reset
             * thread */
            events = 0;
            if (status.interrupta & FUSB_INTERRUPTA_I_HARDRST) {
                events |= PDB_EVT_HARDRST_I_HARDRST;
            }
            if (status.interrupta & FUSB_INTERRUPTA_I_HARDSENT) {
                events |= PDB_EVT_HARDRST_I_HARDSENT;
            }
            chEvtSignal(cfg->prl.hardrst_thread, events);

            /* If the I_OCP_TEMP and OVRTEMP flags are set, tell the Policy
             * Engine thread */
            if (status.interrupta & FUSB_INTERRUPTA_I_OCP_TEMP
                    && status.status1 & FUSB_STATUS1_OVRTEMP) {
                chEvtSignal(cfg->pe.thread, PDB_EVT_PE_I_OVRTEMP);
            }

        }
        chThdSleepMilliseconds(1);
    }
}

void pdb_int_n_run(struct pdb_config *cfg)
{
    cfg->int_n.thread = chThdCreateStatic(cfg->int_n._wa,
            sizeof(cfg->int_n._wa), PDB_PRIO_PRL_INT_N, IntNPoll, cfg);
}
