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

#include "policy_engine.h"

#include <stddef.h>
#include <stdbool.h>

#include <pd.h>
#include "priorities.h"
#include "protocol_tx.h"
#include "hard_reset.h"
#include "fusb302b.h"

#include "pt.h"
#include "pt-evt.h"


static void pe_sink_pps_periodic_timer_cb(void *cfg)
{
    /* Signal the PE thread to make a new PPS request */
    ((struct pdb_config *) cfg)->pe.events |= PDB_EVT_PE_PPS_REQUEST;
}


enum policy_engine_state {
    PESinkStartup,
    PESinkDiscovery,
    PESinkWaitCap,
    PESinkEvalCap,
    PESinkSelectCap,
    PESinkTransitionSink,
    PESinkReady,
    PESinkGetSourceCap,
    PESinkGiveSinkCap,
    PESinkHardReset,
    PESinkTransitionDefault,
    PESinkSoftReset,
    PESinkSendSoftReset,
    PESinkSendNotSupported,
    PESinkChunkReceived,
    PESinkNotSupportedReceived,
    PESinkSourceUnresponsive
};

static PT_THREAD(pe_sink_startup(struct pt *pt, struct pdb_config *cfg, enum policy_engine_state *res))
{
    PT_BEGIN(pt);
    /* We don't have an explicit contract currently */
    cfg->pe._explicit_contract = false;
    /* Tell the DPM that we've started negotiations, if it cares */
    if (cfg->dpm.pd_start != NULL) {
        cfg->dpm.pd_start(cfg);
    }

    /* No need to reset the protocol layer here.  There are two ways into this
     * state: startup and exiting hard reset.  On startup, the protocol layer
     * is reset by the startup procedure.  When exiting hard reset, the
     * protocol layer is reset by the hard reset state machine.  Since it's
     * already done somewhere else, there's no need to do it again here. */

    *res = PESinkDiscovery;
    PT_END(pt);
}

static PT_THREAD(pe_sink_discovery(struct pt *pt, struct pdb_config *cfg, enum policy_engine_state *res))
{
    PT_BEGIN(pt);
    (void) cfg;
    /* Wait for VBUS.  Since it's our only power source, we already know that
     * we have it, so just move on. */

    *res = PESinkWaitCap;
    PT_END(pt);
}
static PT_THREAD(pe_sink_wait_cap(struct pt *pt, struct pdb_config *cfg, enum policy_engine_state *res))
{
    PT_BEGIN(pt);
    /* Fetch a message from the protocol layer */
    static uint32_t evt;
    PT_EVT_WAIT_TO(pt, &cfg->pe.events,
            PDB_EVT_PE_MSG_RX| PDB_EVT_PE_I_OVRTEMP | PDB_EVT_PE_RESET, PD_T_TYPEC_SINK_WAIT_CAP, &evt);
    /* If we timed out waiting for Source_Capabilities, send a hard reset */
    if (evt == 0) {
        *res = PESinkHardReset;
        PT_EXIT(pt);
    }
    /* If we got reset signaling, transition to default */
    if (evt & PDB_EVT_PE_RESET) {
        *res = PESinkTransitionDefault;
        PT_EXIT(pt);
    }
    /* If we're too hot, we shouldn't negotiate power yet */
    if (evt & PDB_EVT_PE_I_OVRTEMP) {
        *res = PESinkWaitCap;
        PT_EXIT(pt);
    }

    /* If we got a message */
    if (evt & PDB_EVT_PE_MSG_RX) {
        /* Get the message */
        if (chMBFetchTimeout(&cfg->pe.mailbox, (msg_t *) &cfg->pe._message, TIME_IMMEDIATE) == MSG_OK) {
            /* If we got a Source_Capabilities message, read it. */
            if (PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_SOURCE_CAPABILITIES
                    && PD_NUMOBJ_GET(cfg->pe._message) > 0) {
                /* First, determine what PD revision we're using */
                if ((cfg->pe.hdr_template & PD_HDR_SPECREV) == PD_SPECREV_1_0) {
                    /* If the other end is using at least version 3.0, we'll
                     * use version 3.0. */
                    if ((cfg->pe._message->hdr & PD_HDR_SPECREV) >= PD_SPECREV_3_0) {
                        cfg->pe.hdr_template |= PD_SPECREV_3_0;
                    /* Otherwise, use 2.0.  Don't worry about the 1.0 case
                     * because we don't have hardware for PD 1.0 signaling. */
                    } else {
                        cfg->pe.hdr_template |= PD_SPECREV_2_0;
                    }
                }
                *res = PESinkEvalCap;
                PT_EXIT(pt);
            /* If the message was a Soft_Reset, do the soft reset procedure */
            } else if (PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_SOFT_RESET
                    && PD_NUMOBJ_GET(cfg->pe._message) == 0) {
                chPoolFree(&pdb_msg_pool, cfg->pe._message);
                cfg->pe._message = NULL;
                *res = PESinkSoftReset;
                PT_EXIT(pt);
            /* If we got an unexpected message, reset */
            } else {
                /* Free the received message */
                chPoolFree(&pdb_msg_pool, cfg->pe._message);
                *res = PESinkHardReset;
                PT_EXIT(pt);
            }
        }
    }

    /* If we failed to get a message, send a hard reset */
    *res = PESinkHardReset;
    PT_END(pt);
}

static PT_THREAD(pe_sink_eval_cap(struct pt *pt, struct pdb_config *cfg, enum policy_engine_state *res))
{
    PT_BEGIN(pt);
    /* If we have a Source_Capabilities message, remember the index of the
     * first PPS APDO so we can check if the request is for a PPS APDO in
     * PE_SNK_Select_Cap. */
    if (cfg->pe._message != NULL) {
        /* Start by assuming we won't find a PPS APDO (set the index greater
         * than the maximum possible) */
        cfg->pe._pps_index = 8;
        /* Search for the first PPS APDO */
        for (int8_t i = 0; i < PD_NUMOBJ_GET(cfg->pe._message); i++) {
            if ((cfg->pe._message->obj[i] & PD_PDO_TYPE) == PD_PDO_TYPE_AUGMENTED
                    && (cfg->pe._message->obj[i] & PD_APDO_TYPE) == PD_APDO_TYPE_PPS) {
                cfg->pe._pps_index = i + 1;
                break;
            }
        }
        /* New capabilities also means we can't be making a request from the
         * same PPS APDO */
        cfg->pe._last_pps = 8;
    }
    /* Get a message object for the request if we don't have one already */
    if (cfg->pe._last_dpm_request == NULL) {
        cfg->pe._last_dpm_request = chPoolAlloc(&pdb_msg_pool);
    } else {
        /* Remember the last PDO we requested if it was a PPS APDO */
        if (PD_RDO_OBJPOS_GET(cfg->pe._last_dpm_request) >= cfg->pe._pps_index) {
            cfg->pe._last_pps = PD_RDO_OBJPOS_GET(cfg->pe._last_dpm_request);
        /* Otherwise, forget any PPS APDO we had requested */
        } else {
            cfg->pe._last_pps = 8;
        }
    }
    /* Ask the DPM what to request */
    cfg->dpm.evaluate_capability(cfg, cfg->pe._message,
            cfg->pe._last_dpm_request);
    /* It's up to the DPM to free the Source_Capabilities message, which it can
     * do whenever it sees fit.  Just remove our reference to it since we won't
     * know when it's no longer valid. */
    cfg->pe._message = NULL;

    *res = PESinkSelectCap;
    PT_END(pt);
}

static PT_THREAD(pe_sink_select_cap(struct pt *pt, struct pdb_config *cfg, enum policy_engine_state *res))
{
    PT_BEGIN(pt);
    /* Transmit the request */
    chMBPostTimeout(&cfg->prl.tx_mailbox, (msg_t) cfg->pe._last_dpm_request, TIME_IMMEDIATE);
    cfg->prl.tx_events |= PDB_EVT_PRLTX_MSG_TX;
    static uint32_t evt;
    PT_EVT_WAIT(pt, &cfg->pe.events, PDB_EVT_PE_TX_DONE | PDB_EVT_PE_TX_ERR | PDB_EVT_PE_RESET, &evt);
    /* Don't free the request; we might need it again */
    /* If we got reset signaling, transition to default */
    if (evt & PDB_EVT_PE_RESET) {
        *res = PESinkTransitionDefault;
        PT_EXIT(pt);
    }
    /* If the message transmission failed, send a hard reset */
    if ((evt & PDB_EVT_PE_TX_DONE) == 0) {
        *res = PESinkHardReset;
        PT_EXIT(pt);
    }

    /* If we're using PD 3.0 */
    if ((cfg->pe.hdr_template & PD_HDR_SPECREV) == PD_SPECREV_3_0) {
        /* If the request was for a PPS APDO, start SinkPPSPeriodicTimer */
        if (PD_RDO_OBJPOS_GET(cfg->pe._last_dpm_request) >= cfg->pe._pps_index) {
            chVTSet(&cfg->pe._sink_pps_periodic_timer, PD_T_PPS_REQUEST,
                    pe_sink_pps_periodic_timer_cb, cfg);
        /* Otherwise, stop SinkPPSPeriodicTimer */
        } else {
            chVTReset(&cfg->pe._sink_pps_periodic_timer);
        }
    }
    /* This will use a virtual timer to send an event flag to this thread after
     * PD_T_PPS_REQUEST */

    /* Wait for a response */
    PT_EVT_WAIT_TO(pt, &cfg->pe.events, PDB_EVT_PE_MSG_RX | PDB_EVT_PE_RESET, PD_T_SENDER_RESPONSE, &evt);
    /* If we got reset signaling, transition to default */
    if (evt & PDB_EVT_PE_RESET) {
        *res = PESinkTransitionDefault;
        PT_EXIT(pt);
    }
    /* If we didn't get a response before the timeout, send a hard reset */
    if (evt == 0) {
        *res = PESinkHardReset;
        PT_EXIT(pt);
    }

    /* Get the response message */
    if (chMBFetchTimeout(&cfg->pe.mailbox, (msg_t *) &cfg->pe._message, TIME_IMMEDIATE) == MSG_OK) {
        /* If the source accepted our request, wait for the new power */
        if (PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_ACCEPT
                && PD_NUMOBJ_GET(cfg->pe._message) == 0) {
            /* Transition to Sink Standby if necessary */
            if (PD_RDO_OBJPOS_GET(cfg->pe._last_dpm_request) != cfg->pe._last_pps) {
                cfg->dpm.transition_standby(cfg);
            }

            cfg->pe._min_power = false;

            chPoolFree(&pdb_msg_pool, cfg->pe._message);
            cfg->pe._message = NULL;
            *res = PESinkTransitionSink;
            PT_EXIT(pt);
        /* If the message was a Soft_Reset, do the soft reset procedure */
        } else if (PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_SOFT_RESET
                && PD_NUMOBJ_GET(cfg->pe._message) == 0) {
            chPoolFree(&pdb_msg_pool, cfg->pe._message);
            cfg->pe._message = NULL;
            *res = PESinkSoftReset;
            PT_EXIT(pt);
        /* If the message was Wait or Reject */
        } else if ((PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_REJECT
                    || PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_WAIT)
                && PD_NUMOBJ_GET(cfg->pe._message) == 0) {
            /* If we don't have an explicit contract, wait for capabilities */
            if (!cfg->pe._explicit_contract) {
                chPoolFree(&pdb_msg_pool, cfg->pe._message);
                cfg->pe._message = NULL;
                *res = PESinkWaitCap;
                PT_EXIT(pt);
            /* If we do have an explicit contract, go to the ready state */
            } else {
                /* If we got here from a Wait message, we Should run
                 * SinkRequestTimer in the Ready state. */
                cfg->pe._min_power = (PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_WAIT);

                chPoolFree(&pdb_msg_pool, cfg->pe._message);
                cfg->pe._message = NULL;
                *res = PESinkReady;
                PT_EXIT(pt);
            }
        } else {
            chPoolFree(&pdb_msg_pool, cfg->pe._message);
            cfg->pe._message = NULL;
            *res = PESinkSendSoftReset;
            PT_EXIT(pt);
        }
    }
    *res = PESinkHardReset;
    PT_END(pt);
}

static PT_THREAD(pe_sink_transition_sink(struct pt *pt, struct pdb_config *cfg, enum policy_engine_state *res))
{
    PT_BEGIN(pt);
    /* Wait for the PS_RDY message */
    static uint32_t evt;
    PT_EVT_WAIT_TO(pt, &cfg->pe.events, PDB_EVT_PE_MSG_RX | PDB_EVT_PE_RESET, PD_T_PS_TRANSITION, &evt);
    /* If we got reset signaling, transition to default */
    if (evt & PDB_EVT_PE_RESET) {
        *res = PESinkTransitionDefault;
    }
    /* If no message was received, send a hard reset */
    if (evt == 0) {
        *res = PESinkHardReset;
        PT_EXIT(pt);
    }

    /* If we received a message, read it */
    if (chMBFetchTimeout(&cfg->pe.mailbox, (msg_t *) &cfg->pe._message, TIME_IMMEDIATE) == MSG_OK) {
        /* If we got a PS_RDY, handle it */
        if (PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_PS_RDY
                && PD_NUMOBJ_GET(cfg->pe._message) == 0) {
            /* We just finished negotiating an explicit contract */
            cfg->pe._explicit_contract = true;

            /* Set the output appropriately */
            if (!cfg->pe._min_power) {
                cfg->dpm.transition_requested(cfg);
            }

            chPoolFree(&pdb_msg_pool, cfg->pe._message);
            cfg->pe._message = NULL;
            *res = PESinkReady;
            PT_EXIT(pt);
        /* If there was a protocol error, send a hard reset */
        } else {
            /* Turn off the power output before this hard reset to make sure we
             * don't supply an incorrect voltage to the device we're powering.
             */
            cfg->dpm.transition_default(cfg);

            chPoolFree(&pdb_msg_pool, cfg->pe._message);
            cfg->pe._message = NULL;
            *res = PESinkHardReset;
            PT_EXIT(pt);
        }
    }

    *res = PESinkHardReset;
    PT_END(pt);
}

static PT_THREAD(pe_sink_ready(struct pt *pt, struct pdb_config *cfg, enum policy_engine_state *res))
{
    PT_BEGIN(pt);
    static uint32_t evt;

    /* Wait for an event */
    if (cfg->pe._min_power) {
        PT_EVT_WAIT_TO(pt, &cfg->pe.events, PDB_EVT_PE_MSG_RX | PDB_EVT_PE_RESET
                | PDB_EVT_PE_I_OVRTEMP | PDB_EVT_PE_GET_SOURCE_CAP
                | PDB_EVT_PE_NEW_POWER | PDB_EVT_PE_PPS_REQUEST,
                PD_T_SINK_REQUEST, &evt);
    } else {
        PT_EVT_WAIT(pt, &cfg->pe.events, PDB_EVT_PE_MSG_RX | PDB_EVT_PE_RESET
                | PDB_EVT_PE_I_OVRTEMP | PDB_EVT_PE_GET_SOURCE_CAP
                | PDB_EVT_PE_NEW_POWER | PDB_EVT_PE_PPS_REQUEST, &evt);
    }

    /* If we got reset signaling, transition to default */
    if (evt & PDB_EVT_PE_RESET) {
        *res = PESinkTransitionDefault;
        PT_EXIT(pt);
    }

    /* If we overheated, send a hard reset */
    if (evt & PDB_EVT_PE_I_OVRTEMP) {
        *res = PESinkHardReset;
        PT_EXIT(pt);
    }

    /* If the DPM wants us to, send a Get_Source_Cap message */
    if (evt & PDB_EVT_PE_GET_SOURCE_CAP) {
        /* Tell the protocol layer we're starting an AMS */
        cfg->prl.tx_events |= PDB_EVT_PRLTX_START_AMS;
        *res = PESinkGetSourceCap;
        PT_EXIT(pt);
    }

    /* If the DPM wants new power, let it figure out what power it wants
     * exactly.  This isn't exactly the transition from the spec (that would be
     * SelectCap, not EvalCap), but this works better with the particular
     * design of this firmware. */
    if (evt & PDB_EVT_PE_NEW_POWER) {
        /* Make sure we're evaluating NULL capabilities to use the old ones */
        if (cfg->pe._message != NULL) {
            chPoolFree(&pdb_msg_pool, cfg->pe._message);
            cfg->pe._message = NULL;
        }
        /* Tell the protocol layer we're starting an AMS */
        cfg->prl.tx_events |= PDB_EVT_PRLTX_START_AMS;
        *res = PESinkEvalCap;
        PT_EXIT(pt);
    }

    /* If SinkPPSPeriodicTimer ran out, send a new request */
    if (evt & PDB_EVT_PE_PPS_REQUEST) {
        /* Tell the protocol layer we're starting an AMS */
        cfg->prl.tx_events |= PDB_EVT_PRLTX_START_AMS;
        *res = PESinkSelectCap;
        PT_EXIT(pt);
    }

    /* If no event was received, the timer ran out. */
    if (evt == 0) {
        /* Repeat our Request message */
        *res = PESinkSelectCap;
        PT_EXIT(pt);
    }

    /* If we received a message */
    if (evt & PDB_EVT_PE_MSG_RX) {
        if (chMBFetchTimeout(&cfg->pe.mailbox, (msg_t *) &cfg->pe._message, TIME_IMMEDIATE) == MSG_OK) {
            /* Ignore vendor-defined messages */
            if (PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_VENDOR_DEFINED
                    && PD_NUMOBJ_GET(cfg->pe._message) > 0) {
                chPoolFree(&pdb_msg_pool, cfg->pe._message);
                cfg->pe._message = NULL;
                *res = PESinkReady;
                PT_EXIT(pt);
            /* Ignore Ping messages */
            } else if (PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_PING
                    && PD_NUMOBJ_GET(cfg->pe._message) == 0) {
                chPoolFree(&pdb_msg_pool, cfg->pe._message);
                cfg->pe._message = NULL;
                *res = PESinkReady;
                PT_EXIT(pt);
            /* DR_Swap messages are not supported */
            } else if (PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_DR_SWAP
                    && PD_NUMOBJ_GET(cfg->pe._message) == 0) {
                chPoolFree(&pdb_msg_pool, cfg->pe._message);
                cfg->pe._message = NULL;
                *res = PESinkSendNotSupported;
                PT_EXIT(pt);
            /* Get_Source_Cap messages are not supported */
            } else if (PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_GET_SOURCE_CAP
                    && PD_NUMOBJ_GET(cfg->pe._message) == 0) {
                chPoolFree(&pdb_msg_pool, cfg->pe._message);
                cfg->pe._message = NULL;
                *res = PESinkSendNotSupported;
                PT_EXIT(pt);
            /* PR_Swap messages are not supported */
            } else if (PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_PR_SWAP
                    && PD_NUMOBJ_GET(cfg->pe._message) == 0) {
                chPoolFree(&pdb_msg_pool, cfg->pe._message);
                cfg->pe._message = NULL;
                *res = PESinkSendNotSupported;
                PT_EXIT(pt);
            /* VCONN_Swap messages are not supported */
            } else if (PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_VCONN_SWAP
                    && PD_NUMOBJ_GET(cfg->pe._message) == 0) {
                chPoolFree(&pdb_msg_pool, cfg->pe._message);
                cfg->pe._message = NULL;
                *res = PESinkSendNotSupported;
                PT_EXIT(pt);
            /* Request messages are not supported */
            } else if (PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_REQUEST
                    && PD_NUMOBJ_GET(cfg->pe._message) > 0) {
                chPoolFree(&pdb_msg_pool, cfg->pe._message);
                cfg->pe._message = NULL;
                *res = PESinkSendNotSupported;
                PT_EXIT(pt);
            /* Sink_Capabilities messages are not supported */
            } else if (PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_SINK_CAPABILITIES
                    && PD_NUMOBJ_GET(cfg->pe._message) > 0) {
                chPoolFree(&pdb_msg_pool, cfg->pe._message);
                cfg->pe._message = NULL;
                *res = PESinkSendNotSupported;
                PT_EXIT(pt);
            /* Handle GotoMin messages */
            } else if (PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_GOTOMIN
                    && PD_NUMOBJ_GET(cfg->pe._message) == 0) {
                if (cfg->dpm.giveback_enabled != NULL
                        && cfg->dpm.giveback_enabled(cfg)) {
                    /* Transition to the minimum current level */
                    cfg->dpm.transition_min(cfg);
                    cfg->pe._min_power = true;

                    chPoolFree(&pdb_msg_pool, cfg->pe._message);
                    cfg->pe._message = NULL;
                    *res = PESinkTransitionSink;
                    PT_EXIT(pt);
                } else {
                    /* GiveBack is not supported */
                    chPoolFree(&pdb_msg_pool, cfg->pe._message);
                    cfg->pe._message = NULL;
                    *res = PESinkSendNotSupported;
                    PT_EXIT(pt);
                }
            /* Evaluate new Source_Capabilities */
            } else if (PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_SOURCE_CAPABILITIES
                    && PD_NUMOBJ_GET(cfg->pe._message) > 0) {
                /* Don't free the message: we need to keep the
                 * Source_Capabilities message so we can evaluate it. */
                *res = PESinkEvalCap;
                PT_EXIT(pt);
            /* Give sink capabilities when asked */
            } else if (PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_GET_SINK_CAP
                    && PD_NUMOBJ_GET(cfg->pe._message) == 0) {
                chPoolFree(&pdb_msg_pool, cfg->pe._message);
                cfg->pe._message = NULL;
                *res = PESinkGiveSinkCap;
                PT_EXIT(pt);
            /* If the message was a Soft_Reset, do the soft reset procedure */
            } else if (PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_SOFT_RESET
                    && PD_NUMOBJ_GET(cfg->pe._message) == 0) {
                chPoolFree(&pdb_msg_pool, cfg->pe._message);
                cfg->pe._message = NULL;
                *res = PESinkSoftReset;
                PT_EXIT(pt);
            /* PD 3.0 messges */
            } else if ((cfg->pe.hdr_template & PD_HDR_SPECREV) == PD_SPECREV_3_0) {
                /* If the message is a multi-chunk extended message, let it
                 * time out. */
                if ((cfg->pe._message->hdr & PD_HDR_EXT)
                        && (PD_DATA_SIZE_GET(cfg->pe._message) > PD_MAX_EXT_MSG_LEGACY_LEN)) {
                    chPoolFree(&pdb_msg_pool, cfg->pe._message);
                    cfg->pe._message = NULL;
                    *res = PESinkChunkReceived;
                    PT_EXIT(pt);
                /* Tell the DPM a message we sent got a response of
                 * Not_Supported. */
                } else if (PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_NOT_SUPPORTED
                        && PD_NUMOBJ_GET(cfg->pe._message) == 0) {
                    chPoolFree(&pdb_msg_pool, cfg->pe._message);
                    cfg->pe._message = NULL;
                    *res = PESinkNotSupportedReceived;
                    PT_EXIT(pt);
                /* If we got an unknown message, send a soft reset */
                } else {
                    chPoolFree(&pdb_msg_pool, cfg->pe._message);
                    cfg->pe._message = NULL;
                    *res = PESinkSendSoftReset;
                    PT_EXIT(pt);
                }
            /* If we got an unknown message, send a soft reset
             *
             * XXX I don't like that this is duplicated. */
            } else {
                chPoolFree(&pdb_msg_pool, cfg->pe._message);
                cfg->pe._message = NULL;
                *res = PESinkSendSoftReset;
                PT_EXIT(pt);
            }
        }
    }

    *res = PESinkReady;
    PT_END(pt);
}

static PT_THREAD(pe_sink_get_source_cap(struct pt *pt, struct pdb_config *cfg, enum policy_engine_state *res))
{
    PT_BEGIN(pt);
    /* Get a message object */
    union pd_msg *get_source_cap = chPoolAlloc(&pdb_msg_pool);
    /* Make a Get_Source_Cap message */
    get_source_cap->hdr = cfg->pe.hdr_template | PD_MSGTYPE_GET_SOURCE_CAP
        | PD_NUMOBJ(0);
    /* Transmit the Get_Source_Cap */
    chMBPostTimeout(&cfg->prl.tx_mailbox, (msg_t) get_source_cap, TIME_IMMEDIATE);
    cfg->prl.tx_events |= PDB_EVT_PRLTX_MSG_TX;
    static uint32_t evt;
    PT_EVT_WAIT(pt, &cfg->pe.events, PDB_EVT_PE_TX_DONE | PDB_EVT_PE_TX_ERR | PDB_EVT_PE_RESET, &evt);
    /* Free the sent message */
    chPoolFree(&pdb_msg_pool, get_source_cap);
    get_source_cap = NULL;
    /* If we got reset signaling, transition to default */
    if (evt & PDB_EVT_PE_RESET) {
        *res = PESinkTransitionDefault;
        PT_EXIT(pt);
    }
    /* If the message transmission failed, send a hard reset */
    if ((evt & PDB_EVT_PE_TX_DONE) == 0) {
        *res = PESinkHardReset;
        PT_EXIT(pt);
    }

    *res = PESinkReady;
    PT_END(pt);
}

static PT_THREAD(pe_sink_give_sink_cap(struct pt *pt, struct pdb_config *cfg, enum policy_engine_state *res))
{
    PT_BEGIN(pt);
    /* Get a message object */
    union pd_msg *snk_cap = chPoolAlloc(&pdb_msg_pool);
    /* Get our capabilities from the DPM */
    cfg->dpm.get_sink_capability(cfg, snk_cap);

    /* Transmit our capabilities */
    chMBPostTimeout(&cfg->prl.tx_mailbox, (msg_t) snk_cap, TIME_IMMEDIATE);
    cfg->prl.tx_events |= PDB_EVT_PRLTX_MSG_TX;
    static uint32_t evt;
    PT_EVT_WAIT(pt, &cfg->pe.events, PDB_EVT_PE_TX_DONE | PDB_EVT_PE_TX_ERR | PDB_EVT_PE_RESET, &evt);

    /* Free the Sink_Capabilities message */
    chPoolFree(&pdb_msg_pool, snk_cap);
    snk_cap = NULL;

    /* If we got reset signaling, transition to default */
    if (evt & PDB_EVT_PE_RESET) {
        *res = PESinkTransitionDefault;
        PT_EXIT(pt);
    }
    /* If the message transmission failed, send a hard reset */
    if ((evt & PDB_EVT_PE_TX_DONE) == 0) {
        *res = PESinkHardReset;
        PT_EXIT(pt);
    }

    *res = PESinkReady;
    PT_END(pt);
}

static PT_THREAD(pe_sink_hard_reset(struct pt *pt, struct pdb_config *cfg, enum policy_engine_state *res))
{
    PT_BEGIN(pt);
    /* If we've already sent the maximum number of hard resets, assume the
     * source is unresponsive. */
    if (cfg->pe._hard_reset_counter > PD_N_HARD_RESET_COUNT) {
        *res = PESinkSourceUnresponsive;
        PT_EXIT(pt);
    }

    /* Generate a hard reset signal */
    cfg->prl.hardrst_events |= PDB_EVT_HARDRST_RESET;
    static uint32_t evt;
    PT_EVT_WAIT(pt, &cfg->pe.events, PDB_EVT_PE_HARD_SENT, &evt);

    /* Increment HardResetCounter */
    cfg->pe._hard_reset_counter++;

    *res = PESinkTransitionDefault;
    PT_END(pt);
}

static PT_THREAD(pe_sink_transition_default(struct pt *pt, struct pdb_config *cfg, enum policy_engine_state *res))
{
    PT_BEGIN(pt);
    cfg->pe._explicit_contract = false;

    /* Tell the DPM to transition to default power */
    cfg->dpm.transition_default(cfg);

    /* There is no local hardware to reset. */
    /* Since we never change our data role from UFP, there is no reason to set
     * it here. */

    /* Tell the protocol layer we're done with the reset */
    cfg->prl.hardrst_events |= PDB_EVT_HARDRST_DONE;

    *res = PESinkStartup;
    PT_END(pt);
}

static PT_THREAD(pe_sink_soft_reset(struct pt *pt, struct pdb_config *cfg, enum policy_engine_state *res))
{
    PT_BEGIN(pt);
    /* No need to explicitly reset the protocol layer here.  It resets itself
     * when a Soft_Reset message is received. */

    /* Get a message object */
    union pd_msg *accept = chPoolAlloc(&pdb_msg_pool);
    /* Make an Accept message */
    accept->hdr = cfg->pe.hdr_template | PD_MSGTYPE_ACCEPT | PD_NUMOBJ(0);
    /* Transmit the Accept */
    chMBPostTimeout(&cfg->prl.tx_mailbox, (msg_t) accept, TIME_IMMEDIATE);
    cfg->prl.tx_events |= PDB_EVT_PRLTX_MSG_TX;
    static uint32_t evt;
    PT_EVT_WAIT(pt, &cfg->pe.events, PDB_EVT_PE_TX_DONE | PDB_EVT_PE_TX_ERR | PDB_EVT_PE_RESET, &evt);
    /* Free the sent message */
    chPoolFree(&pdb_msg_pool, accept);
    accept = NULL;
    /* If we got reset signaling, transition to default */
    if (evt & PDB_EVT_PE_RESET) {
        *res = PESinkTransitionDefault;
        PT_EXIT(pt);
    }
    /* If the message transmission failed, send a hard reset */
    if ((evt & PDB_EVT_PE_TX_DONE) == 0) {
        *res = PESinkHardReset;
        PT_EXIT(pt);
    }

    *res = PESinkWaitCap;
    PT_END(pt);
}

static PT_THREAD(pe_sink_send_soft_reset(struct pt *pt, struct pdb_config *cfg, enum policy_engine_state *res))
{
    PT_BEGIN(pt);
    /* No need to explicitly reset the protocol layer here.  It resets itself
     * just before a Soft_Reset message is transmitted. */

    /* Get a message object */
    union pd_msg *softrst = chPoolAlloc(&pdb_msg_pool);
    /* Make a Soft_Reset message */
    softrst->hdr = cfg->pe.hdr_template | PD_MSGTYPE_SOFT_RESET | PD_NUMOBJ(0);
    /* Transmit the soft reset */
    chMBPostTimeout(&cfg->prl.tx_mailbox, (msg_t) softrst, TIME_IMMEDIATE);
    cfg->prl.tx_events |= PDB_EVT_PRLTX_MSG_TX;
    static uint32_t evt;
    PT_EVT_WAIT(pt, &cfg->pe.events, PDB_EVT_PE_TX_DONE | PDB_EVT_PE_TX_ERR | PDB_EVT_PE_RESET, &evt);
    /* Free the sent message */
    chPoolFree(&pdb_msg_pool, softrst);
    softrst = NULL;
    /* If we got reset signaling, transition to default */
    if (evt & PDB_EVT_PE_RESET) {
        *res = PESinkTransitionDefault;
        PT_EXIT(pt);
    }
    /* If the message transmission failed, send a hard reset */
    if ((evt & PDB_EVT_PE_TX_DONE) == 0) {
        *res = PESinkHardReset;
        PT_EXIT(pt);
    }

    /* Wait for a response */
    PT_EVT_WAIT_TO(pt, &cfg->pe.events, PDB_EVT_PE_MSG_RX | PDB_EVT_PE_RESET, PD_T_SENDER_RESPONSE, &evt);
    /* If we got reset signaling, transition to default */
    if (evt & PDB_EVT_PE_RESET) {
        *res = PESinkTransitionDefault;
        PT_EXIT(pt);
    }
    /* If we didn't get a response before the timeout, send a hard reset */
    if (evt == 0) {
        *res = PESinkHardReset;
        PT_EXIT(pt);
    }

    /* Get the response message */
    if (chMBFetchTimeout(&cfg->pe.mailbox, (msg_t *) &cfg->pe._message, TIME_IMMEDIATE) == MSG_OK) {
        /* If the source accepted our soft reset, wait for capabilities. */
        if (PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_ACCEPT
                && PD_NUMOBJ_GET(cfg->pe._message) == 0) {
            chPoolFree(&pdb_msg_pool, cfg->pe._message);
            cfg->pe._message = NULL;
            *res = PESinkWaitCap;
            PT_EXIT(pt);
        /* If the message was a Soft_Reset, do the soft reset procedure */
        } else if (PD_MSGTYPE_GET(cfg->pe._message) == PD_MSGTYPE_SOFT_RESET
                && PD_NUMOBJ_GET(cfg->pe._message) == 0) {
            chPoolFree(&pdb_msg_pool, cfg->pe._message);
            cfg->pe._message = NULL;
            *res = PESinkSoftReset;
            PT_EXIT(pt);
        /* Otherwise, send a hard reset */
        } else {
            chPoolFree(&pdb_msg_pool, cfg->pe._message);
            cfg->pe._message = NULL;
            *res = PESinkHardReset;
            PT_EXIT(pt);
        }
    }
    *res = PESinkHardReset;
    PT_END(pt);
}

static PT_THREAD(pe_sink_send_not_supported(struct pt *pt, struct pdb_config *cfg, enum policy_engine_state *res))
{
    PT_BEGIN(pt);
    /* Get a message object */
    union pd_msg *not_supported = chPoolAlloc(&pdb_msg_pool);

    if ((cfg->pe.hdr_template & PD_HDR_SPECREV) == PD_SPECREV_2_0) {
        /* Make a Reject message */
        not_supported->hdr = cfg->pe.hdr_template | PD_MSGTYPE_REJECT | PD_NUMOBJ(0);
    } else if ((cfg->pe.hdr_template & PD_HDR_SPECREV) == PD_SPECREV_3_0) {
        /* Make a Not_Supported message */
        not_supported->hdr = cfg->pe.hdr_template | PD_MSGTYPE_NOT_SUPPORTED | PD_NUMOBJ(0);
    }

    /* Transmit the message */
    chMBPostTimeout(&cfg->prl.tx_mailbox, (msg_t) not_supported, TIME_IMMEDIATE);
    cfg->prl.tx_events |= PDB_EVT_PRLTX_MSG_TX;
    static uint32_t evt;
    PT_EVT_WAIT(pt, &cfg->pe.events, PDB_EVT_PE_TX_DONE | PDB_EVT_PE_TX_ERR | PDB_EVT_PE_RESET, &evt);

    /* Free the message */
    chPoolFree(&pdb_msg_pool, not_supported);
    not_supported = NULL;

    /* If we got reset signaling, transition to default */
    if (evt & PDB_EVT_PE_RESET) {
        *res = PESinkTransitionDefault;
        PT_EXIT(pt);
    }
    /* If the message transmission failed, send a soft reset */
    if ((evt & PDB_EVT_PE_TX_DONE) == 0) {
        *res = PESinkSendSoftReset;
        PT_EXIT(pt);
    }

    *res = PESinkReady;
    PT_END(pt);
}

static PT_THREAD(pe_sink_chunk_received(struct pt *pt, struct pdb_config *cfg, enum policy_engine_state *res))
{
    PT_BEGIN(pt);
    (void) cfg;

    /* Wait for tChunkingNotSupported */
    static uint32_t evt;
    PT_EVT_WAIT_TO(pt, &cfg->pe.events, PDB_EVT_PE_RESET, PD_T_CHUNKING_NOT_SUPPORTED, &evt);
    /* If we got reset signaling, transition to default */
    if (evt & PDB_EVT_PE_RESET) {
        *res = PESinkTransitionDefault;
    }

    *res = PESinkSendNotSupported;
    PT_END(pt);
}

static PT_THREAD(pe_sink_not_supported_received(struct pt *pt, struct pdb_config *cfg, enum policy_engine_state *res))
{
    PT_BEGIN(pt);
    /* Inform the Device Policy Manager that we received a Not_Supported
     * message. */
    if (cfg->dpm.not_supported_received != NULL) {
        cfg->dpm.not_supported_received(cfg);
    }

    *res = PESinkReady;
    PT_END(pt);
}

/*
 * When Power Delivery is unresponsive, fall back to Type-C Current
 */
static PT_THREAD(pe_sink_source_unresponsive(struct pt *pt, struct pdb_config *cfg, enum policy_engine_state *res))
{
    PT_BEGIN(pt);
    /* If the DPM can evaluate the Type-C Current advertisement */
    if (cfg->dpm.evaluate_typec_current != NULL) {
        /* Make the DPM evaluate the Type-C Current advertisement */
        int tcc_match = cfg->dpm.evaluate_typec_current(cfg,
                fusb_get_typec_current(&cfg->fusb));

        /* If the last two readings are the same, set the output */
        if (cfg->pe._old_tcc_match == tcc_match) {
            cfg->dpm.transition_typec(cfg);
        }

        /* Remember whether or not the last measurement succeeded */
        cfg->pe._old_tcc_match = tcc_match;
    }

    /* Wait tPDDebounce between measurements */
    // (PD_T_PD_DEBOUNCE);
    PT_YIELD(pt);
    PT_YIELD(pt);

    *res = PESinkSourceUnresponsive;
    PT_END(pt);
}

/*
 * Policy Engine state machine thread
 */
static PT_THREAD(PolicyEngine(struct pt *pt, struct pdb_config *cfg))
{
    PT_BEGIN(pt);
    static enum policy_engine_state state = PESinkStartup;
    static struct pt child;

    /* Initialize the mailbox */
    chMBObjectInit(&cfg->pe.mailbox, cfg->pe._mailbox_queue, PDB_MSG_POOL_SIZE);
    /* Initialize the VT for SinkPPSPeriodicTimer */
    chVTObjectInit(&cfg->pe._sink_pps_periodic_timer);
    /* Initialize the old_tcc_match */
    cfg->pe._old_tcc_match = -1;
    /* Initialize the pps_index */
    cfg->pe._pps_index = 8;
    /* Initialize the last_pps */
    cfg->pe._last_pps = 8;
    /* Initialize the PD message header template */
    cfg->pe.hdr_template = PD_DATAROLE_UFP | PD_POWERROLE_SINK;

    while (true) {
        switch (state) {
            case PESinkStartup:
                PT_SPAWN(pt, &child, pe_sink_startup(&child, cfg, &state));
                break;
            case PESinkDiscovery:
                PT_SPAWN(pt, &child, pe_sink_discovery(&child, cfg, &state));
                break;
            case PESinkWaitCap:
                PT_SPAWN(pt, &child, pe_sink_wait_cap(&child, cfg, &state));
                break;
            case PESinkEvalCap:
                PT_SPAWN(pt, &child, pe_sink_eval_cap(&child, cfg, &state));
                break;
            case PESinkSelectCap:
                PT_SPAWN(pt, &child, pe_sink_select_cap(&child, cfg, &state));
                break;
            case PESinkTransitionSink:
                PT_SPAWN(pt, &child, pe_sink_transition_sink(&child, cfg, &state));
                break;
            case PESinkReady:
                PT_SPAWN(pt, &child, pe_sink_ready(&child, cfg, &state));
                break;
            case PESinkGetSourceCap:
                PT_SPAWN(pt, &child, pe_sink_get_source_cap(&child, cfg, &state));
                break;
            case PESinkGiveSinkCap:
                PT_SPAWN(pt, &child, pe_sink_give_sink_cap(&child, cfg, &state));
                break;
            case PESinkHardReset:
                PT_SPAWN(pt, &child, pe_sink_hard_reset(&child, cfg, &state));
                break;
            case PESinkTransitionDefault:
                PT_SPAWN(pt, &child, pe_sink_transition_default(&child, cfg, &state));
                break;
            case PESinkSoftReset:
                PT_SPAWN(pt, &child, pe_sink_soft_reset(&child, cfg, &state));
                break;
            case PESinkSendSoftReset:
                PT_SPAWN(pt, &child, pe_sink_send_soft_reset(&child, cfg, &state));
                break;
            case PESinkSendNotSupported:
                PT_SPAWN(pt, &child, pe_sink_send_not_supported(&child, cfg, &state));
                break;
            case PESinkChunkReceived:
                PT_SPAWN(pt, &child, pe_sink_chunk_received(&child, cfg, &state));
                break;
            case PESinkSourceUnresponsive:
                PT_SPAWN(pt, &child, pe_sink_source_unresponsive(&child, cfg, &state));
                break;
            case PESinkNotSupportedReceived:
                PT_SPAWN(pt, &child, pe_sink_not_supported_received(&child, cfg, &state));
                break;
            default:
                /* This is an error.  It really shouldn't happen.  We might
                 * want to handle it anyway, though. */
                state = PESinkStartup;
                break;
        }
    }
    PT_END(pt);
}

void pdb_pe_run(struct pdb_config *cfg)
{
    (void)PT_SCHEDULE(PolicyEngine(&cfg->pe.thread, cfg));
}
