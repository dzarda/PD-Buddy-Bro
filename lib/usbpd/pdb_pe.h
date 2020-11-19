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

#ifndef PDB_PE_H
#define PDB_PE_H

#include "pdb_msg.h"

#include <stdbool.h>
#include <stdint.h>

#include "pt-queue.h"

/*
 * Events for the Policy Engine thread, sent by user code
 */
/* Tell the PE to send a Get_Source_Cap message */
#define PDB_EVT_PE_GET_SOURCE_CAP PDB_EVENT_MASK(7)
/* Tell the PE that new power is required */
#define PDB_EVT_PE_NEW_POWER PDB_EVENT_MASK(8)

/*
 * Structure for Policy Engine thread and variables
 */
struct pdb_pe {
    /* Policy Engine thread and event variable */
    struct pt thread;
    uint32_t events;

    /* PE mailbox for received PD messages */
    pd_msg_queue_t mailbox;
    /* PD message header template */
    uint16_t hdr_template;

    /* The received message we're currently working with */
    union pd_msg *_message;
    /* The most recent Request from the DPM */
    union pd_msg _last_dpm_request;
    /* Whether or not we have an explicit contract */
    bool _explicit_contract;
    /* Whether or not we're receiving minimum power */
    bool _min_power;
    /* The number of hard resets we've sent */
    int8_t _hard_reset_counter;
    /* The result of the last Type-C Current match comparison */
    int8_t _old_tcc_match;
    /* The index of the first PPS APDO */
    uint8_t _pps_index;
    /* The index of the just-requested PPS APDO */
    uint8_t _last_pps;
    /* Last time of SinkPPSPeriodicTimer */
    uint32_t _sink_pps_last_time;
    /* True if PPS periodic timer active */
    bool _sink_pps_timer_enabled;
};
#endif /* PDB_PE_H */
