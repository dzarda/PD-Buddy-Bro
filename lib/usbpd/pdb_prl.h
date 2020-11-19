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

#ifndef PDB_PRL_H
#define PDB_PRL_H

#include <stdint.h>

#include "pdb_msg.h"

#include "pt.h"

/*
 * Structure for the protocol layer threads and variables
 */
struct pdb_prl {
    /* RX thread and event variable */
    struct pt rx_thread;
    uint32_t rx_events;
    /* TX thread and event variable */
    struct pt tx_thread;
    uint32_t tx_events;
    /* Hard reset thread and event variable */
    struct pt hardrst_thread;
    uint32_t hardrst_events;

    /* TX mailbox for PD messages to be transmitted */
    pd_msg_queue_t tx_mailbox;

    /* The ID of the last message received */
    int8_t _rx_messageid;
    /* The message being worked with by the RX thread */
    union pd_msg _rx_message;

    /* The ID of the next message we will transmit */
    int8_t _tx_messageidcounter;
    /* The message being worked with by the TX thread */
    union pd_msg *_tx_message;
};

#endif /* PDB_PRL_H */
