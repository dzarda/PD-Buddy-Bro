#include "pt.h"

#include <stdint.h>

uint32_t millis();

inline unsigned pt_evt_getandclear(uint32_t *events, uint32_t mask) {
    unsigned e = *events & mask;
    *events &= ~mask;
    return e;
}

#define PT_EVT_GETANDCLEAR(events, mask) pt_evt_getandclear(events, mask)

#define PT_EVT_WAIT(pt, events, mask, result)                                                      \
    do {                                                                                           \
        PT_WAIT_UNTIL(pt, *result = (*events & (mask)));                                           \
        *events &= ~mask;                                                                          \
    } while (0)

#define PT_EVT_WAIT_TO(pt, events, mask, timeout, result)                                          \
    do {                                                                                           \
        static unsigned _start;                                                                    \
        _start = millis();                                                                         \
        PT_WAIT_UNTIL(pt, *result = (*events & (mask)) || (millis() - _start > timeout));          \
        *events &= ~mask;                                                                          \
    } while (0)
