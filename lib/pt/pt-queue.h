// Queue implementation from https://github.com/zserge/pt

#ifndef PT_QUEUE_H
#define PT_QUEUE_H

#define pt_queue(T, size)                                                                          \
    struct {                                                                                       \
        T buf[size];                                                                               \
        unsigned int r;                                                                            \
        unsigned int w;                                                                            \
    }

#define pt_queue_init()                                                                            \
    { .r = 0, .w = 0 }

#define pt_queue_len(q) (sizeof((q)->buf) / sizeof((q)->buf[0]))
#define pt_queue_cap(q) ((q)->w - (q)->r)
#define pt_queue_empty(q) ((q)->w == (q)->r)
#define pt_queue_full(q) (pt_queue_cap(q) == pt_queue_len(q))
#define pt_queue_reset(q) ((q)->w = (q)->r = 0)

#define pt_queue_push(q, el) (!pt_queue_full(q) && ((q)->buf[(q)->w++ % pt_queue_len(q)] = (el), 1))
#define pt_queue_peek(q) (pt_queue_empty(q) ? NULL : &(q)->buf[(q)->r % pt_queue_len(q)])
#define pt_queue_pop(q) (pt_queue_empty(q) ? NULL : &(q)->buf[(q)->r++ % pt_queue_len(q)])

#endif // PT_QUEUE_H
