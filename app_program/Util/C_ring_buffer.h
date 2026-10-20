#ifndef CRB_UTIL_H
#define CRB_UTIL_H

#include <tk/tkernel.h>

#define QUEUE_SIZE 15

typedef struct {
    B data[QUEUE_SIZE];
    INT head;
    INT tail;
    INT count;
} C_RING_BUFFER;

static inline void crb_init(C_RING_BUFFER *q) {
    q->head = 0;
    q->tail = 0;
    q->count = 0;
}

static inline BOOL crb_push(C_RING_BUFFER *q, B c) {
    if (q->count >= QUEUE_SIZE) {
        return 0;
    }
    q->data[q->tail] = c;
    q->tail = (q->tail + 1) % QUEUE_SIZE;
    q->count++;
    return 1;
}

static inline BOOL crb_pop(C_RING_BUFFER *q, B *c) {
    if (q->count == 0) {
        return 0;
    }
    *c = q->data[q->head];
    q->head = (q->head + 1) % QUEUE_SIZE;
    q->count--;
    return 1;
}

#endif