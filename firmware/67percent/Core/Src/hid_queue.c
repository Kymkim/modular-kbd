#include "hid_queue.h"

bool hid_queue_init(HIDQueue *q, HIDReport *buffer, uint16_t capacity) {
    if (!q || !buffer || capacity == 0) return false;

    q->buffer   = buffer;
    q->capacity = capacity;
    q->head     = 0;
    q->tail     = 0;
    q->count    = 0;

    return true;
}

void hid_queue_reset(HIDQueue *q) {
    if (!q) return;
    q->head = 0;
    q->tail = 0;
    q->count = 0;
}

bool hid_queue_is_empty(HIDQueue *q) {
    return q->count == 0;
}

bool hid_queue_is_full(HIDQueue *q) {
    return q->count == q->capacity;
}

bool hid_queue_push(HIDQueue *q, HIDReport item) {
    if (hid_queue_is_full(q)) {
        return false;   // Queue full (no more shelves for potions!) (；ω；)
    }

    q->buffer[q->tail] = item;
    q->tail = (q->tail + 1) % q->capacity;
    q->count++;

    return true;
}

bool hid_queue_pop(HIDQueue *q, HIDReport *out) {
    if (hid_queue_is_empty(q)) {
        return false;   // Nothing to dequeue
    }

    if (out) {
        *out = q->buffer[q->head];
    }

    q->head = (q->head + 1) % q->capacity;
    q->count--;

    return true;
}
