#ifndef HID_QUEUE_H
#define HID_QUEUE_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t MODIFIER;       // Modifier keys (Ctrl, Shift, Alt, Win)
    uint8_t RESERVED;       // Always 0
    uint8_t KEYPRESS[12];   // Up to 12 keycodes
} __attribute__((packed)) HIDReport;

typedef struct {
    HIDReport *buffer;
    uint16_t capacity;
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} HIDQueue;

// Init + reset
bool hid_queue_init(HIDQueue *q, HIDReport *buffer, uint16_t capacity);
void hid_queue_reset(HIDQueue *q);

// FIFO Ops
bool hid_queue_push(HIDQueue *q, HIDReport item);
bool hid_queue_pop(HIDQueue *q, HIDReport *out);

// Helpers
bool hid_queue_is_empty(HIDQueue *q);
bool hid_queue_is_full(HIDQueue *q);

#endif
