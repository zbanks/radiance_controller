#pragma once
#include "prelude.h"

struct frame {
    uint32_t id;
    uint32_t id_parent;
    uint8_t input_parent;
    union {
        struct {
            uint8_t tempo;
            uint16_t intensity;
        } __attribute__((packed)) input;
        uint8_t rgb[3];
    } __attribute__((packed));
} __attribute__((packed));
#define FS 12
static_assert(sizeof(struct frame) == FS, "FS incorrect");
static_assert(FS <= 15, "FS too big");

struct frame frame_self;

void comm_init(void);
void comm_init2(void);
void comm_poll(void);
void comm_usb(struct frame * frame_io);

