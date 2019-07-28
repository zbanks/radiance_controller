#include "prelude.h"
#include "hal.h"
#include "usb.h"

// this double buffering system is very very fragile

struct state {
    uint32_t id;
    uint16_t intensity;
    uint8_t tempo;
    uint8_t padding;
} __attribute__((packed));

static const uint32_t timeout_ms = 5;
#define BUFSIZE 512
static uint8_t buffers[2][BUFSIZE] = {{0}};
static uint16_t lengths[2];
static uint8_t buffer_tx;
static uint8_t buffer_rx;
static uint16_t index_tx;
static uint16_t index_rx;
static uint32_t last_rx_ms;

void usart1_isr(void) {
}

void usart2_isr(void) {
}

int main(void) {
    hal_init();
    hal_rx();
    usb_init();

    struct state s = (struct state) {
        .id = hal_id(),
        .intensity = hal_pot(),
        .tempo = 0x80,
        .padding = 0,
    };
    memcpy(buffers[0], &s, sizeof s);
    memcpy(buffers[1], &s, sizeof s);
    lengths[0] = sizeof(struct state);
    lengths[1] = sizeof(struct state);
    buffer_tx = 0;
    buffer_rx = 1;
    index_tx = 0;
    index_rx = sizeof(struct state);
    usb_set_response(buffers[buffer_tx], lengths[buffer_tx]);


    while (1) {
        uint32_t x = (hal_pot() >> 8);
        //hal_set_led((x | (x << 8) | (x << 16)));
        hal_set_led(x);
        hal_poll();
        usb_poll();
        if (hal_tx_ready()) {
            hal_tx((uint16_t) ((uint16_t) buffers[buffer_tx][index_tx] | (uint16_t) (index_tx == 0 ? 0x100 : 0)));
            if (++index_tx >= lengths[buffer_tx]) {
                index_tx = 0;
                if (buffer_tx == buffer_rx) {
                    buffer_tx = !buffer_tx;
                }
                if (last_rx_ms + timeout_ms < hal_now_ms()) {
                    lengths[0] = lengths[1] = sizeof(struct state);
                }
                usb_set_response(buffers[buffer_tx], lengths[buffer_tx]);
            }
        }
        if (hal_rx_ready()) {
            uint16_t byte = hal_rx();
            if (byte & 0x100) {
                static_assert(sizeof(struct state) == 8, "state must be 8");
                lengths[buffer_rx] = (uint16_t) (index_rx & ~(uint16_t)(sizeof(struct state) - 1));
                index_rx = sizeof(struct state);
                buffer_rx = !buffer_rx;
                last_rx_ms = hal_now_ms();
            }
            if (index_rx < BUFSIZE) {
                buffers[buffer_rx][index_rx++] = (uint8_t) (byte & 0xff);
            }
        }
        if (hal_button_up()) {
            if (s.tempo < UINT8_MAX) s.tempo++;
        } else if (hal_button_down()) {
            if (s.tempo > 0) s.tempo--;
        }
        s.intensity = hal_pot();
        memcpy(buffers[0], &s, sizeof s);
        memcpy(buffers[1], &s, sizeof s);
    }

    return 0;
}
