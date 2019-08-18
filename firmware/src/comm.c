#include "comm.h"
#include "hal.h"
#include "usb.h"
#include <libopencm3/stm32/usart.h>

static const uint32_t timeout_ms = 3;
static uint32_t last_rx_ms = 0;
struct frame frame_self;

#define BUFFER_SIZE (FS+1)
static struct buffer {
    struct buffer *next;
    uint8_t data[BUFFER_SIZE];
    uint8_t index;
    uint8_t in_use; 
} buffers[32];
// XXX how many buffers do we need?

static struct buffer *buffer_alloc() {
    for (struct buffer *b = buffers; ; b++) {
        while (b->in_use == 0xff); // error!
        if (!b->in_use) {
            memset(b, 0, sizeof *b);
            b->in_use = 1;
            return b;
        }
    }
}
static void buffer_advance(struct buffer **bp) {
    struct buffer *next = (*bp)->next;
    if (next != NULL) {
        memset(*bp, 0, sizeof **bp);
        (*bp) = next;
    }
    while ((*bp)->data[0] == 0); // error, data can't be empty
    (*bp)->index = 0;
}

static void buffer_queue(struct buffer *dst, struct buffer *b) {
    while (b->data[0] == 0); // error, data can't be empty
    while (b->next != NULL); // error, data cannot have next
    while (b == dst); // error, cannot append to itself
    // queue can only go 4 deep
    for (int i = 0; i < 12 && dst->next != NULL; i++) dst = dst->next;
    if (dst->next != NULL) {
        while (b->next != NULL); // error, can only free tail buffers
        memset(b, 0, sizeof *b); // Do not enqueue b; free it
    }
    else dst->next = b;
}

static struct buffer *buffer_dn_tx;
static struct buffer *buffer_dn_rx;
static struct buffer *buffer_up_tx;
static struct buffer *buffer_up_rx;
static uint32_t downstream_rx_timeout = 1;

// Upstream: towards USB


// CONS: Constant-Overhead Nibble Stuffing
// Variant on COBS (Byte)
// Packets are delimited by a byte with a high nibble of 0 (e.g. 0x01)
// n refers to the number of bytes until the next high nibble of 0
// n cannot be 0x0
// If there is no such byte, n can be 0xF
// This scheme is not designed to pack packets with more than 15 bytes of data

static volatile int unpack_errors = 0;
static bool comm_unpack(const struct buffer * packed_buffer, struct frame * unpacked_frame) {
    uint8_t * unpacked_buf = (void *) unpacked_frame;
    const uint8_t *pb = packed_buffer->data;
    if ((*pb & 0xF0) != 0x00) {
        goto fail;
    }
    int next_zero = *pb & 0xF;
    pb++;
    for (int i = 0; i < FS; i++) {
        next_zero--;
        unpacked_buf[i] = *pb;
        if (next_zero <= 0) {
            next_zero = (*pb & 0xF0) >> 4;
            unpacked_buf[i] &= ~0xF0;
            if (next_zero == 0) {
                goto fail;
            }
        }
        pb++;
    }

    return true;
fail:
    unpack_errors++;
    memset(unpacked_buf, 0, FS);
    return false;
}

static void comm_pack(struct buffer * packed_buffer, const struct frame * unpacked_frame) {
    packed_buffer->data[0] = 0x0F;
    memcpy(&packed_buffer->data[1], unpacked_frame, sizeof *unpacked_frame);

    int last_zero = 0;
    for (int i = 1; i < FS+2; i++) {
        if (i == FS+1 || (packed_buffer->data[i] & 0xF0) == 0x00) {
            if (last_zero == 0) {
                packed_buffer->data[0] = 0x00 | (i - last_zero);
            } else {
                packed_buffer->data[last_zero] = ((i - last_zero) << 4) | (packed_buffer->data[last_zero] & 0x0F);
            }
            last_zero = i;
        }
    }
    while (packed_buffer->data[0] == 0);
}

void comm_init() {
    memset(buffers, 0, sizeof buffers);
    buffers[31].in_use = 0xff;

    buffer_up_tx = buffer_alloc();
    buffer_up_rx = buffer_alloc();
    buffer_dn_tx = buffer_alloc();
    buffer_dn_rx = buffer_alloc();

    comm_pack(buffer_up_tx, &frame_self);
    struct frame frame_zero = {0};
    comm_pack(buffer_dn_tx, &frame_zero);
    //usb_set_response(buffer_up_tx->data, BUFFER_SIZE);
    //usart_enable_rx_interrupt(USART1);
    //downstream_rx_timeout = 1;
}

void comm_init2() {
    downstream_rx_timeout = 0;
    usart_disable_tx_interrupt(USART1);
    usart_enable_rx_interrupt(USART1);
    usart_enable_tx_interrupt(USART2);
    usart_disable_rx_interrupt(USART2);
    // Kickstart TX, ew
    hal_tx(USART2, 0x00);
}

static void comm_queue_self(struct buffer *dst) {
    while (!dst->in_use);
    struct buffer *buf_self = buffer_alloc();
    struct buffer *buf_zero = buffer_alloc();
    struct frame frame_zero = {0};

    comm_pack(buf_self, &frame_self);
    comm_pack(buf_zero, &frame_zero);

    buffer_queue(dst, buf_self);
    buffer_queue(dst, buf_zero);
}

static bool comm_recv_dn(struct buffer *buf) {
    struct frame f;
    if (!comm_unpack(buf, &f)) return false;
    if (f.id == 0) return false;
    if (f.id_parent == 0) f.id_parent = frame_self.id;
    comm_pack(buf, &f);
    return true;
}

static bool comm_recv_up(struct buffer *buf) {
    struct frame f;
    if (!comm_unpack(buf, &f)) return false;
    if (f.id == frame_self.id) hal_set_led(f.rgb);
    return true;
}

void comm_usb(struct frame *frame_io) {
    // Treat USB like upstream
    // RX
    if (frame_io->id == frame_self.id) hal_set_led(frame_io->rgb);
    struct buffer *fb = buffer_alloc();
    comm_pack(fb, frame_io);
    buffer_queue(buffer_dn_tx, fb);

    // TX
    comm_unpack(buffer_up_tx, frame_io);
    if (buffer_up_tx->next == NULL) {
        comm_queue_self(buffer_up_tx);
    }
    buffer_advance(&buffer_up_tx);
}

static bool upstream_mode_tx = false;
void usart1_isr(void)  {
    // Upstream, towards USB
    if (upstream_mode_tx) {
        if (hal_tx_ready(USART1)) {
            hal_tx(USART1, buffer_up_tx->data[buffer_up_tx->index]);
            if (++buffer_up_tx->index >= BUFFER_SIZE) {
                if (buffer_up_tx->next == NULL) {
                    comm_queue_self(buffer_up_tx);
                }
                buffer_advance(&buffer_up_tx);
                upstream_mode_tx = false;
                usart_disable_tx_interrupt(USART1);
                usart_enable_rx_interrupt(USART1);
            }
        }
    } else {
        if (hal_rx_ready(USART1)) {
            uint16_t byte = hal_rx(USART1);
            if ((byte & 0xF0) == 0) {
                buffer_up_rx->index = 0;
            }
            if (buffer_up_rx->index < BUFFER_SIZE) {
                buffer_up_rx->data[buffer_up_rx->index++] = (uint8_t) (byte & 0xff);
            }
            if (buffer_up_rx->index == FS+1) {
                if (comm_recv_up(buffer_up_rx)) {
                    // XXX check logic here
                    buffer_queue(buffer_dn_tx, buffer_up_rx);
                    buffer_up_rx = buffer_alloc();
                    upstream_mode_tx = true;
                    usart_disable_rx_interrupt(USART1);
                    usart_enable_tx_interrupt(USART1);
                }
            } 
        }
    }
}

volatile int up_skips = 0;

void usart2_isr(void) {
    // Downstream, away from USB
    // We are the master, and transmit until we get a response
    if (downstream_rx_timeout == 0) {
        if (hal_tx_ready(USART2)) {
            hal_rx(USART2); // Clear pending byte(s)
            hal_tx(USART2, buffer_dn_tx->data[buffer_dn_tx->index]);
            if (++buffer_dn_tx->index >= BUFFER_SIZE) {
                buffer_advance(&buffer_dn_tx);
                downstream_rx_timeout = hal_now_ms() + 50;
                usart_disable_tx_interrupt(USART2);
                usart_enable_rx_interrupt(USART2);
            }
        }
    } else {
        if (hal_rx_ready(USART2)) {
            uint16_t byte = hal_rx(USART2);
            if ((byte & 0xF0) == 0) {
                buffer_dn_rx->index = 0;
                last_rx_ms = hal_now_ms();
            }
            if (buffer_dn_rx->index < BUFFER_SIZE) {
                buffer_dn_rx->data[buffer_dn_rx->index++] = (uint8_t) (byte & 0xff);
            }
            if (buffer_dn_rx->index == FS+1) {
                if (comm_recv_dn(buffer_dn_rx)) {
                    buffer_queue(buffer_up_tx, buffer_dn_rx);
                    buffer_dn_rx = buffer_alloc();
                } else {
                    comm_queue_self(buffer_up_tx);
                }
                downstream_rx_timeout = 0;
                usart_disable_rx_interrupt(USART2);
                usart_enable_tx_interrupt(USART2);
            } 
        }
    }
}

static int timeouts_count = 0;
void comm_poll() {
    //usart1_isr();
    /*
    if (hal_tx_ready(USART2))
        hal_tx(USART2, 0x55);
    if (hal_rx_ready(USART1))
        if (hal_rx(USART1)) nonzero_bytes++; // Clear pending byte(s)
    if (hal_tx_ready(USART1))
        hal_tx(USART1, 0x55);
    if (hal_rx_ready(USART2))
        if (hal_rx(USART2)) nonzero_bytes_2++; // Clear pending byte(s)
    */
    //memcpy(buffer_up_tx->data, &frame_self, FS);

    // Timeout: if we haven't heard from downstream in a while, reset?
    if (downstream_rx_timeout != 0 && (downstream_rx_timeout < hal_now_ms())) {
        timeouts_count++;
        downstream_rx_timeout = 0;
        usart_disable_rx_interrupt(USART2);
        usart_enable_tx_interrupt(USART2);
        // Kickstart TX, ew
        hal_tx(USART2, 0x00);
    }
}

void comm_set_down(const struct frame * frame) {
    comm_pack(buffer_dn_tx, frame);
    if (frame->id == frame_self.id) {
        hal_set_led(frame->rgb);
    }
}
