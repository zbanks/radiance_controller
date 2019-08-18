#include "prelude.h"
#include "hal.h"
#include "usb.h"
#include "comm.h"
#include <libopencm3/stm32/flash.h>

int main(void) {


    frame_self = (struct frame) {
        .id = hal_id(),
        .id_parent = 0,
        .input_parent= 0,
        .input = {
            .tempo = 0x80,
            .intensity = hal_pot(),
        },
    };

    comm_init();
    hal_init();
    usb_init();
    comm_init2();

    static const uint8_t black[3] = {1, 1, 1};
    hal_set_led(black);

    while (1) {
        hal_poll();

        if (hal_button_up()) {
            if (frame_self.input.tempo < UINT8_MAX) frame_self.input.tempo++;
        } else if (hal_button_down()) {
            if (frame_self.input.tempo > 0) frame_self.input.tempo--;
        }
        frame_self.input.intensity = hal_pot();

        usb_poll();
        comm_poll();
    }

    return 0;
}
