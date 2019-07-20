#include "prelude.h"
#include "hal.h"
#include "usb.h"

int main(void) {
    hal_init();
    usb_init();

    while (1) {
        hal_poll();
        usb_poll();
    }

    return 0;
}
