#include "hal.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>

void hal_init() {
    rcc_clock_setup_in_hsi48_out_48mhz();
	rcc_periph_clock_enable(RCC_SYSCFG_COMP);

    rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	

    // Configure systick to 1kHz
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    STK_CVR = 0;
    systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    systick_interrupt_enable();
    systick_counter_enable();
}

void hal_poll() {

}

// SysTick / timer
volatile uint64_t clock_ms = 0;

void systick_handler() {
    clock_ms++;
}

uint64_t hal_now_ms() {
    return clock_ms;
}
