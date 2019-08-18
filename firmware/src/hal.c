#include "hal.h"

#include <libopencm3/cm3/systick.h>
#include <libopencmsis/core_cm3.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/usart.h>

#define RISETIME 0
#define PULSE_PERIOD 60
#define PULSE_WIDTH_0 (17 + 1)
#define PULSE_WIDTH_1 (34 + 1)
#define RESET_PERIODS 50
#define PULSE_BUFFER_LENGTH (24  + RESET_PERIODS)
static uint8_t pulse_buffer[PULSE_BUFFER_LENGTH];
static uint32_t led_color = 0;

void hal_init() {
    rcc_clock_setup_in_hsi48_out_48mhz();

	rcc_periph_clock_enable(RCC_SYSCFG_COMP);
    rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_ADC);
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_clock_enable(RCC_DMA1);

    // Configure systick to 1kHz
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    STK_CVR = 0;
    systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    systick_interrupt_enable();
    systick_counter_enable();
    
    // Configure up/down buttons on PA4 & PA5
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO4);
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO5);

    // Configure ADC "pot" on PA0 (channel 0)
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
    uint8_t adc_channels[1] = {0};

    adc_power_off(ADC);
    adc_set_clk_source(ADC, ADC_CLKSOURCE_ADC);
    adc_calibrate(ADC);
    adc_disable_external_trigger_regular(ADC1);
    adc_set_left_aligned(ADC);
    adc_set_continuous_conversion_mode(ADC);
    adc_enable_temperature_sensor();
    adc_set_sample_time_on_all_channels(ADC, ADC_SMPTIME_071DOT5);
    adc_set_regular_sequence(ADC, 1, adc_channels);
    adc_set_resolution(ADC, ADC_RESOLUTION_12BIT);
    adc_disable_analog_watchdog(ADC);
    adc_power_on(ADC);
    adc_start_conversion_regular(ADC);

    // Configure USARTS
    // Downstream A (Top): USART2, PA2/PA3, AF1
    // Downstream B (Bot): USART2, PA14/PA15, AF1
    // Upstream (Right): USART1, PB6/PB7, AF0
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO2 | GPIO3);
	//gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO2 | GPIO3);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO2 | GPIO3);
	//gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO14 | GPIO15);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO6 | GPIO7);
	//gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO6 | GPIO7);
    //gpio_set_af(GPIOA, GPIO_AF1, GPIO14 | GPIO15);
    gpio_set_af(GPIOB, GPIO_AF0, GPIO6 | GPIO7);

	usart_disable(USART2);
	nvic_enable_irq(NVIC_USART2_IRQ);
	nvic_set_priority(NVIC_USART2_IRQ, 1);
    usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    USART_CR3(USART2) |= USART_CR3_OVRDIS;
    usart_disable_rx_interrupt(USART2);
    usart_disable_tx_interrupt(USART2);
	usart_enable(USART2);

	usart_disable(USART1);
	nvic_enable_irq(NVIC_USART1_IRQ);
	nvic_set_priority(NVIC_USART1_IRQ, 1);
    usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    USART_CR3(USART1) |= USART_CR3_OVRDIS;
    usart_disable_rx_interrupt(USART1);
    usart_disable_tx_interrupt(USART1);
	usart_enable(USART1);

    // Configure WS2812 LED
    gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO1);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO1);
    gpio_set_af(GPIOA, GPIO_AF2, GPIO1);

	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_disable_break(TIM2);
    timer_set_period(TIM2, PULSE_PERIOD);
    timer_enable_break_main_output(TIM2);
    timer_set_dma_on_update_event(TIM2);
    timer_enable_irq(TIM2, TIM_DIER_CC2DE);

    timer_set_oc_mode(TIM2, TIM_OC2, TIM_OCM_PWM2);
    timer_set_oc_polarity_low(TIM2, TIM_OC2);
    timer_enable_oc_output(TIM2, TIM_OC2);
    timer_enable_oc_preload(TIM2, TIM_OC2);
    timer_enable_preload(TIM2);
    timer_set_oc_value(TIM2, TIM_OC2, PULSE_PERIOD / 2);

    timer_enable_counter(TIM2);

    dma_channel_reset(DMA1, DMA_CHANNEL3);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL3, (uint32_t)&TIM_CCR2(TIM2));
    dma_set_memory_address(DMA1, DMA_CHANNEL3, (uint32_t)&pulse_buffer);
    dma_set_read_from_memory(DMA1, DMA_CHANNEL3);
    dma_set_number_of_data(DMA1, DMA_CHANNEL3, PULSE_BUFFER_LENGTH);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL3, DMA_CCR_PSIZE_32BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL3, DMA_CCR_MSIZE_8BIT);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL3);
    dma_enable_circular_mode(DMA1, DMA_CHANNEL3);
    dma_set_priority(DMA1, DMA_CHANNEL3, DMA_CCR_PL_HIGH);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL3);

    nvic_clear_pending_irq(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ);
    nvic_set_priority(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ);
}

bool button_up, button_down;

void hal_poll() {
    // XXX This button reading routine is pretty bad, has false negatives
    // Should implement something that uses systick to debounce and latches
    // Button API is also bad
    static uint32_t button_up_state = 0;
    static uint32_t button_down_state = 0;

    uint16_t buttons = gpio_get(GPIOA, GPIO4 | GPIO5);

    button_up_state = (button_up_state << 1) | !(buttons & GPIO4);
    if (button_up_state == 0x0000FFFF) {
        button_up = true;
    }

    button_down_state = (button_down_state << 1) | !(buttons & GPIO5);
    if (button_down_state == 0x0000FFFF) {
        button_down = true;
    }

    if (DMA_CCR(DMA1, DMA_CHANNEL3) & DMA_CCR_EN) return;

    static_assert(PULSE_BUFFER_LENGTH == sizeof(pulse_buffer), "pb");
    for (uint16_t i = 0; i < PULSE_BUFFER_LENGTH; i++) {
        if (i < RESET_PERIODS)
            pulse_buffer[i] = 0;
        else if (i < RESET_PERIODS + 24) 
            pulse_buffer[i] = (led_color & ((uint32_t)1 << (23 - (i - RESET_PERIODS)))) ? PULSE_WIDTH_1 : PULSE_WIDTH_0;
        else 
            pulse_buffer[i] = 0;
    }
    TIM_CCR2(TIM2) = 0;

    dma_set_number_of_data(DMA1, DMA_CHANNEL3, PULSE_BUFFER_LENGTH);

    dma_clear_interrupt_flags(DMA1, DMA_CHANNEL3, DMA_TCIF);
    dma_enable_channel(DMA1, DMA_CHANNEL3);
    TIM_EGR(TIM2) = TIM_EGR_UG;
}

bool hal_button_up() {
    bool rc = button_up;
    button_up = false;
    return rc;
}

bool hal_button_down() {
    bool rc = button_down;
    button_down = false;
    return rc;
}

uint16_t hal_pot() {
    // Pot is backwards
    return (uint16_t) (((uint16_t) -1) - adc_read_regular(ADC_BASE));
}

// SysTick / timer
volatile uint32_t clock_ms = 0;

void sys_tick_handler() {
    clock_ms++;
}

uint32_t hal_now_ms() {
    return clock_ms;
}

uint32_t hal_id() {
    // XXX some of these bits are constant, we can probably manually guarantee uniqueness?
    // Based on lowbias32 from https://nullprogram.com/blog/2018/07/31/
    uint32_t id[3] = {0};
    desig_get_unique_id(id);
    uint32_t x = id[0];
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    x ^= id[1];
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    x ^= id[2];
    // This last iteration doesn't add any entropy, but it shuffles the bits around to make visual inspection easier
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}

bool hal_tx_ready(uint32_t usart_base) {
	return usart_get_flag(usart_base, USART_FLAG_TXE);
}

void hal_tx(uint32_t usart_base, uint16_t byte) {
    usart_send(usart_base, byte);
}

bool hal_rx_ready(uint32_t usart_base) {
	return usart_get_flag(usart_base, USART_FLAG_RXNE) || usart_get_flag(usart_base, USART_FLAG_ORE);
}

uint16_t hal_rx(uint32_t usart_base) {
    //USART_ISR(usart_base) &= ~USART_ISR_ORE;
    uint16_t v = usart_recv(usart_base);
    USART_ICR(usart_base) |= USART_ICR_ORECF;
    //USART_ICR(usart_base) = (uint32_t) -1;
    return v;
}

void hal_set_led(const uint8_t rgb[3]) {
    //          blue      red              green
    led_color = rgb[2] | (rgb[0] << 8) | (rgb[1] << 16);
}

void __attribute__((used)) dma1_channel2_3_dma2_channel1_2_isr() {
    dma_disable_channel(DMA1, DMA_CHANNEL3);
    if (DMA1_ISR & DMA_ISR_TCIF3) {
        DMA1_IFCR = DMA_ISR_TCIF3;
    }
}
