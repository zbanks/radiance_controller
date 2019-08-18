#pragma once
#include "prelude.h"

void hal_init(void);
void hal_poll(void);

uint32_t hal_now_ms(void);
bool hal_button_up(void); // NB: API is bad
bool hal_button_down(void); // NB: API is bad
uint16_t hal_pot(void);
uint32_t hal_id(void);

bool hal_tx_ready(uint32_t usart_base);
void hal_tx(uint32_t usart_base, uint16_t byte);

bool hal_rx_ready(uint32_t usart_base);
uint16_t hal_rx(uint32_t usart_base);
//void hal_rx_set(uint8_t index);
void hal_set_led(const uint8_t rgb[3]);
