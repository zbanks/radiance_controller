#pragma once
#include "prelude.h"

void hal_init(void);
void hal_poll(void);

uint64_t hal_now_ms(void);
