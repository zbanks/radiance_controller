#pragma once
#include "prelude.h"

void usb_init(void);
void usb_poll(void);

void usb_set_response(const void * data, uint16_t length);

