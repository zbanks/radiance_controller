#include "usb.h"
#include "comm.h"

#include <libopencmsis/core_cm3.h>
#include <libopencm3/stm32/crs.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/usb/usbd.h>

static uint8_t usbd_data_buffer[128];
static struct frame usb_response;
//static uint16_t usb_response_length = 0;

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0xFF,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0xFFFE,
	.idProduct = 0x13D5,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor endps[] = {
    {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 0x01,
        .bmAttributes = USB_ENDPOINT_ATTR_BULK,
        .wMaxPacketSize = sizeof usbd_data_buffer,
        .bInterval = 0,
    },
    {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 0x82,
        .bmAttributes = USB_ENDPOINT_ATTR_BULK,
        .wMaxPacketSize = 512,
        .bInterval = 0,
    },
};

static const struct usb_interface_descriptor iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = 0xFF,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

    .endpoint = endps,
};

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &iface,
}};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"Radiance Labs",
	"Radiace Controller V1",
	"1234",
};

static uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes simple_control_callback(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req)) {
	(void)buf;
	(void)len;
	(void)complete;
	(void)usbd_dev;

	if (req->bmRequestType != 0x40)
        return USBD_REQ_NOTSUPP; /* Only accept vendor request. */

	return USBD_REQ_HANDLED;
}

static void usb_data_rx(usbd_device *usbd_dev, uint8_t endpoint) {
    (void) endpoint;

    int len = usbd_ep_read_packet(usbd_dev, 0x01, usbd_data_buffer, sizeof usbd_data_buffer);
    if (len < 0) {
        return;
    }
    if (len == FS) {
        comm_usb((void *)usbd_data_buffer);
        memcpy(&usb_response, usbd_data_buffer, sizeof usb_response);
    }

    usbd_ep_write_packet(usbd_dev, 0x82, &usb_response, sizeof usb_response);
}

static void usb_set_config_cb(usbd_device *usbd_dev, uint16_t wValue) {
	(void)wValue;

    usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, sizeof usbd_data_buffer, usb_data_rx);
    usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, sizeof usb_response, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE | USB_REQ_TYPE_VENDOR,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				simple_control_callback);
}

static usbd_device * usbd_dev = NULL;

void usb_init() {
	SYSCFG_CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;
    rcc_set_usbclk_source(RCC_HSI48);

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

	usbd_dev = usbd_init(&st_usbfs_v2_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, usb_set_config_cb);
    nvic_set_priority(NVIC_USB_IRQ, 1 << 4);
    nvic_enable_irq(NVIC_USB_IRQ);
}

void usb_poll() {
    usbd_poll(usbd_dev);
}

void __attribute__((used)) usb_isr() {
    usb_poll();
}
