/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Roman Penyaev
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "stm32f1xx_hal.h"

#include "bsp/board_api.h"
#include "tusb.h"

enum  {
	BLINK_ERROR		  = 50,
	BLINK_NOT_MOUNTED = 250,
	BLINK_MOUNTED	  = 500,
	BLINK_SUSPENDED	  = 2500,

	BLINK_ALWAYS_ON	  = UINT32_MAX,
	BLINK_ALWAYS_OFF  = 0
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;
SPI_HandleTypeDef hspi1;

static void error_with_led(void);
static void led_blinking_task(void);
static void ftdi_task(void);

static void spi1_init(void)
{
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		error_with_led();
	}
}

int main(void)
{
	board_init();
	spi1_init();

	// init device stack on configured roothub port
	tud_init(BOARD_TUD_RHPORT);

	if (board_init_after_tusb) {
		board_init_after_tusb();
	}

	while (1) {
		tud_task(); // tinyusb device task
		ftdi_task();
		led_blinking_task();
	}
}

void echo_all(uint8_t buf[], uint32_t count)
{
	tud_vendor_write(buf, count);
	tud_vendor_write_flush();
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
	blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
	blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us	to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
	(void) remote_wakeup_en;
	blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
	blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// FTDI use vendor class
//--------------------------------------------------------------------+


/* Commands */
#define FTDI_SIO_RESET			0 /* Reset the port */
#define FTDI_SIO_MODEM_CTRL		1 /* Set the modem control register */
#define FTDI_SIO_SET_FLOW_CTRL		2 /* Set flow control register */
#define FTDI_SIO_SET_BAUD_RATE		3 /* Set baud rate */
#define FTDI_SIO_SET_DATA		4 /* Set the data characteristics of
									 the port */
#define FTDI_SIO_GET_MODEM_STATUS	5 /* Retrieve current value of modem
										 status register */
#define FTDI_SIO_SET_EVENT_CHAR		6 /* Set the event character */
#define FTDI_SIO_SET_ERROR_CHAR		7 /* Set the error character */
#define FTDI_SIO_SET_LATENCY_TIMER	9 /* Set the latency timer */
#define FTDI_SIO_GET_LATENCY_TIMER	0x0a /* Get the latency timer */
#define FTDI_SIO_SET_BITMODE		0x0b /* Set bitbang mode */
#define FTDI_SIO_READ_PINS		0x0c /* Read immediate value of pins */
#define FTDI_SIO_READ_EEPROM		0x90 /* Read EEPROM */

#define	 FTDI_SIO_GET_LATENCY_TIMER_REQUEST_TYPE 0xC0
#define	 FTDI_SIO_SET_LATENCY_TIMER_REQUEST_TYPE 0x40
#define	 FTDI_SIO_READ_EEPROM_REQUEST_TYPE 0xc0


// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage (setup/data/ack)
// return false to stall control endpoint (e.g unsupported request)
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
				const tusb_control_request_t *request)
{
	// nothing to with DATA & ACK stage
	if (stage != CONTROL_STAGE_SETUP)
		return true;

	switch (request->bmRequestType_bit.type)
	{
	case TUSB_REQ_TYPE_VENDOR:
		switch (request->bRequest) {
		case FTDI_SIO_SET_LATENCY_TIMER:
			return tud_control_xfer(rhport, request, NULL, 0);
		case FTDI_SIO_GET_LATENCY_TIMER: {
			uint8_t ret = 20;
			return tud_control_xfer(rhport, request, &ret, sizeof(ret));
		}
		case FTDI_SIO_READ_EEPROM: {
			uint16_t ret = 0xffff;
			return tud_control_xfer(rhport, request, &ret, sizeof(ret));
		}
		case FTDI_SIO_SET_BITMODE:
			return tud_control_xfer(rhport, request, NULL, 0);
		case FTDI_SIO_RESET:
			return tud_control_xfer(rhport, request, NULL, 0);
		case FTDI_SIO_SET_BAUD_RATE:
			return tud_control_xfer(rhport, request, NULL, 0);
		case FTDI_SIO_SET_FLOW_CTRL:
			return tud_control_xfer(rhport, request, NULL, 0);
		default:
			return false;
		}
	}

	// stall unknown request
	return false;
}

static void ftdi_task(void)
{
	if (tud_vendor_available()) {
		uint8_t buf[64];
		uint32_t count = tud_vendor_read(buf, sizeof(buf));

		(void)count;
	}
}

static void led_blinking_task(void)
{
	static uint32_t start_ms = 0;
	static bool led_state = false;

	// Blink every interval ms
	if ( board_millis() - start_ms < blink_interval_ms)
		return;
	start_ms += blink_interval_ms;

	board_led_write(led_state);
	led_state = 1 - led_state;
}

static void error_with_led(void)
{
	blink_interval_ms = BLINK_ERROR;
	while (1) {
		led_blinking_task();
	}
}
