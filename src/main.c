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

/* Shifting commands IN MPSSE Mode*/
#define MPSSE_WRITE_NEG 0x01   /* Write TDI/DO on negative TCK/SK edge*/
#define MPSSE_BITMODE   0x02   /* Write bits, not bytes */
#define MPSSE_READ_NEG  0x04   /* Sample TDO/DI on negative TCK/SK edge */
#define MPSSE_LSB       0x08   /* LSB first */
#define MPSSE_DO_WRITE  0x10   /* Write TDI/DO */
#define MPSSE_DO_READ   0x20   /* Read TDO/DI */
#define MPSSE_WRITE_TMS 0x40   /* Write TMS/CS */

/* FTDI MPSSE commands */
#define SET_BITS_LOW   0x80
/*BYTE DATA*/
/*BYTE Direction*/
#define SET_BITS_HIGH  0x82
/*BYTE DATA*/
/*BYTE Direction*/
#define GET_BITS_LOW   0x81
#define GET_BITS_HIGH  0x83
#define LOOPBACK_START 0x84
#define LOOPBACK_END   0x85
#define TCK_DIVISOR    0x86
/* H Type specific commands */
#define DIS_DIV_5       0x8a
#define EN_DIV_5        0x8b
#define EN_3_PHASE      0x8c
#define DIS_3_PHASE     0x8d
#define CLK_BITS        0x8e
#define CLK_BYTES       0x8f
#define CLK_WAIT_HIGH   0x94
#define CLK_WAIT_LOW    0x95
#define EN_ADAPTIVE     0x96
#define DIS_ADAPTIVE    0x97
#define CLK_BYTES_OR_HIGH 0x9c
#define CLK_BYTES_OR_LOW  0x9d

/* Custom commands */
#define SET_CLK_PIN       0xb0

/* Commands */
#define FTDI_SIO_RESET			0 /* Reset the port */
#define FTDI_SIO_MODEM_CTRL		1 /* Set the modem control register */
#define FTDI_SIO_SET_FLOW_CTRL		2 /* Set flow control register */
#define FTDI_SIO_SET_BAUD_RATE		3 /* Set baud rate */
#define FTDI_SIO_SET_DATA		4 /* Set the data characteristics of
									 the port */
#define FTDI_SIO_GET_MODEM_STATUS	5 /* Retrieve current value of modem status register */
#define FTDI_SIO_SET_EVENT_CHAR		6 /* Set the event character */
#define FTDI_SIO_SET_ERROR_CHAR		7 /* Set the error character */
#define FTDI_SIO_SET_LATENCY_TIMER	9 /* Set the latency timer */
#define FTDI_SIO_GET_LATENCY_TIMER	0x0a /* Get the latency timer */
#define FTDI_SIO_SET_BITMODE		0x0b /* Set bitbang mode */
#define FTDI_SIO_READ_PINS		0x0c /* Read immediate value of pins */
#define FTDI_SIO_READ_EEPROM		0x90 /* Read EEPROM */

enum  {
	BLINK_ERROR	  = 50,
	BLINK_NOT_MOUNTED = 250,
	BLINK_MOUNTED	  = 500,
	BLINK_SUSPENDED	  = 2500,

	BLINK_ALWAYS_ON	  = UINT32_MAX,
	BLINK_ALWAYS_OFF  = 0,

	CLK_FREQ          = 12000000,
	CLK_PIN           = 0, /* AD0 by default */
	DMA_GPIO_NUM      = 128,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;
static uint32_t dma_gpio_arr[DMA_GPIO_NUM];

static uint32_t clk_freq = CLK_FREQ;
static uint8_t  clk_pin  = CLK_PIN;

static uint8_t gpio_low_dir;
static uint8_t gpio_low_val;
static uint8_t gpio_high_dir;
static uint8_t gpio_high_val;

SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_up;

struct gpio_desc {
	GPIO_TypeDef *bank;
	uint32_t      pin;
};

static const struct gpio_desc low_pin_mapping[8] = {
	[0] = { GPIOA, GPIO_PIN_5 }, /* AD0 -> PA5 */
	[1] = { GPIOA, GPIO_PIN_7 }, /* AD1 -> PA7 */
	[2] = { GPIOA, GPIO_PIN_6 }, /* AD2 -> PA6 */
	[3] = { GPIOA, GPIO_PIN_4 }, /* AD3 -> PA4 */
	[4] = { GPIOA, GPIO_PIN_3 }, /* AD4 -> PA3 */
	[5] = { GPIOA, GPIO_PIN_2 }, /* AD5 -> PA2 */
	[6] = { GPIOA, GPIO_PIN_1 }, /* AD6 -> PA1 */
	[7] = { GPIOA, GPIO_PIN_0 }, /* AD7 -> PA0 */
};

static const struct gpio_desc high_pin_mapping[8] = {
	[0] = { GPIOB, GPIO_PIN_12 }, /* AC0 -> PB12 */
	[1] = { GPIOB, GPIO_PIN_13 }, /* AC1 -> PB13 */
	[2] = { GPIOB, GPIO_PIN_14 }, /* AC2 -> PB14 */
	[3] = { GPIOB, GPIO_PIN_15 }, /* AC3 -> PB15 */
	[4] = { GPIOA, GPIO_PIN_8  }, /* AC4 -> PA8  */
	[5] = { GPIOA, GPIO_PIN_9  }, /* AC5 -> PA9  */
	[6] = { GPIOA, GPIO_PIN_10 }, /* AC6 -> PA10 */
	[7] = { GPIOA, GPIO_PIN_11 }, /* AC7 -> PA11 */
};

__attribute__((noreturn))
static void error_with_led(void);
static void led_blinking_task(void);
static void ftdi_task(void);
static void ftdi_reset(void);
static void ftdi_set_bits_low(uint8_t value, uint8_t dir);
static void ftdi_set_bits_high(uint8_t value, uint8_t dir);

static void dma_init(void)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

static bool tim1_init(uint32_t freq)
{
	TIM_ClockConfigTypeDef clk;
	TIM_MasterConfigTypeDef master;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = SystemCoreClock / freq / 2;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
		return false;

	clk = (TIM_ClockConfigTypeDef) {
		.ClockSource = TIM_CLOCKSOURCE_INTERNAL,
	};
	if (HAL_TIM_ConfigClockSource(&htim1, &clk) != HAL_OK)
		return false;

	master = (TIM_MasterConfigTypeDef) {
		.MasterOutputTrigger = TIM_TRGO_UPDATE,
		.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE,
	};
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &master) != HAL_OK)
		return false;

	return true;
}

static void spi1_init(uint8_t cmd)
{
	bool rx = cmd & MPSSE_DO_READ;
	bool tx = cmd & MPSSE_DO_WRITE;
	bool tx_neg = cmd & MPSSE_WRITE_NEG;
	bool lsb = cmd & MPSSE_LSB;
	bool clk_pol = (gpio_low_val & TU_BIT(0));
	uint32_t clk_phase;
	GPIO_InitTypeDef gpio;

	if (!(gpio_low_dir & TU_BIT(0)))
	    /*
	     * We control SPI CLK polarity by initial setting of CLK pin,
	     * which should be already configured as output. Consider as
	     * fatal. Yeah, proper error should be returned, I know.
	     */
	    error_with_led();

	if (!clk_pol && !tx_neg)
		/* SPI mode 1 (out on +v edge, in on -v edge) */
		clk_phase = SPI_PHASE_2EDGE;
	else if (!clk_pol && tx_neg)
		/* SPI mode 0 (out on -v edge, in on +v edge) */
		clk_phase = SPI_PHASE_1EDGE;
	else if (clk_pol && !tx_neg)
		/* SPI mode 2 (out on +v edge, in on -v edge) */
		clk_phase = SPI_PHASE_1EDGE;
	else
		/* SPI mode 3 (out on -v edge, in on +v edge) */
		clk_phase = SPI_PHASE_2EDGE;

	__HAL_RCC_SPI1_CLK_ENABLE();

	/* PA5 -> SPI1_SCK */
	gpio = (GPIO_InitTypeDef) {
		.Pin = GPIO_PIN_5,
		.Mode = GPIO_MODE_AF_PP,
		.Speed = GPIO_SPEED_FREQ_HIGH,
	};
	if (tx)
		/* PA7 -> SPI1_MOSI */
		gpio.Pin |= GPIO_PIN_7;
	HAL_GPIO_Init(GPIOA, &gpio);

	if (rx) {
		/* PA6 -> SPI1_MISO */
		gpio = (GPIO_InitTypeDef) {
			.Pin = GPIO_PIN_6,
			.Mode = GPIO_MODE_INPUT,
			.Pull = GPIO_NOPULL,
		};
		HAL_GPIO_Init(GPIOA, &gpio);
	}

	/* SPI1 interrupt Init */
	HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(SPI1_IRQn);

	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = (rx && !tx) ?
		SPI_DIRECTION_2LINES_RXONLY :
		SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = clk_pol ?
		SPI_POLARITY_HIGH :
		SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = clk_phase;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi1.Init.FirstBit = lsb ?
		SPI_FIRSTBIT_LSB :
		SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
		error_with_led();
}

static void spi1_deinit(void)
{
	HAL_SPI_DeInit(&hspi1);
	__HAL_RCC_SPI1_CLK_DISABLE();
	HAL_NVIC_DisableIRQ(SPI1_IRQn);

	/* Restore GPIO to values before SPI */
	ftdi_set_bits_low(gpio_low_val, gpio_low_dir);
}

int main(void)
{
	board_init();
	dma_init();

	// init device stack on configured roothub port
	tud_init(BOARD_TUD_RHPORT);

	if (board_init_after_tusb) {
		board_init_after_tusb();
	}

	while (1) {
		tud_task();
		ftdi_task();
		led_blinking_task();
	}
}

/*
 * tinyusb callbacks
 */

/* Invoked when device is mounted */
void tud_mount_cb(void)
{
	blink_interval_ms = BLINK_MOUNTED;
}

/* Invoked when device is unmounted */
void tud_umount_cb(void)
{
	blink_interval_ms = BLINK_NOT_MOUNTED;
}

/*
 * Invoked when usb bus is suspended
 * remote_wakeup_en : if host allow us	to perform remote wakeup
 * Within 7ms, device must draw an average of current less than 2.5 mA from bus
 */
void tud_suspend_cb(bool remote_wakeup_en)
{
	(void) remote_wakeup_en;
	blink_interval_ms = BLINK_SUSPENDED;
}

/* Invoked when usb bus is resumed */
void tud_resume_cb(void)
{
	blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// FTDI use vendor class
//--------------------------------------------------------------------+

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
			ftdi_reset();
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

static void set_gpio(GPIO_TypeDef *GPIOx, uint32_t pin, bool out, bool high)
{
	GPIO_InitTypeDef gpio = {
		.Pin   = pin,
		.Mode  = out ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT,
		.Pull  = GPIO_NOPULL,
		.Speed = GPIO_SPEED_FREQ_HIGH,
	};
	if (out)
		HAL_GPIO_WritePin(GPIOx, pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_Init(GPIOx, &gpio);
}

static void __set_bits(const struct gpio_desc *map,
		       uint8_t value, uint8_t dir)
{
	int i;

	for (i = 0; i < 8; i++)
		set_gpio(map[i].bank, map[i].pin,
			 dir & TU_BIT(i), value & TU_BIT(i));
}

static void ftdi_set_bits_low(uint8_t value, uint8_t dir)
{
	const struct gpio_desc *map = low_pin_mapping;

	gpio_low_dir = dir;
	gpio_low_val = value;
	__set_bits(map, value, dir);
}

static void ftdi_set_bits_high(uint8_t value, uint8_t dir)
{
	const struct gpio_desc *map = high_pin_mapping;

	gpio_high_dir = dir;
	gpio_high_val = value;
	__set_bits(map, value, dir);
}

static void ftdi_reset(void)
{
	/* To default freqeuncy */
	clk_freq = CLK_FREQ;
	/* To default pin */
	clk_pin = CLK_PIN;

	/* All to in */
	ftdi_set_bits_low(0, 0);
	ftdi_set_bits_high(0, 0);
}

static void ftdi_set_tck_divisor(uint8_t low, uint8_t high)
{
	uint16_t div = ((uint16_t)high << 8) | low;
	uint32_t freq = CLK_FREQ / ((1 + div) * 2);

	if (freq)
		clk_freq = freq;
}

static bool ftdi_mpsse_write_read(uint8_t cmd, uint8_t low, uint8_t high)
{
	static uint8_t out[64], in[64];
	static bool read;

	uint16_t len = ((uint16_t)high << 8) | low;

	/* See FTDI Application Note AN_108 */
	len += 1;
	if (len > sizeof(in))
		/* TODO: handle more than buf size */
		error_with_led();

	if (!read && len > tud_vendor_available())
		return false;
	else if (!read) {
		HAL_StatusTypeDef st;
		uint8_t rdwr;

		tud_vendor_read(out, len);
		read = true;

		spi1_init(cmd);

		rdwr = (cmd & (MPSSE_DO_WRITE|MPSSE_DO_READ));
		if (rdwr == (MPSSE_DO_WRITE|MPSSE_DO_READ))
			st = HAL_SPI_TransmitReceive_IT(&hspi1, out, in, len);
		else if (rdwr == (MPSSE_DO_WRITE))
			st = HAL_SPI_Transmit_IT(&hspi1, out, len);
		else
			st = HAL_SPI_Receive_IT(&hspi1, in, len);
		if (st != HAL_OK)
			error_with_led();
	}
	if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY) {
		spi1_deinit();
		read = false;

		if (cmd & MPSSE_DO_READ) {
			/* Push length */
			tud_vendor_write(&low, 1);
			tud_vendor_write(&high, 1);
			/* Push buffer */
			tud_vendor_write(in, len);
			tud_vendor_write_flush();
		}

		return true;
	}

	return false;
}

static void ftdi_set_clk_pin(uint8_t val)
{
	uint8_t bank = val / 8;

	if (bank > 1)
		/* Icorrect value */
		error_with_led();

	clk_pin = val;
}

static void ftdi_get_clk_gpio(const struct gpio_desc **desc,
			      bool *dir, bool *val)
{
	const struct gpio_desc *map;
	uint8_t bank, pin;

	bank = clk_pin / 8;
	pin = clk_pin % 8;

	if (bank == 0) {
		map = low_pin_mapping;
		*dir = gpio_low_dir & TU_BIT(pin);
		*val = gpio_low_val & TU_BIT(pin);
	} else {
		map = high_pin_mapping;
		*dir = gpio_high_dir & TU_BIT(pin);
		*val = gpio_high_val & TU_BIT(pin);
	}
	*desc = &map[pin];
}

static bool ftdi_clk_bytes(uint8_t low, uint8_t high)
{
	static bool started;

	const struct gpio_desc *gpio;
	bool dir, val;
	uint16_t num, i;

	/* Get clock GPIO bank, pin, and current direction and value */
	ftdi_get_clk_gpio(&gpio, &dir, &val);

	if (!started) {
		bool transition;

		num = (((uint16_t)high << 8) | low);
		/* Plus 1 according to FTDI datasheet */
		num += 1;
		/* In bits */
		num *= 8;
		/* Rising and falling edge of clock pulses */
		num *= 2;

		/* Start from 0 */
		i = 0;
		transition = false;

		if (dir && val) {
			/* Account 2 edge transitions */
			transition = true;
			num += 2;
			i = 1;
		}

		if (num > DMA_GPIO_NUM)
			error_with_led();

		/* Init timer and clock source */
		tim1_init(clk_freq);

		if (!dir) {
			/* Init GPIO as a clock pin */
			set_gpio(gpio->bank, gpio->pin, true, false);
		} else if (transition) {
			/* Reset first */
			dma_gpio_arr[0] = (gpio->pin << 16);
			/* Set last */
			dma_gpio_arr[num - 1] = gpio->pin;
		}
		for (; i < num; i += 2) {
			/* Set */
			dma_gpio_arr[i] = gpio->pin;
			/* Reset */
			dma_gpio_arr[i+1] = (gpio->pin << 16);
		}

		/*
		 * Start DMA and timer to the BSRR gpio register.
		 * See the RM0008 Reference Manual 9.2.5 for details.
		 */
		HAL_DMA_Start_IT(&hdma_tim1_up, (uintptr_t)dma_gpio_arr,
				 (uintptr_t)&(GPIOA->BSRR), num);
		HAL_TIM_Base_Start(&htim1);
		__HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
		started = true;
	}
	if (hdma_tim1_up.State != HAL_DMA_STATE_READY)
		return false;

	/* DMA completed, stop timer */
	HAL_TIM_Base_Stop(&htim1);
	__HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);
	HAL_TIM_Base_DeInit(&htim1);
	/* Restore GPIO */
	if (!dir)
		set_gpio(gpio->bank, gpio->pin, false, false);

	started = false;

	/* Done */
	return true;
}

static void ftdi_task(void)
{
	static uint8_t cmd, buf[2];

	if (!cmd) {
		if (!tud_vendor_available())
			return;
		tud_vendor_read(&cmd, 1);
	}
	if (cmd == TCK_DIVISOR) {
		if (tud_vendor_available() < 2)
			return;

		tud_vendor_read(buf, 2);
		ftdi_set_tck_divisor(buf[0], buf[1]);
		cmd = 0;
	} else if (cmd == SET_BITS_LOW) {
		if (tud_vendor_available() < 2)
			return;

		tud_vendor_read(buf, 2);
		ftdi_set_bits_low(buf[0], buf[1]);
		cmd = 0;
	} else if (cmd == SET_BITS_HIGH) {
		if (tud_vendor_available() < 2)
			return;

		tud_vendor_read(buf, 2);
		ftdi_set_bits_high(buf[0], buf[1]);
		cmd = 0;
	} else if (cmd == SET_CLK_PIN) {
		/* Custom command */
		if (tud_vendor_available() < 1)
			return;

		tud_vendor_read(buf, 1);
		ftdi_set_clk_pin(buf[0]);
		cmd = 0;
	} else if (cmd & (MPSSE_DO_WRITE|MPSSE_DO_READ) ||
		   cmd == CLK_BYTES) {
		static bool got_len;
		bool done;

		if (!got_len && tud_vendor_available() < 2)
			return;
		else if (!got_len) {
			tud_vendor_read(buf, 2);
			got_len = true;
		}
		if (cmd & (MPSSE_DO_WRITE|MPSSE_DO_READ))
			done = ftdi_mpsse_write_read(cmd, buf[0], buf[1]);
		else
			done = ftdi_clk_bytes(buf[0], buf[1]);
		if (done) {
			cmd = 0;
			got_len = false;
		}
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

__attribute__((noreturn))
static void error_with_led(void)
{
	blink_interval_ms = BLINK_ERROR;
	while (1) {
		led_blinking_task();
	}
}
