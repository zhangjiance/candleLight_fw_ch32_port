/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Hubert Denkmair
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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "board_layout.h"
#include "can.h"
#include "can_common.h"
#include "config.h"
#include "device.h"
#include "dfu.h"
#include "gpio.h"
#include "led.h"
#include "timer.h"
#include "usbd_conf.h"
#include "usbd_compat.h"
#include "usbd_desc.h"
#include "usbd_gs_can.h"

#include "status_led.h"

#include "board_contract.h"        /* BOARD_USB_BASE */

void initialise_monitor_handles(void);

/* src/can_ch32_adapter.c: performs the CAN controller start outside the USB
 * interrupt (can_drv_enable() only queues it). */
void can_drv_poll(can_data_t *channel);

static USBD_GS_CAN_HandleTypeDef hGS_CAN;
static USBD_HandleTypeDef hUSB = {0};

int main(void)
{
	if (IS_ENABLED(CONFIG_SEMIHOSTING))
		initialise_monitor_handles();

	HAL_Init();
	device_sysclock_config();


	board_setup(&hGS_CAN);
	timer_init();

	status_led_init();
	status_led_boot_anim();

	INIT_LIST_HEAD(&hGS_CAN.list_frame_pool);
	INIT_LIST_HEAD(&hGS_CAN.list_to_host);

	for (unsigned i = 0; i < ARRAY_SIZE(hGS_CAN.msgbuf); i++) {
		list_add_tail(&hGS_CAN.msgbuf[i].list, &hGS_CAN.list_frame_pool);
	}

	for (unsigned int i = 0; i < ARRAY_SIZE(hGS_CAN.channels); i++) {
		const struct board_channel_config *channel_config = &config.channel[i];
		const struct led_config *led_config = channel_config->leds;
		can_data_t *channel = &hGS_CAN.channels[i];

		can_channel_set_nr(channel, i);

		INIT_LIST_HEAD(&channel->list_from_host);

		led_init(&channel->leds,
				 led_config[LED_RX].port, led_config[LED_RX].pin, led_config[LED_RX].active_high,
				 led_config[LED_TX].port, led_config[LED_TX].pin, led_config[LED_TX].active_high);

		can_init(channel, channel_config);

		can_disable(&hGS_CAN, channel);
	}

	/*
	 * USB comes up last, like upstream candleLight: the gs_usb class arms its OUT
	 * endpoint as soon as the host configures the device, and that needs the frame
	 * pool and the channel bookkeeping above to exist already - otherwise the class
	 * finds an empty pool, never arms EP 0x02 and the host times out on every bulk
	 * OUT transfer.
	 */
	USBD_Init(&hUSB, (USBD_DescriptorsTypeDef *)&FS_Desc, DEVICE_FS);
	USBD_RegisterClass(&hUSB, &USBD_GS_CAN);
	USBD_GS_CAN_Init(&hGS_CAN, &hUSB);
	USBD_Start(&hUSB);

	while (1) {
		for (unsigned int i = 0; i < ARRAY_SIZE(hGS_CAN.channels); i++) {
			can_data_t *channel = &hGS_CAN.channels[i];

			can_drv_poll(channel);      /* deferred CAN controller start */
			CAN_SendFrame(&hGS_CAN, channel);
		}

		USBD_GS_CAN_ReceiveFromHost(&hUSB);
		USBD_GS_CAN_SendToHost(&hUSB);

		for (unsigned int i = 0; i < ARRAY_SIZE(hGS_CAN.channels); i++) {
			can_data_t *channel = &hGS_CAN.channels[i];

			CAN_ReceiveFrame(&hGS_CAN, channel);
			CAN_HandleError(&hGS_CAN, channel);
			led_update(&channel->leds);
		}

		status_led_tick();

		if (USBD_GS_CAN_DfuDetachRequested(&hUSB)) {
			dfu_run_bootloader();
		}

	}
}
