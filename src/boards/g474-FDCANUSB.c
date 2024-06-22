/*

The MIT License (MIT)

Copyright (c) 2024 Filics,
			  Igor Knippenberg <knippenberg@filics.eu>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include "board.h"
#include "config.h"
#include "device.h"
#include "gpio.h"
#include "led.h"
#include "usbd_gs_can.h"

static void fdcanusb_gpio_init_termination(void)
{
	HAL_GPIO_WritePin(TERM_GPIO_Port, TERM_Pin, GPIO_INIT_STATE(TERM_Active_High));

	GPIO_InitTypeDef GPIO_InitStruct = {
		.Pin = TERM_Pin,
		.Mode = TERM_Mode,
		.Pull = GPIO_NOPULL,
		.Speed = GPIO_SPEED_FREQ_LOW,
		.Alternate = 0};
	HAL_GPIO_Init(TERM_GPIO_Port, &GPIO_InitStruct);
}

static void fdcanusb_termination_set(can_data_t *channel,
								   enum gs_can_termination_state state)
{
	UNUSED(channel);

#if (TERM_Active_High == 1)
	#define TERM_ON	 GPIO_PIN_SET
	#define TERM_OFF GPIO_PIN_RESET
#else
	#define TERM_ON	 GPIO_PIN_RESET
	#define TERM_OFF GPIO_PIN_SET
#endif

	HAL_GPIO_WritePin(TERM_GPIO_Port, TERM_Pin, (state ? TERM_ON : TERM_OFF));
}

static void fdcanusb_setup(USBD_GS_CAN_HandleTypeDef *hcan)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	UNUSED(hcan);

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* CAN RX LED */
	HAL_GPIO_WritePin(LEDRX_GPIO_Port, LEDRX_Pin, GPIO_INIT_STATE(LEDRX_Active_High));
	GPIO_InitStruct.Pin = LEDRX_Pin;
	GPIO_InitStruct.Mode = LEDRX_Mode;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LEDRX_GPIO_Port, &GPIO_InitStruct);

	/* CAN TX LED */
	HAL_GPIO_WritePin(LEDTX_GPIO_Port, LEDTX_Pin, GPIO_INIT_STATE(LEDTX_Active_High));
	GPIO_InitStruct.Pin = LEDTX_Pin;
	GPIO_InitStruct.Mode = LEDTX_Mode;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LEDTX_GPIO_Port, &GPIO_InitStruct);

	/* Power LED Pin */
	HAL_GPIO_WritePin(LEDPWR_GPIO_Port, LEDPWR_Pin, GPIO_INIT_STATE(LEDPWR_Active_High));
	GPIO_InitStruct.Pin = LEDPWR_Pin;
	GPIO_InitStruct.Mode = LEDPWR_Mode;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LEDPWR_GPIO_Port, &GPIO_InitStruct);

	/* Heartbeat (COM) LED Pin */
	HAL_GPIO_WritePin(LEDHEARTBEAT_GPIO_Port, LEDHEARTBEAT_Pin, 0);
	GPIO_InitStruct.Pin = LEDHEARTBEAT_Pin;
	GPIO_InitStruct.Mode = LEDHEARTBEAT_Mode;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LEDHEARTBEAT_GPIO_Port, &GPIO_InitStruct);

	/* STB Transceiver Standby Pin */
	HAL_GPIO_WritePin(nCANSTBY_Port, nCANSTBY_Pin, GPIO_INIT_STATE(nCANSTBY_Active_High));
	GPIO_InitStruct.Pin = nCANSTBY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(nCANSTBY_Port, &GPIO_InitStruct);

	/* SHDN Transceiver Shutdown Pin */
	HAL_GPIO_WritePin(SHDN_Port, SHDN_Pin, GPIO_INIT_STATE(SHDN_Active_High));
	GPIO_InitStruct.Pin = SHDN_Pin;
	GPIO_InitStruct.Mode = SHDN_Mode;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SHDN_Port, &GPIO_InitStruct);

	/* FDCAN */
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {
		.PeriphClockSelection = RCC_PERIPHCLK_FDCAN,
		.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL,
	};

	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	__HAL_RCC_FDCAN_CLK_ENABLE();

	/* FDCAN2_RX, FDCAN2_TX */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	fdcanusb_gpio_init_termination();
}

static void fdcanusb_phy_power_set(can_data_t *channel, bool enable)
{
	UNUSED(channel);

	if (enable)
	{
#ifdef nCANSTBY_Pin
		HAL_GPIO_WritePin(nCANSTBY_Port, nCANSTBY_Pin,
						  !GPIO_INIT_STATE(nCANSTBY_Active_High));
#endif
	}
	else
	{
#ifdef nCANSTBY_Pin
		HAL_GPIO_WritePin(nCANSTBY_Port, nCANSTBY_Pin,
						  GPIO_INIT_STATE(nCANSTBY_Active_High));
#endif
	}
}

const struct BoardConfig config = {
	.setup = fdcanusb_setup,
	.phy_power_set = fdcanusb_phy_power_set,
	.termination_set = fdcanusb_termination_set,
	.channels[0] = {
		.interface = FDCAN2,
		.leds = {
			[LED_RX] = {
				.port = LEDRX_GPIO_Port,
				.pin = LEDRX_Pin,
				.active_high = LEDRX_Active_High,
			},
			[LED_TX] = {
				.port = LEDTX_GPIO_Port,
				.pin = LEDTX_Pin,
				.active_high = LEDTX_Active_High,
			},
			[LED_HEARTBEAT] = {
				.port = LEDHEARTBEAT_GPIO_Port,
				.pin = LEDHEARTBEAT_Pin,
				.active_high = LEDHEARTBEAT_Active_High,
			},
		},
	}};
