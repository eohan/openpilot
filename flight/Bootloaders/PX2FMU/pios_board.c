/**
 ******************************************************************************
 * @addtogroup OpenPilotBL OpenPilot BootLoader
 * @{
 *
 * @file       pios_board.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Defines board specific static initialisers for hardware for the PX2FMU board.
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/* 
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation; either version 3 of the License, or 
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for more details.
 * 
 * You should have received a copy of the GNU General Public License along 
 * with this program; if not, write to the Free Software Foundation, Inc., 
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <pios.h>
#include "pios_usart_priv.h"
#include "pios_com_priv.h"

/*
 * Clocking
 *
 * Note that it would be nice to run the F2 with a 2MHz PLL input, but the VCO output
 * needs to be 240MHz if we are to have a USB clock, and the STM32 RCC driver (erroneously)
 * confuses the PLL_N constraint with the VCO output frequency constraint.
 */
const struct pios_clock_cfg px2fmu_clock_config = {
		.source				= RCC_PLLSource_HSE,
		.refclock_frequency = HSE_VALUE,
		.pll_m				= HSE_VALUE / 2000000,	// assumes HSE_VALUE is whole number of MHz, gives 1MHz PLL input
		.pll_n				= 240,					// gives 240MHz VCO output
		.pll_p				= 2,					// gives 120MHz SYSCLK
		.pll_q				= 5,					// gives 48MHz USB clock
		.hclk_prescale		= RCC_SYSCLK_Div1,		// AHB at 120MHz
		.pclk1_prescale		= RCC_HCLK_Div4,		// APB1 at 30MHz
		.pclk2_prescale		= RCC_HCLK_Div2,		// APB2 at 60MHz
		.flash_latency		= FLASH_Latency_5,		// for 3.3V operation
};

const struct pios_clock_cfg px4fmu_clock_config = {
		.source				= RCC_PLLSource_HSE,
		.refclock_frequency = HSE_VALUE,
		.pll_m				= HSE_VALUE / 2000000,	// assumes HSE_VALUE is an even number of MHz, gives 2MHz PLL input
		.pll_n				= 168,					// gives 336MHz VCO output
		.pll_p				= 2,					// gives 168MHz SYSCLK
		.pll_q				= 7,					// gives 48MHz USB clock
		.hclk_prescale		= RCC_SYSCLK_Div1,		// AHB at 168MHz
		.pclk1_prescale		= RCC_HCLK_Div4,		// APB1 at 42MHz
		.pclk2_prescale		= RCC_HCLK_Div2,		// APB2 at 84MHz
		.flash_latency		= FLASH_Latency_7,		// for 3.3V operation
};

/*
 * Telemetry USART
 */
const struct pios_usart_cfg pios_usart_telem_cfg = USART2_CONFIG(PIOS_COM_TELEM_BAUDRATE);
#define PIOS_COM_TELEM_RF_RX_BUF_LEN 192
#define PIOS_COM_TELEM_RF_TX_BUF_LEN 192

static uint8_t pios_com_telem_rf_rx_buffer[PIOS_COM_TELEM_RF_RX_BUF_LEN];
static uint8_t pios_com_telem_rf_tx_buffer[PIOS_COM_TELEM_RF_TX_BUF_LEN];


/*
 * Debug USART
 */
const struct pios_usart_cfg pios_usart_aux_cfg = USART1_CONFIG(PIOS_COM_AUX_BAUDRATE);
#define PIOS_COM_AUX_RX_BUF_LEN 64
#define PIOS_COM_AUX_TX_BUF_LEN 64

static uint8_t pios_com_aux_rx_buffer[PIOS_COM_AUX_RX_BUF_LEN];
static uint8_t pios_com_aux_tx_buffer[PIOS_COM_AUX_TX_BUF_LEN];


uint32_t pios_com_telem_rf_id;
uint32_t pios_com_aux_id;

/**
 * PIOS_Board_Init()
 * initializes all the core subsystems on this specific hardware
 * called from System/openpilot.c
 */
void PIOS_Board_Init(void)
{
	uint32_t	usart_id;

	// XXX is any of this crap necessary?
	/* Enable Prefetch Buffer */
	FLASH_PrefetchBufferCmd(ENABLE);

	/* Flash 2 wait state */
	FLASH_SetLatency(FLASH_Latency_2);

	/* configure the USB VBUS detect pin with an internal pulldown */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = PIOS_USB_DETECT_GPIO_PIN;
	GPIO_Init(PIOS_USB_DETECT_GPIO_PORT, &GPIO_InitStructure);

	/* configure the USARTs */
	if (PIOS_USART_Init(&usart_id, &pios_usart_telem_cfg) ||
		PIOS_COM_Init(&pios_com_telem_rf_id, &pios_usart_com_driver, 
			      usart_id,
			      pios_com_telem_rf_rx_buffer, sizeof(pios_com_telem_rf_rx_buffer), 
			      pios_com_telem_rf_tx_buffer, sizeof(pios_com_telem_rf_tx_buffer))) {
		PIOS_DEBUG_Assert(0);
	}
	if (PIOS_USART_Init(&usart_id, &pios_usart_aux_cfg) ||
		PIOS_COM_Init(&pios_com_aux_id, &pios_usart_com_driver,
			      usart_id,
			      pios_com_aux_rx_buffer, sizeof(pios_com_aux_rx_buffer),
			      pios_com_aux_tx_buffer, sizeof(pios_com_aux_tx_buffer))) {
		PIOS_DEBUG_Assert(0);
	}

}

/**
 * @}
 */
