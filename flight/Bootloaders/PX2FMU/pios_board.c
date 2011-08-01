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
 * Telemetry USART
 */
const struct pios_usart_cfg pios_usart_telem_cfg = USART2_CONFIG(PIOS_COM_TELEM_BAUDRATE);

/*
 * Debug USART
 */
const struct pios_usart_cfg pios_usart_aux_cfg = USART1_CONFIG(PIOS_COM_AUX_BAUDRATE);

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
		PIOS_COM_Init(&pios_com_telem_rf_id, &pios_usart_com_driver, usart_id)) {
		PIOS_DEBUG_Assert(0);
	}
	if (PIOS_USART_Init(&usart_id, &pios_usart_aux_cfg) ||
		PIOS_COM_Init(&pios_com_aux_id, &pios_usart_com_driver, usart_id)) {
		PIOS_DEBUG_Assert(0);
	}

}

/**
 * @}
 */
