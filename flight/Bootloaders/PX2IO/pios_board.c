/**
 ******************************************************************************
 *
 * @file       pios_board.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      Defines board specific static initializers for hardware for the PX2IO board.
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

#include <bl_fsm.h>

/**
 * I2C slave-only config structure.
 *
 * Eventually this should probably be shared/common with the regular config structure...
 */
struct i2c_slave_config {
	I2C_TypeDef			*regs;
	I2C_InitTypeDef		init;
	struct stm32_gpio	scl;
	struct stm32_gpio	sda;
};

/**
 * FMU interface port.
 *
 * Note that we poll this port, rather than using interrupts.
 */
static const struct i2c_slave_config i2c_config = {
		.regs = I2C1,
		.init = {
				.I2C_ClockSpeed				= 100000,
				.I2C_Mode					= I2C_Mode_I2C,
				.I2C_DutyCycle				= I2C_DutyCycle_2,
				.I2C_OwnAddress1			= 0x10,
				.I2C_Ack					= I2C_Ack_Enable,
				.I2C_AcknowledgedAddress	= I2C_AcknowledgedAddress_7bit,
		},
		.scl = {
				.gpio = GPIOB,
				.init = {
						.GPIO_Pin		= GPIO_Pin_6,
						.GPIO_Mode		= GPIO_Mode_AF_OD,
						.GPIO_Speed		= GPIO_Speed_10MHz,
				},
		},
		.sda = {
				.gpio = GPIOB,
				.init = {
						.GPIO_Pin		= GPIO_Pin_7,
						.GPIO_Mode		= GPIO_Mode_AF_OD,
						.GPIO_Speed		= GPIO_Speed_10MHz,
				},
		}
};

/*
 * Clocking
 */
const struct pios_clock_cfg px2io_clock_config = {
		.source				= RCC_PLLSource_PREDIV1,
		.refclock_frequency = HSE_VALUE,
		.refclock_prescale	= RCC_PREDIV1_Div2,
		.pll_multiply		= RCC_PLLMul_6,
		.hclk_prescale		= RCC_SYSCLK_Div1,
		.pclk1_prescale		= RCC_HCLK_Div1,
		.pclk2_prescale		= RCC_HCLK_Div1,
		.adc_prescale		= RCC_PCLK2_Div2,
};

void PIOS_Board_Init() {

	// initialise I2C pins
	GPIO_Init(i2c_config.sda.gpio, &(i2c_config.sda.init));
	GPIO_Init(i2c_config.scl.gpio, &(i2c_config.scl.init));

	// tell the FSM which port it's using
	i2c_fsm_attach(i2c_config.regs);

	// init the I2C interface
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
	I2C_DeInit(i2c_config.regs);
	I2C_Init(i2c_config.regs, &i2c_config.init);
	I2C_Cmd(i2c_config.regs, ENABLE);
	//I2C_SoftwareResetCmd(i2c_config.regs, ENABLE);
	//I2C_SoftwareResetCmd(i2c_config.regs, DISABLE);
}
