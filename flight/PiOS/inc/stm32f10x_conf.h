/**
  ******************************************************************************
  * @file    Project/Template/stm32f10x_conf.h 
  * @author  MCD Application Team
  * @version V3.1.2
  * @date    09/28/2009
  * @brief   Library configuration file.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_CONF_H
#define __STM32F10x_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Uncomment the line below to enable peripheral header file inclusion */
#include "stm32f10x_adc.h"
#include "stm32f10x_bkp.h"
/* #include "stm32f10x_can.h" */
#include "stm32f10x_crc.h"
#include "stm32f10x_dac.h"
/* #include "stm32f10x_dbgmcu.h" */
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_flash.h"
/* #include "stm32f10x_fsmc.h" */
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
/* #include "stm32f10x_iwdg.h" */
#include "stm32f10x_pwr.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_rtc.h"
/* #include "stm32f10x_sdio.h" */
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_wwdg.h"
#include "misc.h"		/* High level functions for NVIC and SysTick (add-on to CMSIS functions) */

/* Exported types ------------------------------------------------------------*/
/* Public structures */
struct pios_clock_cfg {
	/**
	 * RCC_PLLSource_HSI_Div2 always selects HSI/2 as the PLL
	 * clock source.
	 * On Value Line devices, use RCC_PLLSource_PREDIV1 and
	 * set refclock_prescale to a value between 1 and 16.  Otherwise
	 * select one of RCC_PLLSource_HSE_Div1 or RCC_PLLSource_HSE_Div2.
	 */
	uint32_t source;

	/**
	 * The reference clock frequency, not required if
	 * source is RCC_PLLSource_HSI_Div2.
	 */
	uint32_t refclock_frequency;

	/**
	 * The desired reference clock prescaler.
	 * For Value Line devices this can be 1-16,
	 * ignored for other devices.
	 */
	uint32_t refclock_prescale;

	/** The PLL multiplier value.  One of RCC_PLLMul_* */
	uint32_t pll_multiply;

	/** Divider from SYSCLK to HCLK, a value from RCC_SYSCLK_Div* */
	uint32_t hclk_prescale;

	/** Divider from HCLK to PCLK1, a value from RCC_HCLK_Div* */
	uint32_t pclk1_prescale;

	/** Divider from HCLK to PCLK2, a value from RCC_HCLK_Div* */
	uint32_t pclk2_prescale;

	/** Divider from PCLK to ADCCLK, a value from RCC_PCLK2_Div* */
	uint32_t adc_prescale;
};

/* Exported constants --------------------------------------------------------*/
/* Uncomment the line below to expanse the "assert_param" macro in the 
   Standard Peripheral Library drivers code */
//#define USE_FULL_ASSERT    1

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
* The assert_param macro is used for function's parameters check.
* \param[in]  expr: If expr is false, it calls assert_failed function
*   which reports the name of the source file and the source
*   line number of the call that failed. 
*   If expr is true, it returns no value.
* \retval None
*/
#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
extern void assert_failed(uint8_t * file, uint32_t line);
#else
#define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F10x_CONF_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
