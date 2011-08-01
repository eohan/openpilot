/**
 *
 * @file      vectors_stm32f100.S
 * @author    Copyright (c) 2011 Michael Smith
 * @brief     STM32F100 vector table
 *
 */
    
	.syntax	unified
	.cpu	cortex-m3
	.fpu	softvfp
	.thumb

	.section .isr_vector,"a",%progbits
	.type	g_pfnVectors, %object
	.size	g_pfnVectors, .-g_pfnVectors
	.global g_pfnVectors
	
g_pfnVectors:
	// standard CM3 vectors
	.word	INITIAL_STACK_TOP
	.word	Reset_Handler
	.word	NMI_Handler
	.word	HardFault_Handler
	.word	MemManage_Handler
	.word	BusFault_Handler
	.word	UsageFault_Handler
	.word	0
	.word	0
	.word	0
	.word	0
	.word	vPortSVCHandler
	.word	DebugMon_Handler
	.word	0
	.word	xPortPendSVHandler
	.word	xPortSysTickHandler

	// ST device interrupt vectors
	.word	WWDG_IRQHandler
	.word	PVD_IRQHandler
	.word	TAMPER_IRQHandler
	.word	RTC_IRQHandler
	.word	FLASH_IRQHandler
	.word	RCC_IRQHandler
	.word	EXTI0_IRQHandler
	.word	EXTI1_IRQHandler
	.word	EXTI2_IRQHandler
	.word	EXTI3_IRQHandler
	.word	EXTI4_IRQHandler
	.word	DMA1_Channel1_IRQHandler
	.word	DMA1_Channel2_IRQHandler
	.word	DMA1_Channel3_IRQHandler
	.word	DMA1_Channel4_IRQHandler
	.word	DMA1_Channel5_IRQHandler
	.word	DMA1_Channel6_IRQHandler
	.word	DMA1_Channel7_IRQHandler
	.word	ADC1_2_IRQHandler
	.word	0
	.word	0
	.word	0
	.word	0
	.word	EXTI9_5_IRQHandler
	.word	TIM1_BRK_IRQHandler
	.word	TIM1_UP_IRQHandler
	.word	TIM1_TRG_COM_IRQHandler
	.word	TIM1_CC_IRQHandler
	.word	TIM2_IRQHandler
	.word	TIM3_IRQHandler
	.word	TIM4_IRQHandler
	.word	I2C1_EV_IRQHandler
	.word	I2C1_ER_IRQHandler
	.word	I2C2_EV_IRQHandler
	.word	I2C2_ER_IRQHandler
	.word	SPI1_IRQHandler
	.word	SPI2_IRQHandler
	.word	USART1_IRQHandler
	.word	USART2_IRQHandler
	.word	USART3_IRQHandler
	.word	EXTI15_10_IRQHandler
	.word	RTCAlarm_IRQHandler
	.word	CEC_IRQHandler
	.word	TIM12_IRQHandler
	.word	TIM13_IRQHandler
	.word	TIM14_IRQHandler
	.word	0
	.word	0
	.word	FSMC_IRQ_Handler
	.word	0
	.word	TIM5_IRQHandler
	.word	SPI3_IRQHandler
	.word	UART4_IRQHandler
	.word	UART5_IRQHandler
	.word	TIM6_DAC_IRQHandler
	.word	TIM7_IRQHandler
	.word	DMA2_Channel1_IRQHandler
	.word	DMA2_Channel2_IRQHandler
	.word	DMA2_Channel3_IRQHandler
	.word	DMA2_Channel4_5_IRQHandler
   
/*******************************************************************************
 *
 * Provide weak aliases for each Exception handler to the Default_Handler.
 * As they are weak aliases, any function with the same name will override
 * this definition.
 *
 *******************************************************************************/
    
	.weak  NMI_Handler
	.thumb_set NMI_Handler,Default_Handler

	.weak  HardFault_Handler
	.thumb_set HardFault_Handler,Default_Handler

	.weak  MemManage_Handler
	.thumb_set MemManage_Handler,Default_Handler

	.weak  BusFault_Handler
	.thumb_set BusFault_Handler,Default_Handler

	.weak  UsageFault_Handler
	.thumb_set UsageFault_Handler,Default_Handler

	.weak  vPortSVCHandler
	.thumb_set vPortSVCHandler,Default_Handler

	.weak  DebugMon_Handler
	.thumb_set DebugMon_Handler,Default_Handler

	.weak  xPortPendSVHandler
	.thumb_set xPortPendSVHandler,Default_Handler

	.weak  xPortSysTickHandler
	.thumb_set xPortSysTickHandler,Default_Handler

	.weak  WWDG_IRQHandler
	.thumb_set WWDG_IRQHandler,Default_Handler

	.weak  PVD_IRQHandler
	.thumb_set PVD_IRQHandler,Default_Handler

	.weak  TAMPER_IRQHandler
	.thumb_set TAMPER_IRQHandler,Default_Handler

	.weak  RTC_IRQHandler
	.thumb_set RTC_IRQHandler,Default_Handler

	.weak  FLASH_IRQHandler
	.thumb_set FLASH_IRQHandler,Default_Handler

	.weak  RCC_IRQHandler
	.thumb_set RCC_IRQHandler,Default_Handler

	.weak  EXTI0_IRQHandler
	.thumb_set EXTI0_IRQHandler,Default_Handler

	.weak  EXTI1_IRQHandler
	.thumb_set EXTI1_IRQHandler,Default_Handler

	.weak  EXTI2_IRQHandler
	.thumb_set EXTI2_IRQHandler,Default_Handler

	.weak  EXTI3_IRQHandler
	.thumb_set EXTI3_IRQHandler,Default_Handler

	.weak  EXTI4_IRQHandler
	.thumb_set EXTI4_IRQHandler,Default_Handler

	.weak  DMA1_Channel1_IRQHandler
	.thumb_set DMA1_Channel1_IRQHandler,Default_Handler

	.weak  DMA1_Channel2_IRQHandler
	.thumb_set DMA1_Channel2_IRQHandler,Default_Handler

	.weak  DMA1_Channel3_IRQHandler
	.thumb_set DMA1_Channel3_IRQHandler,Default_Handler

	.weak  DMA1_Channel4_IRQHandler
	.thumb_set DMA1_Channel4_IRQHandler,Default_Handler

	.weak  DMA1_Channel5_IRQHandler
	.thumb_set DMA1_Channel5_IRQHandler,Default_Handler

	.weak  DMA1_Channel6_IRQHandler
	.thumb_set DMA1_Channel6_IRQHandler,Default_Handler

	.weak  DMA1_Channel7_IRQHandler
	.thumb_set DMA1_Channel7_IRQHandler,Default_Handler

	.weak  ADC1_2_IRQHandler
	.thumb_set ADC1_2_IRQHandler,Default_Handler

	.weak  EXTI9_5_IRQHandler
	.thumb_set EXTI9_5_IRQHandler,Default_Handler

	.weak  TIM1_BRK_IRQHandler
	.thumb_set TIM1_BRK_IRQHandler,Default_Handler

	.weak  TIM1_UP_IRQHandler
	.thumb_set TIM1_UP_IRQHandler,Default_Handler

	.weak  TIM1_TRG_COM_IRQHandler
	.thumb_set TIM1_TRG_COM_IRQHandler,Default_Handler

	.weak  TIM1_CC_IRQHandler
	.thumb_set TIM1_CC_IRQHandler,Default_Handler

	.weak  TIM2_IRQHandler
	.thumb_set TIM2_IRQHandler,Default_Handler

	.weak  TIM3_IRQHandler
	.thumb_set TIM3_IRQHandler,Default_Handler

	.weak  TIM4_IRQHandler
	.thumb_set TIM4_IRQHandler,Default_Handler

	.weak  I2C1_EV_IRQHandler
	.thumb_set I2C1_EV_IRQHandler,Default_Handler

	.weak  I2C1_ER_IRQHandler
	.thumb_set I2C1_ER_IRQHandler,Default_Handler

	.weak  I2C2_EV_IRQHandler
	.thumb_set I2C2_EV_IRQHandler,Default_Handler

	.weak  I2C2_ER_IRQHandler
	.thumb_set I2C2_ER_IRQHandler,Default_Handler

	.weak  SPI1_IRQHandler
	.thumb_set SPI1_IRQHandler,Default_Handler

	.weak  SPI2_IRQHandler
	.thumb_set SPI2_IRQHandler,Default_Handler

	.weak  USART1_IRQHandler
	.thumb_set USART1_IRQHandler,Default_Handler

	.weak  USART2_IRQHandler
	.thumb_set USART2_IRQHandler,Default_Handler

	.weak  USART3_IRQHandler
	.thumb_set USART3_IRQHandler,Default_Handler

	.weak  EXTI15_10_IRQHandler
	.thumb_set EXTI15_10_IRQHandler,Default_Handler

	.weak  RTCAlarm_IRQHandler
	.thumb_set RTCAlarm_IRQHandler,Default_Handler

	.weak  CEC_IRQHandler
	.thumb_set  CEC_IRQHandler,Default_Handler

	.weak  TIM12_IRQHandler
	.thumb_set  TIM12_IRQHandler,Default_Handler

	.weak  TIM13_IRQHandler
	.thumb_set  TIM13_IRQHandler,Default_Handler

	.weak  TIM14_IRQHandler
	.thumb_set  TIM14_IRQHandler,Default_Handler

	.weak  FSMC_IRQ_Handler
	.thumb_set  FSMC_IRQ_Handler,Default_Handler

	.weak  TIM5_IRQHandler
	.thumb_set  TIM5_IRQHandler,Default_Handler

	.weak  SPI3_IRQHandler
	.thumb_set  SPI3_IRQHandler,Default_Handler

	.weak  UART4_IRQHandler
	.thumb_set  UART4_IRQHandler,Default_Handler

	.weak  UART5_IRQHandler
	.thumb_set  UART5_IRQHandler,Default_Handler

	.weak  TIM6_DAC_IRQHandler
	.thumb_set  TIM6_DAC_IRQHandler,Default_Handler

	.weak  TIM7_IRQHandler
	.thumb_set  TIM7_IRQHandler,Default_Handler

	.weak  DMA2_Channel1_IRQHandler
	.thumb_set  DMA2_Channel1_IRQHandler,Default_Handler

	.weak  DMA2_Channel2_IRQHandler
	.thumb_set  DMA2_Channel2_IRQHandler,Default_Handler

	.weak  DMA2_Channel3_IRQHandler
	.thumb_set  DMA2_Channel3_IRQHandler,Default_Handler

	.weak  DMA2_Channel4_5_IRQHandler
	.thumb_set  DMA2_Channel4_5_IRQHandler,Default_Handler
