/*
    FreeRTOS V7.0.2 - Copyright (C) 2011 Real Time Engineers Ltd.
	

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ARM Cortex-M4F port.
 *----------------------------------------------------------*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* For backward compatibility, ensure configKERNEL_INTERRUPT_PRIORITY is
defined.  The value should also ensure backward compatibility.
FreeRTOS.org versions prior to V4.4.0 did not include this definition. */
#ifndef configKERNEL_INTERRUPT_PRIORITY
	#define configKERNEL_INTERRUPT_PRIORITY 255
#endif

/* Constants required to manipulate the NVIC. */
#define portNVIC_SYSTICK_CTRL		( ( volatile unsigned long *) 0xe000e010 )
#define portNVIC_SYSTICK_LOAD		( ( volatile unsigned long *) 0xe000e014 )
#define portNVIC_INT_CTRL			( ( volatile unsigned long *) 0xe000ed04 )
#define portNVIC_SYSPRI2			( ( volatile unsigned long *) 0xe000ed20 )
#define portNVIC_SYSTICK_CLK		0x00000004
#define portNVIC_SYSTICK_INT		0x00000002
#define portNVIC_SYSTICK_ENABLE		0x00000001
#define portNVIC_PENDSVSET			0x10000000
#define portNVIC_PENDSV_PRI			( ( ( unsigned long ) configKERNEL_INTERRUPT_PRIORITY ) << 16 )
#define portNVIC_SYSTICK_PRI		( ( ( unsigned long ) configKERNEL_INTERRUPT_PRIORITY ) << 24 )

/* Constants required to set up the initial stack. */
#define portINITIAL_XPSR			( 0x01000000 )
#define portINITIAL_FPSCR			( (1<<25) )			/* select Default NaN mode XXX this is arguably not appropriate here */

/* The priority used by the kernel is assigned to a variable to make access
from inline assembler easier. */
const unsigned long ulKernelPriority = configKERNEL_INTERRUPT_PRIORITY;

/* Each task maintains its own interrupt status in the critical nesting
variable. */
static unsigned portBASE_TYPE uxCriticalNesting = 0xaaaaaaaa;

/*
 * Setup the timer to generate the tick interrupts.
 */
static void prvSetupTimerInterrupt( void );

/*
 * Exception handlers.
 */
void xPortPendSVHandler( void ) __attribute__ (( naked )) __attribute__((no_instrument_function));
void xPortSysTickHandler( void ) __attribute__((no_instrument_function));
void vPortSVCHandler( void ) __attribute__ (( naked )) __attribute__((no_instrument_function));

/*
 * Start first task is a separate function so it can be tested in isolation.
 */
void vPortStartFirstTask( void ) __attribute__ (( naked )) __attribute__((no_instrument_function));

/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
portSTACK_TYPE *pxPortInitialiseStack( portSTACK_TYPE *pxTopOfStack, portSTACK_TYPE *pxStartOfStack, pdTASK_CODE pxCode, void *pvParameters )
{
	/*
	 * Create an extended stack frame plus manual save area as would be
	 * saved by xPortPendSVHandler on entry.
	 */

	/* automatically stacked state */
	*--pxTopOfStack = 0;								/* reserved */
	*--pxTopOfStack = portINITIAL_FPSCR;				/* FPSCR */
	*--pxTopOfStack = 0;								/* s15 */
	*--pxTopOfStack = 0;								/* s14 */
	*--pxTopOfStack = 0;								/* s13 */
	*--pxTopOfStack = 0;								/* s12 */
	*--pxTopOfStack = 0;								/* s11 */
	*--pxTopOfStack = 0;								/* s10 */
	*--pxTopOfStack = 0;								/* s9 */
	*--pxTopOfStack = 0;								/* s8 */
	*--pxTopOfStack = 0;								/* s7 */
	*--pxTopOfStack = 0;								/* s6 */
	*--pxTopOfStack = 0;								/* s5 */
	*--pxTopOfStack = 0;								/* s4 */
	*--pxTopOfStack = 0;								/* s3 */
	*--pxTopOfStack = 0;								/* s2 */
	*--pxTopOfStack = 0;								/* s1 */
	*--pxTopOfStack = 0;								/* s0 */
	*--pxTopOfStack = portINITIAL_XPSR;					/* xPSR */
	*--pxTopOfStack = ( portSTACK_TYPE ) pxCode;		/* pc */
	*--pxTopOfStack = 0;								/* lr */
	*--pxTopOfStack = 0;								/* r12 */
	*--pxTopOfStack = 0;								/* r3 */
	*--pxTopOfStack = 0;								/* r2 */
	*--pxTopOfStack = 0;								/* r1 */
	*--pxTopOfStack = ( portSTACK_TYPE ) pvParameters;	/* r0 */

	/* manually stacked state */
	*--pxTopOfStack = 0;								/* s31 */
	*--pxTopOfStack = 0;								/* s30 */
	*--pxTopOfStack = 0;								/* s29 */
	*--pxTopOfStack = 0;								/* s28 */
	*--pxTopOfStack = 0;								/* s27 */
	*--pxTopOfStack = 0;								/* s26 */
	*--pxTopOfStack = 0;								/* s25 */
	*--pxTopOfStack = 0;								/* s24 */
	*--pxTopOfStack = 0;								/* s23 */
	*--pxTopOfStack = 0;								/* s22 */
	*--pxTopOfStack = 0;								/* s21 */
	*--pxTopOfStack = 0;								/* s20 */
	*--pxTopOfStack = 0;								/* s19 */
	*--pxTopOfStack = 0;								/* s18 */
	*--pxTopOfStack = 0;								/* s17 */
	*--pxTopOfStack = 0;								/* s16 */
	*--pxTopOfStack = 0;								/* r11 */
	*--pxTopOfStack = ( portSTACK_TYPE ) pxStartOfStack + 200; /* r10 (base of stack) reserve room for a full context save */
	*--pxTopOfStack = 0;								/* r9 */
	*--pxTopOfStack = 0;								/* r8 */
	*--pxTopOfStack = 0;								/* r7 */
	*--pxTopOfStack = 0;								/* r6 */
	*--pxTopOfStack = 0;								/* r5 */
	*--pxTopOfStack = 0;								/* r4 */

	return pxTopOfStack;
}
/*-----------------------------------------------------------*/

void vPortSVCHandler( void )
{
	__asm volatile (
			"	ldr	r3, pxCurrentTCBConst2		\n" /* pointer to pxCurrentTCB */
			"	ldr r1, [r3]					\n" /* pointer to current TCB */
			"	ldr r0, [r1]					\n" /* first member of pxCurrentTCB is the current task stack pointer */
			"	vldmia r0!, {s16-s31}			\n"	/* pop the manually-stacked FP registers */
			"	ldmia r0!, {r4-r11}				\n" /* pop the manually-stacked GP registers */
			"	msr psp, r0						\n" /* reload the program stack pointer */
			"	mov r0, #0 						\n" /* reset to priority 0 */
			"	msr	basepri, r0					\n"
			"	orr lr, #0xed					\n"	/* return to thread mode with extended stackframe, force CONTROL.FPCA to 1 */
			"	bx lr							\n"
			"									\n"
			"	.align 2						\n"
			"pxCurrentTCBConst2: .word pxCurrentTCB	\n"
	);
}
/*-----------------------------------------------------------*/

void vPortStartFirstTask( void )
{
	__asm volatile(
			" ldr r0, =0xE000ED08 	\n" /* Use the NVIC offset register to locate the stack. */
			" ldr r0, [r0] 			\n"
			" ldr r0, [r0] 			\n"
			" msr msp, r0			\n" /* Set the msp back to the start of the stack. */
			" cpsie i				\n" /* Globally enable interrupts. */
			" svc 0					\n" /* System call to start first task. */
			" nop					\n"
	);
}
/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
portBASE_TYPE xPortStartScheduler( void )
{
	/* Make PendSV, CallSV and SysTick the same priroity as the kernel. */
	*(portNVIC_SYSPRI2) |= portNVIC_PENDSV_PRI;
	*(portNVIC_SYSPRI2) |= portNVIC_SYSTICK_PRI;

	/* Start the timer that generates the tick ISR.  Interrupts are disabled
	here already. */
	prvSetupTimerInterrupt();

	/* Initialise the critical nesting count ready for the first task. */
	uxCriticalNesting = 0;

	/* Start the first task. */
	vPortStartFirstTask();

	/* Should not get here! */
	return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* It is unlikely that the CM3 port will require this function as there
	is nothing to return to.  */
}
/*-----------------------------------------------------------*/

void vPortYieldFromISR( void )
{
	/* Set a PendSV to request a context switch. */
	*(portNVIC_INT_CTRL) = portNVIC_PENDSVSET;
}
/*-----------------------------------------------------------*/

void vPortEnterCritical( void )
{
	portDISABLE_INTERRUPTS();
	uxCriticalNesting++;
}
/*-----------------------------------------------------------*/

void vPortExitCritical( void )
{
	uxCriticalNesting--;
	if( uxCriticalNesting == 0 )
	{
		portENABLE_INTERRUPTS();
	}
}
/*-----------------------------------------------------------*/

void xPortPendSVHandler( void )
{
	/* This is a naked function. */

	__asm volatile
	(
			"	mrs r0, psp							\n"	/* get the program stack pointer (base of automatic frame) in r0 */
			"										\n"
			"	ldr	r3, pxCurrentTCBConst			\n" /* Get the location of the current TCB. */
			"	ldr	r2, [r3]						\n"
			"										\n"
			"	stmdb r0!, {r4-r11}					\n" /* stack the remaining GP registers */
			"	vstmdb r0!, {s16-s31}				\n" /* stack the remaining FP registers */
			"	str r0, [r2]						\n" /* Save the new top of stack into the first member of the TCB. */
			"										\n"
			"	stmdb sp!, {r3, lr}					\n"	/* save r3 (pointer to pointer to the current TCB) and lr (exception return code) */
			"	mov r0, %0							\n"
			"	msr basepri, r0						\n"
			"	bl vTaskSwitchContext				\n"
			"	mov r0, #0							\n"
			"	msr basepri, r0						\n"
			"	ldmia sp!, {r3, lr}					\n"
			"										\n"	/* Restore the context, including the critical nesting count. */
			"	ldr r1, [r3]						\n"
			"	ldr r0, [r1]						\n" /* The first item in pxCurrentTCB is the task top of stack. */
			"	vldmia r0!, {s16-s31}				\n" /* pop the manually-stacked FP registers */
			"	ldmia r0!, {r4-r11}					\n" /* pop the manually-stacked GP registers */
			"	msr psp, r0							\n"	/* reload the program stack pointer */
			"	bx lr								\n"	/* perform an exception return as per the encoding in r14 */
			"										\n"
			"	.align 2							\n"
			"pxCurrentTCBConst: .word pxCurrentTCB	\n"
			::"i"(configMAX_SYSCALL_INTERRUPT_PRIORITY)
	);
}
/*-----------------------------------------------------------*/

void xPortSysTickHandler( void )
{
unsigned long ulDummy;

	/* If using preemption, also force a context switch. */
	#if configUSE_PREEMPTION == 1
		*(portNVIC_INT_CTRL) = portNVIC_PENDSVSET;
	#endif

	ulDummy = portSET_INTERRUPT_MASK_FROM_ISR();
	{
		vTaskIncrementTick();
	}
	portCLEAR_INTERRUPT_MASK_FROM_ISR( ulDummy );
}
/*-----------------------------------------------------------*/

/*
 * Setup the systick timer to generate the tick interrupts at the required
 * frequency.
 */
void prvSetupTimerInterrupt( void )
{
	/* Configure SysTick to interrupt at the requested rate. */
	*(portNVIC_SYSTICK_LOAD) = ( configCPU_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
	*(portNVIC_SYSTICK_CTRL) = portNVIC_SYSTICK_CLK | portNVIC_SYSTICK_INT | portNVIC_SYSTICK_ENABLE;
}
/*-----------------------------------------------------------*/

void	__cyg_profile_func_enter(void *func, void *caller) __attribute__((naked, no_instrument_function));
void	__cyg_profile_func_exit(void *func, void *caller)  __attribute__((naked, no_instrument_function));

void
__cyg_profile_func_enter(void *func, void *caller)
{
    asm volatile (
        "    mrs    r2, ipsr        \n"
        "    cmp    r2, #0          \n"
        "    bne    L__out          \n"             /* ignore this test if we are in interrupt mode */
        "    cmp    sp, r10         \n"
        "    bgt    L__out          \n"             /* stack is above limit and thus OK */
        "    mov    r2, #0xac       \n"             /* force a hard fault with a distinctive address 0x57ac ('stac') */
        "    add    r2, r2, #0x5700 \n"
        "    ldr    r2, [r2]        \n"
        "    b      .               \n"
        "L__out:                    \n"
        "    bx     lr              \n"
    );
}

void
__cyg_profile_func_exit(void *func, void *caller)
{
	asm volatile("bx lr");
}


