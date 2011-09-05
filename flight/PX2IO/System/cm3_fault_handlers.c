/*
 * cm3_fault_handlers.c
 *
 *  Created on: Apr 24, 2011
 *      Author: msmith
 */

#include <stdint.h>
#include "dcc_stdio.h"
#include "stm32f10x.h"

#define FAULT_TRAMPOLINE(_vec)										\
__attribute__((naked))												\
void																\
_vec##_Handler(void)												\
{																	\
     __asm(  ".syntax unified\n"									\
                     "MOVS   R0, #4  \n"							\
                     "MOV    R1, LR  \n"							\
                     "TST    R0, R1  \n"							\
                     "BEQ    1f    \n"								\
                     "MRS    R0, PSP \n"							\
                     "B      " #_vec "_Handler2      \n"			\
             "1:  \n"												\
                     "MRS    R0, MSP \n"							\
                     "B      " #_vec "_Handler2      \n"			\
             ".syntax divided\n");									\
}																	\
struct hack

struct cm3_frame {
	uint32_t	r0;
	uint32_t	r1;
	uint32_t	r2;
	uint32_t	r3;
	uint32_t	r12;
	uint32_t	lr;
	uint32_t	pc;
	uint32_t	psr;
};

FAULT_TRAMPOLINE(HardFault);
FAULT_TRAMPOLINE(BusFault);
FAULT_TRAMPOLINE(UsageFault);

/* this is a hackaround to avoid an issue where dereferencing SCB seems to result in bad codegen and a link error */
#define SCB_REG(_reg)	(*(uint32_t *)&(SCB->_reg))

void
HardFault_Handler2(struct cm3_frame *frame)
{
	dbg_write_str("\nHARD FAULT");
	dbg_write_hex32(frame->pc);
	dbg_write_char('\n');
	dbg_write_hex32(SCB_REG(HFSR));
	dbg_write_char('\n');
	for (;;);
}

void
BusFault_Handler2(struct cm3_frame *frame)
{
	dbg_write_str("\nBUS FAULT @ 0x");
	dbg_write_hex32(frame->pc);
	dbg_write_str("  CFSR 0x");
	dbg_write_hex32(SCB_REG(CFSR));
	dbg_write_str("  BFAR 0x");
	dbg_write_hex32(SCB_REG(BFAR));
	for (;;);
}

void
UsageFault_Handler2(struct cm3_frame *frame)
{
	dbg_write_str("\nUSAGE FAULT @ 0x");
	dbg_write_hex32(frame->pc);
	dbg_write_str("  CFSR 0x");
	dbg_write_hex32(SCB_REG(CFSR));
	for (;;);
}

void	__cyg_profile_func_enter(void *func, void *caller) __attribute__((naked, no_instrument_function));
void	__cyg_profile_func_exit(void *func, void *caller)  __attribute__((naked, no_instrument_function));
static void StackOverflow_Handler(uint32_t func, uint32_t caller) __attribute__((used, no_instrument_function));

static void
StackOverflow_Handler(uint32_t func, uint32_t caller)
{
	dbg_write_str("\nSTACK OVERFLOW");
	dbg_write_hex32(func);
	dbg_write_char('\n');
	dbg_write_hex32(caller);
	dbg_write_char('\n');
	for (;;);
}

void
__cyg_profile_func_enter(void *func, void *caller)
{
	asm volatile (
			"    mrs	r2, ipsr	\n"
			"    cmp    r2, #0		\n"
			"    bne    L__out		\n"		/* ignore this test if we are in interrupt mode */
			"    cmp	sp, r10		\n"
			"    bgt    L__out	 	\n"		/* stack is above limit and thus OK */
			"    cpsid	i			\n"		/* disable interrupts, we're toast now */
			"    mov    r2, r10		\n"		/* push the stack back up 64 bytes XXX this is ho-key */
			"    add	r2, r2, #64	\n"		/* we should probably switch to the MSP and run from there ... */
			"    mov	sp, r2		\n"
			"    b      StackOverflow_Handler\n"
			"L__out:				\n"
			"    bx		lr			\n"
			);
}

void
__cyg_profile_func_exit(void *func, void *caller)
{
	asm volatile("bx lr");
}

