/*
 * Startup code for the STM32F2xx.
 */

#include <string.h>

/* prototype for main() that tells us not to worry about it possibly returning */
extern int main(void) __attribute__((noreturn));

/* prototype our _main() to avoid prolog/epilog insertion and other related junk */
void _main(void) __attribute__((noreturn)) __attribute__((naked));

/* BSS symbols XXX should have a header that defines all of these */
extern char		_sbss, _ebss;

/* DATA symbols XXX should have a header that defines all of these */
extern char		_sidata, _sdata, _edata;

/* The bootstrap/IRQ stack XXX should define size somewhere else */
char			irq_stack[1024] __attribute__((section(".irqstack"))) __attribute__((used));

void
_main(void)
{
	/* copy initialised data from flash to RAM */
	memcpy(&_sdata, &_sidata, &_edata - &_sdata);

	/* zero the BSS */
	memset(&_sbss, 0, &_ebss - &_sbss);

	/* fill the IRQ/bootstrap stack with a watermark pattern so we can measure how much is used */
	memset(&irq_stack, 0xa5, sizeof(irq_stack));

	/* call main */
	(void)main();
}

