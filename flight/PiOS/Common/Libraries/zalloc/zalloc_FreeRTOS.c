/*
 * Implementation of pvPortMalloc and friends that uses the
 * Dillon zalloc/malloc allocator.
 *
 * (c) 2011 Michael Smith, All Rights Reserved
 */
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

#include <stdlib.h>

#include "zalloc_defs.h"

/*
 * Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
 * all the API functions to use the MPU wrappers.  That should only be done when
 * task.h is included from an application file.
 * */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE
# include "FreeRTOS.h"
# include "task.h"
#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/*
 * Symbols exported by the linker script telling us where the heap
 * and init stack are.
 */
extern char	_sheap;
extern char	_eheap;
extern char	_init_stack_end;
extern char	_init_stack_top;

extern void vApplicationMallocFailedHook(void);

void *
pvPortMalloc(size_t s)
{
	void *p;

	p = malloc(s);

#if (configUSE_MALLOC_FAILED_HOOK == 1)
	if (p == NULL) {
		vApplicationMallocFailedHook();
	}
#endif
	
	return p;
}

void
vPortFree(void *p)
{
	/* this is the init stack being freed */
	if (p == (void *)&_init_stack_end) {
		zfree(&MallocPool, (void *)&_init_stack_end, (&_init_stack_top - &_init_stack_end));
	} else {
		free(p);
	}
}

size_t
xPortGetFreeHeapSize(void)
{
	/* XXX this can easily be implemented by tweaking zallocstats */
	return 1;
}

void
vPortInitialiseBlocks(void)
{
	uint32_t	initial_heap_size, final_heap_size;

	/*
	 * If the init stack is immediately adjacent to the zalloc pool then we can absorb it if/when it is freed,
	 * so we should account for it in the sizing of the heap.
	 */
	initial_heap_size = &_eheap - &_sheap;
	if (&_init_stack_end == &_eheap) {
		final_heap_size = &_init_stack_top - &_sheap;
	} else {
		final_heap_size = initial_heap_size;
	}

	/* assign all of the heap space to the zalloc pool */
	/* XXX might be nice to have better panic hooks here */
	zinitPool(&MallocPool, "Malloc", znop, znot, (void *)&_sheap, final_heap_size);

	/* free back all of the heap except for the init stack; this will be given back when the init task terminates */
	zfree(&MallocPool, (void *)&_sheap, initial_heap_size);
}

void
xPortIncreaseHeapSize(size_t bytes)
{
	/* can't implement this as-is due to zalloc supporting non-contiguous extensions */
	/* might want to implement an extended version to deal with that at some point */
}


/*
 * Mutual exclusion support for the allocator.
 *
 * Note that xTaskResumeAll is pretty heavyweight.  We might be better off with a mutex.
 */
void
zlock(void)
{
	vTaskSuspendAll();
}

void
zunlock(void)
{
	xTaskResumeAll();
}
