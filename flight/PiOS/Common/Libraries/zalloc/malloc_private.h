/**
 * @file	malloc_private.h
 * @brief	Private definitions for the malloc/zalloc allocator.
 */

#ifndef __MALLOC_PRIVATE_H
#define __MALLOC_PRIVATE_H

#include "zalloc_private.h"

/**
 * Memory pool used by malloc.
 */
extern struct MemPool	MallocPool;

/**
 * Allocation front guard and size tracking structure.
 */
typedef struct Guard {
    size_t	ga_Bytes;		/**< size of the allocation */
    size_t	ga_Magic;		/**< head guard, also used to assert pointer validity in free */
} Guard;

/** guard/sanity magic */
#define GAMAGIC		0x55FF44FD

/** end guard magic */
#define EGAMAGIC	-2

/** non-magic value */
#define NOMAGIC		-1

#endif // __MALLOC_PRIVATE_H
