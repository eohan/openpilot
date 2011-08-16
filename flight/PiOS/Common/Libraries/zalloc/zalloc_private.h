/**
 * @file	zalloc_private.h
 * @brief	Private definitions and options for the zalloc allocator
 */

#ifndef __ZALLOC_PRIVATE_H
#define __ZALLOC_PRIVATE_H

/*
 * Enable trailing allocation guard to help detect buffer overflows,
 * unless explicitly disabled.  This adds 1 byte to allocations.
 */
#ifndef DMALLOC_NO_GUARD
# define USEENDGUARD	1
#endif

#include "zalloc.h"

/**
 * A node in the zalloc freelist.
 */
typedef struct MemNode {
    struct MemNode	*mr_Next;
    iaddr_t			mr_Bytes;
} MemNode;

#define MEMNODE_SIZE_MASK       ((sizeof(MemNode) <= 8) ? 7 : 15)

/**
 * A pool from which memory may be allocated and to which it may be freed.
 */
typedef struct MemPool {
    const char		*mp_Ident;
    void			*mp_Base;
    void			*mp_End;
    MemNode			*mp_First;
    void			(* mp_Panic)(const char *ctl, ...);
    int				(* mp_Reclaim)(struct MemPool *memPool, iaddr_t bytes);
    iaddr_t			mp_Size;
    iaddr_t			mp_Used;
} MemPool;

#endif // __ZALLOC_PRIVATE_H
