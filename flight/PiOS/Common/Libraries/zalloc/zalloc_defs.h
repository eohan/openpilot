
/*
 * DEFS.H
 */

#define MALLOCLIB	1
#define USEGUARD	1
//#define DMALLOCDEBUG	1
//#define USEENDGUARD	1
//#define USEPANIC	1
//#define ZALLOCDEBUG	1

#include <stdint.h>
#include <stdlib.h>

typedef uintptr_t iaddr_t;	/* unsigned int same size as pointer	*/
typedef intptr_t saddr_t;	/* signed int same size as pointer	*/

#include "zalloc_mem.h"

#define Prototype extern
#define Library extern

#ifndef NULL
#define NULL	((void *)0)
#endif

/*
 * block extension for sbrk()
 */

#define BLKEXTEND	(64 * 1024)
#define BLKEXTENDMASK	(BLKEXTEND - 1)

/*
 * required malloc alignment.  Use sizeof(long double) for architecture
 * independance.
 *
 * Note: if we implement a more sophisticated realloc, we should ensure that
 * MALLOCALIGN is at least as large as MemNode.
 */

typedef struct Guard {
    size_t	ga_Bytes;
    size_t	ga_Magic;	/* must be at least 32 bits */
} Guard;

#define MATYPE		long double
#define MALLOCALIGN	((sizeof(MATYPE) > sizeof(Guard)) ? sizeof(MATYPE) : sizeof(Guard))
#define GAMAGIC		0x55FF44FD

#include "zalloc_protos.h"

