
/*
 * MALLOC.C - malloc equivalent, runs on top of zalloc and uses sbrk
 */

#include "zalloc_defs.h"
#include <string.h>

MemPool MallocPool;

#ifdef DMALLOCDEBUG
static int MallocMax;
static int MallocCount;
static int DidAtExit;
static int DoneAtExit;

void mallocstats(void);
#endif

void *
malloc(size_t bytes)
{
	Guard *res;

#ifdef USEENDGUARD
	bytes += MALLOCALIGN + 1;
#else
	bytes += MALLOCALIGN;
#endif

	res = znalloc(&MallocPool, bytes);
	if (res == NULL)
		return(NULL);

#ifdef DMALLOCDEBUG
	if (++MallocCount > MallocMax)
		MallocMax = MallocCount;
	if (DidAtExit == 0) {
		DidAtExit = 1;
		atexit(mallocstats);
	}
#endif
#ifdef USEGUARD
	res->ga_Magic = GAMAGIC;
#endif
	res->ga_Bytes = bytes;
#ifdef USEENDGUARD
	*((char *)res + bytes - 1) = -2;
#endif
	return((char *)res + MALLOCALIGN);
}

void
free(void *ptr)
{
	size_t bytes;

	if (ptr != NULL) {
		Guard *res = (void *)((char *)ptr - MALLOCALIGN);

#ifdef USEGUARD
		if (res->ga_Magic != GAMAGIC) {
#ifdef USEPANIC
			panic("free(): guard1 fail @ %08lx\n", ptr);
#else
			*(char *)0 = 1;
#endif
		}
		res->ga_Magic = -1;
#endif
#ifdef USEENDGUARD
		if (*((char *)res + res->ga_Bytes - 1) != -2) {
#ifdef USEPANIC
			panic("free(): guard2 fail @ %08lx + %d\n", ptr, res->ga_Bytes - MALLOCALIGN);
#else
			*(char *)0 = 1;
#endif
		}
		*((char *)res + res->ga_Bytes - 1) = -1;
#endif

		bytes = res->ga_Bytes;
		zfree(&MallocPool, res, bytes);
#ifdef DMALLOCDEBUG
		--MallocCount;
#endif
	}
}


void *
calloc(size_t n1, size_t n2)
{
	iaddr_t bytes = (iaddr_t)n1 * (iaddr_t)n2;
	void *res;

	if ((res = malloc(bytes)) != NULL) {
		memset(res, 0, bytes);
#ifdef DMALLOCDEBUG
		if (++MallocCount > MallocMax)
			MallocMax = MallocCount;
		if (DidAtExit == 0) {
			DidAtExit = 1;
			atexit(mallocstats);
		}
#endif
	}
	return(res);
}

/*
 * realloc() - I could be fancier here and free the old buffer before
 * 	       allocating the new one (saving potential fragmentation
 *	       and potential buffer copies).  But I don't bother.
 */

void *
realloc(void *ptr, size_t size)
{
	void *res;
	size_t old;

	if ((res = malloc(size)) != NULL) {
		if (ptr) {
			old = *(size_t *)((char *)ptr - MALLOCALIGN) - MALLOCALIGN;
			if (old < size)
				bcopy(ptr, res, old);
			else
				bcopy(ptr, res, size);
			free(ptr);
		} else {
#ifdef DMALLOCDEBUG
			if (++MallocCount > MallocMax)
				MallocMax = MallocCount;
			if (DidAtExit == 0) {
				DidAtExit = 1;
				atexit(mallocstats);
			}
#endif
		}
	}
	return(res);
}

void *
reallocf(void *ptr, size_t size)
{
	void *res;

	if ((res = realloc(ptr, size)) == NULL)
		free(ptr);
	return(res);
}

#ifdef DMALLOCDEBUG

void
mallocstats(void)
{
	if (DoneAtExit == 0) {
		++DoneAtExit;
		fprintf(stderr, "Active Allocations: %d/%d\n", MallocCount, MallocMax);
#ifdef ZALLOCDEBUG
		zallocstats(&MallocPool);
		fflush(stderr);
#endif
	}
}

#endif

