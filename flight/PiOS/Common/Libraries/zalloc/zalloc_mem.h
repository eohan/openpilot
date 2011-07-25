
/*
 * H/MEM.H
 *
 * Basic memory pool / memory node structures.
 */

typedef struct MemNode {
    struct MemNode	*mr_Next;
    iaddr_t		mr_Bytes;
} MemNode;

typedef struct MemPool {
    const char 		*mp_Ident;
    void		*mp_Base;  
    void		*mp_End;
    MemNode		*mp_First; 
    void		(*mp_Panic)(const char *ctl, ...);
    int			(*mp_Reclaim)(struct MemPool *memPool, iaddr_t bytes);
    iaddr_t		mp_Size;
    iaddr_t		mp_Used;
} MemPool;

#define MEMNODE_SIZE_MASK       ((sizeof(MemNode) <= 8) ? 7 : 15)

#define INITPOOL(name,panic,reclaim)	{ name, NULL, NULL, NULL, panic, reclaim }

#define ZNOTE_FREE	0
#define ZNOTE_REUSE	1

