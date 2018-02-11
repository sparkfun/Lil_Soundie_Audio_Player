/**
    \file mappertiny.h File System: Tiny Flash Mapper.

    \section intro_sec Introduction

	The Tiny Flash Mapper is a read-only mapper that reads a wear-levelled
	buffers created by the Flash Mapper (mapperflash.h). It acts as a
	converter tool between a file system logical and Flash physical
	layers.

	This tiny mapper is not a very efficient implementation. For every
	logical block read there are max five physical read operations.

    \version 1.0

    \date 2006-08-25
*/
#ifndef FS_MAP_TINY
#define FS_MAP_TINY

#ifndef ASM
#include <vstypes.h>
#include <mapper.h>
#include <mapperflash.h>

struct FsPhysical;

/**
   A Tiny Flash Mapper specific structure that contains required
   extensions to the basic Tiny Flash Mapper structure.
   The Tiny Flash Mapper is a read-only system that has been
   optimized to access consecutive blocks using only a few words
   of memory. For such a file performance is very good, but if
   several file are open, performance may be slow.
*/
struct FsMapperTiny {
  /** Public structure that is common to all mappers. */
  struct FsMapper m;
  /** Root node physical address. */
  s_int32 root;
  /** Logical blocks in erase unit. */
  s_int16 blocksPerErase;
  /** First logical in access list. */
  s_int32 firstBlock;
  /** Last logical in access list. */
  s_int32 lastBlock;
  /** Logical block to physical page conversion number. */
  s_int32 logToPhys;
  /** Meta info (buffer space) */
  struct FmfMeta meta;
};
#endif /* !ASM */




#ifndef ASM
/** Create a tiny mapper */
struct FsMapper *FsMapTnCreate(struct FsPhysical *physical,
			       u_int16 cacheSize);
/** Delete a tiny mapper */
s_int16 FsMapTnDelete(struct FsMapper *map);
/** Read blocks */
s_int16 FsMapTnRead(struct FsMapper *map, u_int32 firstLogicalBlock,
		    u_int16 logicalBlocks, u_int16 *data);
/** Write blocks */
s_int16 FsMapTnWrite(struct FsMapper *map, u_int32 firstLogicalBlock,
		     u_int16 logicalBlocks, u_int16 *data);
/** Flush all cached data. if \e hard is non-zero, all potential journals are
    also flushed. */
s_int16 FsMapTnFlush(struct FsMapper *map, u_int16 hard);
/** Free blocks. */
s_int16 FsMapTnFree(struct FsMapper *m, u_int32 logicalBlockNo,
		    u_int32 logicalBlocks);

s_int16 FsMapFlNullFail(); /*< intentionally incomplete prototype */
s_int16 FsMapFlNullOk();   /*< intentionally incomplete prototype */
#endif /* !ASM */

#endif /* !FS_MAP_FLASH */
