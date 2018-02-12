/**
    \file mapperflash.h File System: Flash Mapper.

    \section intro_sec Introduction

	The Flash Mapper creates a wear-levelling buffer between a
	file system and a Flash memory physical layer.

    \version 1.0

    \date 2006-xx-xx
   
    \author Henrik Herranen

*/
#ifndef FS_MAP_FLASH
#define FS_MAP_FLASH

#define FS_MAP_FLASH_PAGE_SIZE 256
/** 4 x FS_MAP_FLASH_MAX_ERASE_PAGES + 1 words of memory is required */
#define FS_MAP_FLASH_MAX_ERASE_PAGES 256

#ifndef ASM
#include <vstypes.h>
#include <mapper.h>

struct FsPhysical;

/** Meta data */
struct FmfMeta {
  u_int16 ecc01;
  u_int16 ecc2AndType;
  u_int16 reservedAndBadBlock;
  u_int16 unused;
  u_int32 logicalPageNo;	/* For root node, this is last used */
  s_int32 newBranch;		/* Page # for root node, non- -1 for others */
};


#define FS_MAP_NON_FULL 4

/**
   A Flash Mapper specific structure that contains required
   extensions to the basic Mapper structure.
*/
struct FsMapperFlash {
  /** Public structure that is common to all mappers. */
  struct FsMapper m;
  /** Root node physical address. */
  u_int32 root;
  /** Logical blocks in erase unit. */
  s_int16 blocksPerErase;
  /** Last new physical address. */
  s_int32 lastUsed;
  /** Internal cache. */
  struct FmfCache *cache;
  /** Total of physical pages. */
  s_int32 physPages;
  /** Blocks that are not (almost) completely full with FMF_TYPE_DATA */
  s_int32 emptyBlock[FS_MAP_NON_FULL];
  /** How many pages in a block must be free for the block to be non-full */
  s_int16 nonFullLimit;
  /** How many blocks have been skipped while cleaning. */
  s_int32 skipped;
  /** How many blocks have been cleaned */
  s_int32 freed;
};
#endif /* !ASM */




#ifndef ASM
/** Create a mapper */
struct FsMapper *FsMapFlCreate(struct FsPhysical *physical, u_int16 format);
/** Delete a mapper */
s_int16 FsMapFlDelete(struct FsMapper *map);
/** Read blocks */
s_int16 FsMapFlRead(struct FsMapper *map, u_int32 firstLogicalBlock,
		    u_int16 logicalBlocks, u_int16 *data);
/** Write blocks */
s_int16 FsMapFlWrite(struct FsMapper *map, u_int32 firstLogicalBlock,
		     u_int16 logicalBlocks, u_int16 *data);
/** Flush all cached data. if \e hard is non-zero, all potential journals are
    also flushed. */
s_int16 FsMapFlFlush(struct FsMapper *map, u_int16 hard);
/** Free blocks. */
s_int16 FsMapFlFree(struct FsMapper *m, u_int32 logicalBlockNo,
		    u_int32 logicalBlocks);

/* Debug functions (fsMapFlDebug.h) */
void FsMapFlDump(struct FsMapper *map, s_int32 maxBlocks);
void FsMapFlCacheDump(struct FsMapper *map);
void FsMapFlPrint(s_int32 page); /* If page == 0, print from root node */

#ifndef __VSDSP__
#  define memcpyXY memcpy
#  define memcpyYX memcpy
#  define memcpyYY memcpy
#  define memsetY memset
#  define qsorty qsort
#endif

#endif /* !ASM */

#endif /* !FS_MAP_FLASH */
