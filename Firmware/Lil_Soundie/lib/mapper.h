/**
   \file mapper.h File system layer 5: Mapper layer.
   The mapper layer takes care of mapping between logical and physical
   blocks. Also, if there is a difference between read/write block size
   and erase page size, it is hidden by the mapper. Also caching is
   performed by the mapper.

   With simple file systems, like MMC, where the MMC card itself takes
   care of mapping and erasing, caching is the only operation needed
   by the mapper.
*/

#ifndef MAPPER_H
#define MAPPER_H

#include <vstypes.h>


/**
   Current version number. 8 MSBs contain version number, 8 LSBs size of
   the structure in words.

   Version number history:
   <OL>
	<LI>0x010C First version
   </OL>
*/
#define FS_MAPPER_VERSION 0x010C


#ifndef ASM
struct FsPhysical;

/**
   File system Mapper layer structure. Each Mapper should begin
   its own internal structure with this common structure.
 */
struct FsMapper {
  /** Version number. 8 MSBs contain version number, 8 LSBs size of
      the structure in words. */
  u_int16 version;
  /** How many 16-bit words in a block */
  u_int16 blockSize;
  /** How many usable blocks in the whole system */
  u_int32 blocks;
  /** How many blocks can be cached by the mapper */
  u_int16 cacheBlocks;
  /** Create a mapper. */
  struct FsMapper *(*Create)(struct FsPhysical *physical, u_int16 cacheSize);
  /** Delete a mapper */
  s_int16 (*Delete)(struct FsMapper *map);
  /** Read blocks */
  s_int16 (*Read)(struct FsMapper *map, u_int32 firstBlock, u_int16 blocks,
		  u_int16 *data);
  /** Write blocks */
  s_int16 (*Write)(struct FsMapper *map, u_int32 firstBlock, u_int16 blocks,
		   u_int16 *data);
  /** Free blocks (implementation must be able to go fastly through
      large free areas. */
  s_int16 (*Free)(struct FsMapper *map, u_int32 firstBlock, u_int32 blocks);
  /** Flush all data. if \e hard is non-zero, all potential journals are
	also flushed. */
  s_int16 (*Flush)(struct FsMapper *map, u_int16 hard);
  /** Pointer to this Mapper's Physical layer. */
  struct FsPhysical *physical;
};
#endif /* !ASM */

#define MAP_VERSION_OFFSET	 0
#define MAP_BLOCK_SIZE_OFFSET	 1
#define MAP_BLOCKS_OFFSET	 2
#define MAP_CACHE_BLOCKS_OFFSET	 4
#define MAP_CREATE_OFFSET	 5
#define MAP_DELETE_OFFSET	 6
#define MAP_READ_OFFSET		 7
#define MAP_WRITE_OFFSET	 8
#define MAP_FREE_OFFSET		 9
#define MAP_FLUSH_OFFSET	10
#define MAP_PHYSICAL_OFFSET	11

#endif /* !MAPPER */
