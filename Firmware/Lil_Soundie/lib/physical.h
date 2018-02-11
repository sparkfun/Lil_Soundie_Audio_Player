/**
   \file physical.h File system layer 6: Physical layer.
   This is the lowest layer of a file system. This layer takes care
   of the actual, physical connection to a device.
 */

#ifndef FS_PHYSICAL_H
#define FS_PHYSICAL_H

#include <vstypes.h>

/**
   Current version number. 8 MSBs contain version number, 8 LSBs size of
   the structure in words.

   Version number history:
   <OL>
	<LI>0x010B
   </OL>
*/
#define FS_PHYSICAL_VERSION 0x010B


/**
   File system Physical layer basic structure. Each Physical layer
   internal structure should begin with this.
*/
struct FsPhysical {
  /** Version number. 8 MSBs contain version number, 8 LSBs size of
      the structure in words. */
  u_int16 version;
  /** In 16-bit words */
  u_int16 pageSize;
  /** In pages */
  u_int16 eraseBlockSize;
  /** The size of the memory unit in erasable blocks */
  u_int16 eraseBlocks;
  /** Creates a physical layer. param is a device-specific parameter,
      usually 0. */
  struct FsPhysical *(*Create)(u_int16 param);
  /** Delete a physical layer */
  s_int16 (*Delete)(struct FsPhysical *p);
  /** Read sectors. meta is physical-specific data and not necessarily
      used. If either data or meta is NULL, that information is not
      returned. Setting both pointers to NULL is an error condition. */
  s_int16 (*Read)(struct FsPhysical *p, s_int32 firstPage, u_int16 pages,
		  u_int16 *data, u_int16 *meta);
  /** Write sectors. meta is physical-specific data and not necessarily
      used. If either data or meta is NULL, that information is not
      written. Setting both pointers to NULL is an error condition. */
  s_int16 (*Write)(struct FsPhysical *p, s_int32 firstPage, u_int16 pages,
		   u_int16 *data, u_int16 *meta);
  /** Erase block. \e firsPage is the page number of the first page in the
      block. */
  s_int16 (*Erase)(struct FsPhysical *p, s_int32 page);
  /** Frees the bus for other devices */
  s_int16 (*FreeBus)(struct FsPhysical *p);
  /** Re-initializes bus after a fatal error (eg memory card removal) */
  s_int16 (*Reinitialize)(struct FsPhysical *p);
};


#endif /* !PHYSICAL */
