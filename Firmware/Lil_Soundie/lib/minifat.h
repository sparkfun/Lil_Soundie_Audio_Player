/**
   \file minifat.h MiniFAT contains functions to handle one simultaneous open file.
*/
#ifndef MINIFAT_H
#define MINIFAT_H

#define MINIFAT_H

#include <vstypes.h>
#include "fat.h"

/** Reads byte values from minifatBuffer.
    \param n byte offset of the value inside minifatBuffer
    \return byte value
 */
auto u_int16 FatGetByte(register __c0 u_int16 n);
/** Reads 16-bit word values from minifatBuffer.
    \param n byte offset of the little-endian value inside minifatBuffer
    \return word value
 */
auto u_int16 FatGetWord(register __c0 u_int16 n);
/** Reads 32-bit long values from minifatBuffer.
    \param n byte offset of the little-endian value inside minifatBuffer
    \return long value
 */
auto u_int32 FatGetLong(register __c0 u_int16 n);
/** Initializes the file system and checks if FAT present.
    \return 0 for success.
 */
auto u_int16 FatInitFileSystem(void);
/** Creates a list of fragments in a file, starting from a specified FAT cluster.
    \param frag fragment array entry to start filling information from.
    \param fatCluster  fat cluster to start from.
    \return the next free fragment array entry.
 */
auto __y struct FRAGMENT *FatFragmentList(
    register __i2 __y struct FRAGMENT *frag,
    register __reg_b u_int32 fatCluster);
/** Internal function that scans files until the right file is found.
      Uses minifatInfo.gFileNum[0] and minifatInfo.gFileNum[1] for
      file number counts.
    \param curFragment The clusters in the directory to scan.
    \param nextFragment The first free fragment table entry, used for recursion.
    \return < 0 for file found, otherwise the total number of files (can be 0).
 */
auto s_int16 FatHandleDir(register __y struct FRAGMENT *curFragment,
			  __y struct FRAGMENT *nextFragment);
/** Opens the specified file for reading.
          Only counts files that match a suffix set in
	  the array set to minifatInfo.supportedSuffixes,
	  or all files if minifatInfo.supportedSuffixes is NULL.
    \param fileNum  The number of file to open, starting from 0.
    \return < 0 for file found, otherwise the total number of files (can be 0).
 */
auto s_int16 FatOpenFile(register __c0 u_int16 fileNum);
/** Reads bytes from the current file position.
    \param buf Packed buffer to read bytes to.
    \param byteOff Packed byte offset of the first byte. Even = high part of word, odd = low part of word.
    \param byteSize The number of bytes to read.
    \return the number of bytes actually read.
    \example tmpBuf[0] = 0; FatReadFile(tmpBuf, 1, 1); reads one byte to the low part of tmpBuf[0].
 */
auto s_int16 FatReadFile(register __i3 u_int16 *buf,
			 register __c1 s_int16 byteOff,
			 register __c0 s_int16 byteSize);
/** Returns the current read byte position of the file.
    \return Current read position.
 */
u_int32 FatTell(void);
/** Changes the current read byte position of the file.
    \param pos Absolute position for the new read byte position.
    \return The old read position.
 */
u_int32 FatSeek(register __reg_a u_int32 pos); /*< returns old position */
/** Internal function to locate the right sector for the current read position.
    \param pos Byte position to find.
    \return the FAT sector that corresponds to the pos.
 */
auto u_int32 FatFindSector(register __reg_d u_int32 pos);
/** Internal function to compare the 24-bit parameter to allowed suffixes.
    \param suffix The suffix to compare.
    \return non-zero if suffix matches, or minifatInfo.supportedSuffixes is NULL.
 */
auto s_int16 FatCheckFileType(register __reg_a u_int32 suffix);


/** Copies big-endian packed byte strings with arbitrary alignments (in X memory).
    \param dst destination word pointer
    \param dstidx destination byte offset
    \param src source word pointer
    \param srcidx source byte offset
    \param byteSize the number of bytes to copy
 */
void MemCopyPackedBigEndian(register __i0 u_int16 *dst,
			    register __a0 u_int16 dstidx,
			    register __i1 u_int16 *src,
			    register __a1 u_int16 srcidx,
			    register __b0 u_int16 byteSize);
/** Not in VS1000B ROM!
 */
void MemCopyPackedLittleEndian(register __i0 u_int16 *dst,
			       register __a0 u_int16 dstidx,
			       register __i1 u_int16 *src,
			       register __a1 u_int16 srcidx,
			       register __b0 u_int16 byteSize);
/** Not in VS1000B ROM!
 */
void MemWritePacked(register __a0 void *dst, register __a1 u_int16 dstidx,
		    register __b0 u_int16 dat);
/** Not in VS1000B ROM!
 */
u_int16 MemReadPacked(register __a0 const void *src,
		      register __a1 u_int16 srcidx);
/** Writes bytes to big-endian packed byte array in Y memory.
    \param dst destination word pointer
    \param dstidx destination byte offset
    \param dat byte to write, the value should be 0..255
 */
void MemWritePackedY(register __a0 __y void *dst, register __a1 u_int16 dstidx,
		    register __b0 u_int16 dat);
/** Not in VS1000B ROM!
 */
u_int16 MemReadPackedY(register __a0 __y const void *src,
		      register __a1 u_int16 srcidx);

typedef s_int16 (*freeSectorCallback)(void *private, u_int32 sector, u_int32 numSecs);
/** Finds free areas from the FAT disk and executes a callback function.
    \param callBackFunction Is called for each free area. If callback returns
      non-zero, the iteration is ended prematurely.
    \param private a private parameter to be passed to the callBackFunction.
 */
s_int16 FatIterateOverFreeSectors(freeSectorCallback callBackFunction,
				  void *private);

/** Outside service that must be provided for minifat.
        VS1000 provides this function through IRAM hook ReadDiskSector.
	The default value for it is MapperReadDiskSector(), which uses
	map->Read() to implement the sector read.
    \param buffer the buffer for the sector data.
    \param sector the sector to read.
 */
auto u_int16 ReadDiskSector(register __i0 u_int16 *buffer,
			    register __reg_a u_int32 sector);

/**
   Minifat interface:
   - ReadDiskSector() must be provided

 */


#endif/*MINIFAT_H*/
