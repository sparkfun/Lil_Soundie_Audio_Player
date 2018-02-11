/**
   \file fat.h FAT definitions.
*/
#ifndef _FATINFO_H_
#define _FATINFO_H_

#define FATINFO_IN_Y       /* put the fatInfo to Y(defined) or X (undef) */
#define MAX_FRAGMENTS 35
/* long filename support can be disabled by undeffing FAT_LFN_SIZE--65 words */
#define FAT_LFN_SIZE (2*13 *2) /*52-byte filename*/

#ifdef ASM

#define ISFAT32  0
#define FATSTART 1
#define ROOTSTART 3
#define DATASTART 5
#define CURRENTSECTOR 7
#define FILESIZE 9
#define FATSECTORSPERCLUSTER 11
#define BPBROOTENTCNT 12
#define FILSYSTYPE 13
#define TOTSIZE 14
#define FILENAME 16
#define GFILENUM 22
#define FILEPOS 24
#define PARENTDIR 26
#define SUPPORTEDSUFFIXES 28

#define _IS_FAT_32                _fatInfo+ISFAT32
#define _fatStart                 _fatInfo+FATSTART
#define _rootStart                _fatInfo+ROOTSTART
#define _dataStart                _fatInfo+DATASTART
#define _currentSector            _fatInfo+CURRENTSECTOR
#define _fileSize                 _fatInfo+FILESIZE
#define _fatSectorsPerCluster     _fatInfo+FATSECTORSPERCLUSTER
#define _BPB_RootEntCnt           _fatInfo+BPBROOTENTCNT

#else

#define FAT_MKID(a,b,c) ((a)|((b)<<8)|((u_int32)(c)<<16))
//const u_int32 fatDefSupported[] = {FAT_MKID('O','G','G'), FAT_MKID('R','U','N'), 0};

struct FATINFO { /* do not change sizes or order of fields! */
    u_int16 IS_FAT_32;       /**< FAT32 or FAT16/FAT12 */         // _fatInfo+0
    u_int32 fatStart;        /**< Fat start sector number */      // _fatInfo+1
    u_int32 rootStart;       /**< Root director start */          // _fatInfo+3
    u_int32 dataStart;       /**< File Allocation Table start */  // _fatInfo+5
    u_int32 currentSector;   /**< Sector in minifatBuffer */      // _fatInfo+7
    u_int32 fileSize;        /**< Current file size */            // _fatInfo+9
    u_int16 fatSectorsPerCluster;     // _fatInfo+11
    u_int16 BPB_RootEntCnt;           // _fatInfo+12
    u_int16 FilSysType; /**< "\p21" for FAT12 */ // _fatInfo+13 to detect FAT12
    s_int32 totSize;    /**< Total sectors */    // _fatInfo+14
    u_int16 fileName[6];/**< Current file name */ // _fatInfo+16
    u_int16 gFileNum[2];/**< File number counters */ // _fatInfo+22
    s_int32 filePos;    /**< Current file byte read position*/ // _fatInfo+24
    s_int32 parentDir;  /**< Parent directory start sector */  // _fatInfo+26
    const u_int32 *supportedSuffixes; /**< NULL or list of supported suffixes. */ //_fatInfo+28
#ifdef FAT_LFN_SIZE
    u_int16 longFileName[FAT_LFN_SIZE/2]; /**< Long filename, if exists */ // _fatInfo+29
#endif
};


#define LAST_FRAGMENT 0x80000000UL
struct FRAGMENT {
    u_int32 start; /**< high bit set if last fragment*/
    u_int16 size;  /**< fragment size in sectors */
};
extern __y struct FRAGMENT minifatFragments[MAX_FRAGMENTS];
#ifdef FATINFO_IN_Y
extern __y struct FATINFO minifatInfo;
#else
extern struct FATINFO minifatInfo;
#endif
extern u_int16 minifatBuffer[256]; /**< holds one sector of data */


#endif/*ASM*/

#endif/*_FATINFO_H_*/
