/**
   \file vsNand.h NAND FLASH routines.
*/
#ifndef VS_NAND_H
#define VS_NAND_H

#include <vstypes.h>
#include <stdlib.h>
#include <physical.h>

/** Nand Flash Opcode: Read Signature*/
#define NAND_OP_READ_SIGNATURE 0x90

/** Nand Flash Opcode: Read Signature*/
#define NAND_OP_READ_STATUS 0x70

/** Nand Flash Opcode: Read A - data area */
#define NAND_OP_READ_A 0x00

/** Nand Flash Opcode: Read C - spare area */
#define NAND_OP_READ_C 0x50

/** Nand Flash Opcode: Commit read address */
#define NAND_OP_COMMIT_DATA_ADDRESS 0x30

/** Nand Flash Opcode: Prepare to Program*/
#define NAND_OP_PREPARE_TO_PROGRAM 0x80

/** Nand Flash Opcode: Execute Programming*/
#define NAND_OP_PERFORM_PROGRAM 0x10


struct FsNandPhys {
  /** Physical basic structure */
  struct FsPhysical p;
  /* Custom fields follow */
  u_int16 nandType; /**< NAND type, small/large-page, number of address bytes*/
  u_int16 waitns;   /**< read/write time in ns */
};

/** Erase the block that starts from page "block". */
s_int16 FsPhNandErase(struct FsPhysical *p, s_int32 block);

/** Creates a physical layer. */
struct FsPhysical *FsPhNandCreate(u_int16 param);

/** Free resources allocated by FsPhNandCreate and release HW */
s_int16 FsPhNandDelete(struct FsPhysical *p);

/** Free hardware bus for possible other devices */
s_int16 FsPhNandFreeBus(struct FsPhysical *p);

/** Reinitialize bus */
s_int16 FsPhNandReinitialize(struct FsPhysical *p);

/** Read pages. If meta is non-NULL, will use error correction.
 To read both sector and spare areas without error correction,
 you need two reads: first read only sector datas (meta = NULL),
 then read only spares (data = NULL). */
s_int16 FsPhNandRead(struct FsPhysical *p, s_int32 firstPage,
		     u_int16 pages, u_int16 *data, u_int16 *meta);

/** Write pages. If meta is non-NULL, will generate error correction. */
s_int16 FsPhNandWrite(struct FsPhysical *p, s_int32 firstPage,
		      u_int16 pages, u_int16 *data, u_int16 *meta);

/** Support: Count the number of 1-bits */
s_int16 NandCountBits(register __reg_a u_int32 val);
/** Support: takes every other bit from the value. */
s_int16 NandMingle(register __reg_a u_int32 val);
/** Support: reorder large-page bad block indicators to be compatible
    with small-page bad block indicators. */
void NandSwapBad(register __i0 u_int16 *spare);
/** Support: Waits until the NAND FLASH interface is idle. */
void NandWaitIdle(void);

#endif
