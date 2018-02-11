/**
   \file dev1000.h Development library header file.
*/
#ifndef __DEV1000_H__
#define __DEV1000_H__

#include <vstypes.h>


#define GPIO1_XCS  (1<<0)
#define GPIO1_SCLK (1<<1)
#define GPIO1_SI   (1<<2)
#define GPIO1_SO   (1<<3)
#define GPIO1_TX   (1<<4)
#define GPIO1_RX   (1<<5)
#define GPIO0_CS2  (1<<14)

#define PATCH_TEST_UNIT_READY /*PatchMSCPacketFromPC() calls ScsiTestUnitReady()*/
#define MMC_MISO_BIT 8  /*NFRDY with 10kOhm pull-up*/
#define MMC_MOSI_BIT 12 /*NFCLE*/
#define MMC_CLK      (1<<9)  /*NFRD*/
#define MMC_MOSI     (1<<MMC_MOSI_BIT)
#define MMC_MISO     (1<<MMC_MISO_BIT)
#define MMC_XCS      (1<<11) /*NFWR*/

#define MMC_GO_IDLE_STATE   0
#define MMC_SEND_OP_COND   1
#define MMC_SEND_IF_COND   8
#define MMC_SEND_CSD   9
#define MMC_SEND_CID   10
#define MMC_STOP_TRANSMISSION   12
#define MMC_SEND_STATUS   13
#define MMC_SET_BLOCKLEN   16
#define MMC_READ_SINGLE_BLOCK   17
#define MMC_READ_MULTIPLE_BLOCK   18
#define MMC_WRITE_BLOCK   24
#define MMC_WRITE_MULTIPLE_BLOCK   25
#define MMC_PROGRAM_CSD   27
#define MMC_SET_WRITE_PROT   28
#define MMC_CLR_WRITE_PROT   29
#define MMC_SEND_WRITE_PROT   30
#define MMC_TAG_SECTOR_START   32
#define MMC_TAG_SECTOR_END   33
#define MMC_UNTAG_SECTOR   34
#define MMC_TAG_ERASE_GROUP_START   35
#define MMC_TAG_ERARE_GROUP_END   36
#define MMC_UNTAG_ERASE_GROUP   37
#define MMC_ERASE   38
#define MMC_READ_OCR     58
#define MMC_CRC_ON_OFF   59
#define MMC_R1_BUSY   0x80
#define MMC_R1_PARAMETER   0x40
#define MMC_R1_ADDRESS   0x20
#define MMC_R1_ERASE_SEQ   0x10
#define MMC_R1_COM_CRC   0x08
#define MMC_R1_ILLEGAL_COM   0x04
#define MMC_R1_ERASE_RESET   0x02
#define MMC_R1_IDLE_STATE   0x01
#define MMC_STARTBLOCK_READ   0xFE
#define MMC_STARTBLOCK_WRITE   0xFE
#define MMC_STARTBLOCK_MWRITE   0xFC
#define MMC_STOPTRAN_WRITE   0xFD
#define MMC_DE_MASK   0x1F
#define MMC_DE_ERROR   0x01
#define MMC_DE_CC_ERROR   0x02
#define MMC_DE_ECC_FAIL   0x04
#define MMC_DE_OUT_OF_RANGE   0x04
#define MMC_DE_CARD_LOCKED   0x04
#define MMC_DR_MASK   0x1F
#define MMC_DR_ACCEPT   0x05
#define MMC_DR_REJECT_CRC   0x0B
#define MMC_DR_REJECT_WRITE_ERROR   0x0D

#define LCD_HEIGHT   8
#define LCD_WIDTH    132

/** Nand Flash Opcode: Read for Copy-Back*/
#define NAND_OP_READ_FOR_COPY_BACK 0x35
/** Nand Flash Opcode: Copy-back Program*/
#define NAND_OP_COPY_BACK_PROGRAM 0x85

#define LP4K_ECC_MASK 0x0001

#ifdef ASM

#else /*elseASM*/
/** Send and receive bits from MMC/SD.
    \param dataTopAligned The first bit to send must be in the most-significant bit.
    \param bits The number of bits to send/receive.
    \return If less than 16 bits are sent, the high bits of the result
            will be the low bits of the dataTopAligned parameter.
 */
auto u_int16 SpiSendReceiveMmc(register __a0 u_int16 dataTopAligned,
			       register __a1 s_int16 bits);
/** Send and receive words from MMC/SD. Not much faster than calling
    SpiSendReceiveMmc() from a loop.
    \param data  Pointer to data in X memory.
    \param words The number of words to send/receive.
 */
auto void SpiSendBlockMmc(register __i0 u_int16 *data, register __c0 s_int16 words);
/** Send MMC clocks with XCS high.
 */
auto void SpiSendClocks(void);
/** Send MMC/SD command. The CRC that is sent will always be 0x95.
    \param cmd The MMC command, must include the start bit (0x40).
    \param arg The command argument.
    \return the result code or 0xff (8th bit set) for timeout.
 */
auto u_int16 MmcCommand(register __b0 s_int16 cmd, register __reg_d u_int32 arg);

/** LBAB patch -- removes 4G restriction from USB (SCSI).
    Is not needed for VS1000d, but is compatible with it. */
void PatchMSCPacketFromPC(void *); /* Set MSCPacketFromPC hook here. */
/** Hook called by PatchMSCPacketFromPC(). */
void ScsiTestUnitReady(void);

/** Replacement for FatOpenFile that is normally in OpenFile hook.
    This version disables subdirectories for FAT12 partitions so
    OpenFile will not go into infinite recursion trying to handle it.
    Is not needed for VS1000d, but is compatible with it.

    SetHookFunction((u_int16)OpenFile, Fat12OpenFile);
    \param fileNum the same as for OpenFile
    \return the same as OpenFile
*/
auto u_int16 Fat12OpenFile(register __c0 u_int16 fileNum);
/** Replacement for FatOpenFile that is normally in OpenFile hook.
    This version speeds up directory processing by detecting end of
    directory instead of reading to the end of the cluster.
    This is significant when using a large card with a large
    sectors-per-cluster value and there are a lot of directories.

    The code also disables subdirectories for FAT12 partitions so
    OpenFile will not go into infinite recursion trying to handle it.
    Is compatible with VS1000d.

    SetHookFunction((u_int16)OpenFile, FatFastOpenFile);
    \param fileNum the same as for OpenFile
    \return the same as OpenFile
*/
auto u_int16 FatFastOpenFile(register __c0 u_int16 fileNum);
/** Replacement for FatOpenFile that is normally in OpenFile hook.
    This version speeds up directory processing by detecting end of
    directory instead of reading to the end of the cluster.
    This is significant when using a large card with a large
    sectors-per-cluster value and there are a lot of directories.

    The code also fixes some long-filename problems,
    but does not fix fat12 issues.
    Is compatible with both VS1000B/C and VS1000D.

    SetHookFunction((u_int16)OpenFile, FatFastOpenFileD);
    \param fileNum the same as for OpenFile
    \return the same as OpenFile
*/
auto u_int16 FatFastOpenFileD(register __c0 u_int16 fileNum);

/** Open a file based on the name. The suffix is not changed, the caller
    must select the right suffix and restore the original after the call.
    See also the faster OpenFileNamed().
    \param packedName The 8-character name must be in upper case and in packed format.
    \return The file index or 0xffffU if file is not found.
*/
auto u_int16 OpenFileBaseName(register __i2 const u_int16 *packedName);

/** Goes through files with the active suffix, calls IterateFilesCallBack()
    for all files. */
auto void IterateFiles(void);
/** Callback function for IterateFiles(). The parameter points to the start
    of the packed file name (in minifatBuffer). Other FAT fields follow the
    name, so they can be used as well. You can abort scan by setting
    minifatInfo.gFileNum[0] to zero.
*/
void IterateFilesCallback(register __b0 u_int16 *name);
/** A faster named file open routine. Note that only the short filename
    is checked. The FAT supportedSuffixes is set to supportedFiles afterwards.
    \param fname The 8-character name must be in upper case and in packed format padded with spaces. For example "\pMYFILE  ".
    \param suffix The desired suffix, for example FAT_MKID('O','G','G')
    \return The file index within files with the same suffix or
            0xffffU if file is not found.
*/
auto s_int16 OpenFileNamed(const u_int16 *fname, u_int32 suffix);


/** Sets the start and end positions for the PlayRange() function.
    \param start Play start time in 1/65536th of a second resolution.
    \param end Play end time in 1/65536th of a second resolution. Set to 0x7fffffffUL to disable end time.
*/
void PlayRangeSet(u_int32 start, u_int32 end);
/** Plays from a previously set start position to end position.
    Handles player and cs structure initialization.
    You must call PlayRangeSet() before calling PlayRange().
*/
void PlayRange(void);
/** For debugging with vs3emu: print 4-digit hex number. */
void puthex(u_int16 d);

/** Replacement routine for KeyScan() that does not check USB insertion.
    Normal use:
    \code
    if (uiTrigger) {
      uiTrigger = 0;
      KeyScanNoUSB(0x1f);
    }
    \endcode
    \param gpioMask - 0x1f for 5 keys + POWER button
 */
void KeyScanNoUSB(register __a1 u_int16 gpioMask);
/** Replacement routine for KeyScan() to read GPIO0[7:0] and power button
    instead of just GPIO[4:0] and power button. GPIO0_7 polarity is
    reversed, i.e. a press of the key will connect a pull-down. */
void KeyScan9(void);
/** Replacement routine for KeyScan() to read GPIO[5:0] and power button
    instead of just GPIO[4:0] and power button. See also Suspend7(). */
void KeyScan7(void);
#define KEY_6 (1<<5)
#define KEY_7 (1<<6)
#define KEY_8 (1<<7)
/** Replacement routine for USBSuspend() to wake up from GPIO[5:0] and
    power button (and USB pins) instead of just GPIO[4:0] and power button
    (and USB pins). Also puts the analog to powerdown for the duration
    of USB suspend/low-power pause. See also KeyScan7().
 */
void Suspend7(u_int16 timeOut);
/** Replacement routine for USBSuspend() to wake up from GPIO[5:0] and
    power button (and USB pins) instead of just GPIO[4:0] and power button
    (and USB pins). Does not put the analog to powerdown.
    See also KeyScan7().
 */
void Suspend7WithPower(u_int16 timeOut);
/** Replacement routine for USBSuspend() to wake up from GPIO[6:0] and
    power button (and USB pins) instead of just GPIO[4:0] and power button
    (and USB pins). Does not put the analog to powerdown.
    See also KeyScan7(), KeyScan9().
 */
void Suspend8WithPower(u_int16 timeOut);
/** Replacement routine for KeyScan() to read matrix keyboard connected
    to GPIO[7:4] and GPIO[3:0] and power button instead of just
    GPIO[4:0] and power button. See also SuspendMatrix(). */
void KeyScanMatrix(register __i2 const u_int16 *matrix);
/** Replacement routine for USBSuspend() to wake up from a matrix keyboard
    connected to GPIO[3:0] and GPIO[7:4] and power button (and USB pins)
    instead of just GPIO[4:0] and power button
    (and USB pins). Also puts the analog to powerdown for the duration
    of USB suspend/low-power pause. See also KeyScanMatrix().
 */
void SuspendMatrix(void);

auto u_int16 MapperlessReadDiskSector(register __i0 u_int16 *buffer,
				      register __reg_a u_int32 sector);


/*
  Some low-level NAND-interface routines that are useful with parallel
  displays. You must handle CS yourself.
 */
void NandPutCommand(register __a0 u_int16 command); /**< Writes 8 bits with CLE=1. You must handle NFCS yourself. */
void NandPutAddressOctet(register __a0 u_int16 address); /**< writes 8 bits with ALE=1. You must handle NFCS yourself. */
void NandGetOctets(register __c0 s_int16 length, register __i2 u_int16 *buf);/**< Writes packed data to NFIO. You must handle NFCS yourself. */
void NandPutOctets(register __c0 s_int16 length, register __i2 u_int16 *buf);/**< Reads packed data from NFIO. You must handle NFCS yourself. */
void NandSetWaits(register __a0 u_int16 waitns); /**< Sets NAND-interface waitstates in nanoseconds. Uses current clockX value for calculation. */

u_int32 ReadIRam(register __i0 u_int16 addr); /**< Reads instruction RAM. */
void WriteIRam(register __i0 u_int16 addr, register __reg_a u_int32 ins); /**< Writes instruction RAM. */

/**
  Four interrupt stubs that the programmer can use without using ASM.
  For example to set up the first interrupt stub to GPIO0 interrupt,
  use the following:
  \code
    WriteIRam(0x20+INTV_GPIO0, ReadIRam((u_int16)InterruptStub0));
  \endcode

  Then you can enable GPIO0 interrupt and it will call your routine
  Interrupt0() whenever GPIO0 interrupt request is generated.
  \code
    PERIP(GPIO0_INT_FALL) |= DISPLAY_XCS;
    PERIP(INT_ENABLEL) |= INTF_GPIO0;
  \endcode
 */
void InterruptStub0(void);
void InterruptStub1(void); /**< Stub function that can be plugged to any interrupt vector. Calls Interrupt1(), which must be provided by the user. */
void InterruptStub2(void); /**< Stub function that can be plugged to any interrupt vector. Calls Interrupt2(), which must be provided by the user. */
void InterruptStub3(void); /**< Stub function that can be plugged to any interrupt vector. Calls Interrupt3(), which must be provided by the user. */
auto void Interrupt0(void); /**< Called by InterruptStub0. */
auto void Interrupt1(void); /**< Called by InterruptStub1. */
auto void Interrupt2(void); /**< Called by InterruptStub2. */
auto void Interrupt3(void); /**< Called by InterruptStub3. */


/** Initializes RC5 structure and installs interrupt handler.
    RC5 receiver uses one of the interruptable GPIO pins as input.
    Interrupt must be generated on both edges. The polarity of the
    receiver does not matter. Currently receives only codes that have
    two start bits.
    \code
    Rc5Init(INTV_GPIO0);
    PERIP(GPIO0_INT_FALL) |= (1<<14);
    PERIP(GPIO0_INT_RISE) |= (1<<14);
    PERIP(INT_ENABLEL) |= INTF_GPIO0;
    while (1) {
	register u_int16 t = Rc5GetFIFO();
	if (t) {
	    puthex(t);
	    puts("=got");
	}
    }
    \endcode
    See also example code rc5.c .
 */
void Rc5Init(u_int16 vector);
u_int16 Rc5GetFIFO(void); /**< Returns 0 if no values are available, received 14-bit value otherwise. */

#define RC5_SYS_TVSET          0
#define RC5_SYS_TELETEXT       2
#define RC5_SYS_VCR            5
#define RC5_SYS_EXPERIMENTAL7  7
#define RC5_SYS_PREAMP        16
#define RC5_SYS_RECEIVERTUNER 17
#define RC5_SYS_TAPERECORDER  18
#define RC5_SYS_EXPERIMENTAL19 19
#define RC5_SYS_CDPLAYER      20

#define RC5_CMD_0  0
#define RC5_CMD_1  1
#define RC5_CMD_2  2
#define RC5_CMD_3  3
#define RC5_CMD_4  4
#define RC5_CMD_5  5
#define RC5_CMD_6  6
#define RC5_CMD_7  7
#define RC5_CMD_8  8
#define RC5_CMD_9  9
#define RC5_CMD_ONETWODIGITS  10
#define RC5_CMD_STANDBY       12
#define RC5_CMD_MUTE          13
#define RC5_CMD_PRESET        14
#define RC5_CMD_DISPLAY       15
#define RC5_CMD_VOLUME_UP     16
#define RC5_CMD_VOLUME_DOWN   17
#define RC5_CMD_BRIGHT_UP     18
#define RC5_CMD_BRIGHT_DOWN   19
#define RC5_CMD_COLOR_UP      20
#define RC5_CMD_COLOR_DOWN    21
#define RC5_CMD_BASS_UP       22
#define RC5_CMD_BASS_DOWN     23
#define RC5_CMD_TREBLE_UP     24
#define RC5_CMD_TREBLE_DOWN   25
#define RC5_CMD_BALANCE_LEFT  26
#define RC5_CMD_BALANCE_RIGHT 27
#define RC5_CMD_CONTRAST_UP   28
#define RC5_CMD_CONTRAST_DOWN 29
#define RC5_CMD_PROGRAM_UP    32
#define RC5_CMD_PROGRAM_DOWN  33
#define RC5_CMD_ALTERNATE     34 /*P<P swap last channel */
#define RC5_CMD_LANGUAGE      35
#define RC5_CMD_EXPAND        36
#define RC5_CMD_TIMER         38
#define RC5_CMD_STORE         41
#define RC5_CMD_CLOCK         42
#define RC5_CMD_FINETUNE_PLUS 43  /* expand text-tv */
#define RC5_CMD_FINETUNE_MINUS 44 /* ? text-tv */
#define RC5_CMD_CROSS         45 /* X text-tv */
#define RC5_CMD_MIX           46 /* mix text-tv */
#define RC5_CMD_PAUSE         48
#define RC5_CMD_FAST_REVERSE  50
#define RC5_CMD_FAST_FORWARD  52
#define RC5_CMD_PLAY          53
#define RC5_CMD_STOP          54
#define RC5_CMD_RECORD        55
#define RC5_CMD_SWITCH        56 /*swap screens*/
#define RC5_CMD_MENU          59
#define RC5_CMD_TEXT          60
#define RC5_CMD_SYSTEM_SELECT 63



/** 20091016: IR Receive routines for NEC protocol. */
#define IR_MODE_MSB 0x8000
#define IR_MODE_LSB 0x0000
#define IR_MODE_BIT_MASK  0x003f /*16,32*/
#define IR_MODE_16BIT 0x0010
#define IR_MODE_32BIT 0x0020
struct FIFOX {
    u_int16 *wr;
    u_int16 *rd;
    u_int16 *start;
    u_int16 *lastPlusOne;
    u_int16 size;
};
extern struct IR_RECEIVE {
    u_int16 lastTime; /* 0 -- do not change order of fields! */
    int count;        /* 1 */
    u_int16 mode;     /* 2 */
    u_int16 timeCountLo; /* 3 */
    u_int32 data;     /* 4 -- larger data needed.. */
    struct FIFOX fifo; /* 6 */
} irReceive;
extern u_int32 irReceiveFIFO[8];
extern void IrReceiveInt(void);
void IrReceiveInit(u_int16 vector, u_int16 mode);
/** Use FIFO routines to read data. Check that data is available
    and slways read two words at a time using two
    FIFOGet(&irReceive.fifo); calls. */
/** Example setup using 32-bit NEC code and GPIO1_2 (SI).
    Rising-edge interrupt is used.
  \code
    #include <dev1000.h>
    IrReceiveInit(INTV_GPIO1, IR_MODE_32BIT | IR_MODE_LSB);
    PERIP(GPIO1_INT_RISE) |= (1<<2);
    PERIP(INT_ENABLEL) |= INTF_GPIO1;
    PERIP(GPIO1_MODE) &= ~(1<<2);
    PERIP(GPIO1_DDR)  &= ~(1<<2);
    while (1) {
        if (!FIFOEmpty(&irReceive.fifo)) {
            u_int16 cmd, addr;
            cmd = FIFOGet(&irReceive.fifo);
            addr = FIFOGet(&irReceive.fifo);
            if (cmd == 0 && addr == 0xffff) {
                repeat();
            }
        }
    }
  \endcode
 */



//
/**
   A replacement for MSCPacketFromPC() that can be used to divert SCSI
   commands to NewDiskProtocolCommand(). For example SCSI_START_UNIT can be
   used to turn the unit off or leave USB when 'eject' is pressed on PC.
   Now (20090316) also contains the LBAB patch, compatible with VS1000D.
   Now (20110628) renamed to PatchDiskProtocolCommandC (was
   PatchDiskProtocolCommand).
   \param SCSI command packet
 */
void PatchDiskProtocolCommandC(/*USBPacket **/ void *inPacket);
/**
   A replacement for MSCPacketFromPC(); for VS1000D and VS1000E, that can be
   used to divert SCSI commands to NewDiskProtocolCommand(). For example
   SCSI_START_UNIT can be used to turn the unit off or leave USB when
   'eject' is pressed on PC. Also contains patch for SCSI_READ_10,
   which makes VS1000 behave much better with Linux.
   \param SCSI command packet
 */
void PatchDiskProtocolCommandDE(/*USBPacket **/ void *inPacket);
/**
   NewDiskProtocolCommand() processes SCSI commands.
   For example SCSI_START_UNIT can be used to turn the unit off
   or leave USB when 'eject' is pressed on PC.
   \param pointer to the SCSI command structure
   \return SCSI command, or -1 if already performed the command.
    \code
    auto u_int16 NewDiskProtocolCommand(register __i2 u_int16 *cmd) {
      int c = (cmd[OPERATION_CODE] & 0xff);
      if (c == SCSI_START_STOP_UNIT) {
        startUnit = cmd[2] & 3;
      } else {
	if (startUnit == 2) {
	    startUnit = 4;
	}
	if (c == SCSI_REQUEST_SENSE) {
	    SCSI.DataBuffer[0] = 0x7000;
	    SCSI.DataOutBuffer = SCSI.DataBuffer;
	    SCSI.DataOutSize = 18;
	    SCSI.State = SCSI_DATA_TO_HOST;
	    return -1;
	}
      }
      return cmd[OPERATION_CODE];
    }
    \endcode
 */
auto u_int16 NewDiskProtocolCommand(register __i2 u_int16 *cmd);

/*
  Shuffle routines.
 */
extern __y u_int32 shuffleSeeds; /**< Seed values for Shuffle(). */
/** Routine to select files in a random order as opposed to randomly
    select the next file. When all files have been played once, a
    new order is selected. A file is not played twice in a row.
    \param numFiles The number of files.
    \param oldFile  Previously played file.
    \return The file to play next, or oldFile is numFiles is 0 or 1.
    \code
    if (player.randomOn) {
      player.currentFile = Shuffle(player.totalFiles, player.currentFile);
    } else {
      player.currentFile = player.nextFile;
    }
    \endcode
 */
u_int16 Shuffle(register __c0 u_int16 numFiles, register __c1 u_int16 oldFile);

/** A routine that will run with 1/512 clock until keys are pressed,
    changes in USB pins are detected, or the wakeup time is reached.
    This routine will keep the timeCount variable accurate.
    Interrupts will be disabled from INT_ENABLEL during LowClock.
    Before calling this function 1.0x clock should be active,
    key IO's should be in GPIO mode and as inputs, and INT_ENABLEH
    should be 0. Analog powerdown bits can also be set and voltages
    lowered to get very low power consumption.
    LowClock assumes 12.0MHz clock.

    \param wakeupIoKeys Mask for keys that should cause wakeup.
    \param wakeupTime   timeCount value (not interval!) that causes wakeup.
 */
void LowClock(register __a1 u_int16 wakeupIoKeys, register __reg_c u_int32 wakeupTime);


/** Miscellaneous display routines for serial LCD/OLED display.
    Uses CS2 for LCD/OLED chip select. Uses SI for data/command select.
 */
struct lcdControl {
    u_int16 X, Y, invertMask, options; /*20111005 added options*/
#define LCD_OPTION_LATIN1 (1<<0) /* print using Latin1 font if set (129w)*/
};
extern struct lcdControl lcdControl;
#include <vs1000.h> /*for PERIP()*/
/*Note: macros are not used in ASM, so do not change them! */
#define LCD_SET_COMMAND_MODE PERIP(GPIO1_CLEAR_MASK) = 0x0004
#define LCD_SET_DATA_MODE    PERIP(GPIO1_SET_MASK) = 0x0004
#define LCD_SELECT           PERIP(GPIO0_CLEAR_MASK) = 0x4000 //CS2
#define LCD_END              PERIP(GPIO0_SET_MASK) = 0x4000 //CS2
#define OLED_INIT_STRING     "\xae\xd3\x00\xa8\x3f\xad\x8b\x40\xa0\xc8\x81\x30\xb0\x00\x10\xa4\xa6\xaf\xd8\x05\xffff"

/** Clears the lcdControl structure, sets up GPIO mode registers,
    then sends the initialization commands to LCD. End marker in
    the init string is a negative value.
    \param init LCD init string, OLED_INIT_STRING for VS1000 Developer Board
 */
void LcdInit(register __i0 const unsigned char *init);
void LcdClear(void); /**< Clears LCD: LCD_HEIGHT * LCD_WIDTH bytes. */
/** Sends a locate command to display.
    \param x the horizontal position in pixels
    \param y the vertical position in 8-pixel rows
 */
void LcdLocate(register __b0 int x, register __b1 int y); /**< goto (x,y) */
/** Puts a data byte to LCD. Wraps automatically after LCD_WIDTH.
    \param c - the data to write, low 8 bits are used. */
void LcdPutData(register __a1 u_int16 c);
/** Puts a character to LCD using the rom font. Characters will not be
    split between lines. If a character does not fit into line, it is
    put on the next line.
    \param c - character code, >132 are fixed-width characters.
 */
void LcdPutChar(register __c0 u_int16 c);
/** Puts a character to LCD using the rom font. Lines do not wrap.
    If a character does not fit into line, zero-fills the rest of
    the line and returns 1.
    \param c - character code, >132 are fixed-width characters.
    \return 0 for success, 1 if the char did not fit.
 */
u_int16 LcdPutCharNoWrap(register __c0 u_int16 c);
/** Puts packed data to LCD.
    \param d - start of packed data, for example "\p\xff\xfe\xfc\xf8\xf0\xe0"
    \param words - the number of words to send (two bytes per word), must be non-zero.
*/
auto void LcdPutGfx(register __i1 const u_int16 *d,
		    register __c0 u_int16 words);
#define LcdPutGfxY(a,b) LcdPutGfx((void*)(a),-(b))

/** Puts packed data to LCD with double width.
    \param d - start of packed data, for example "\p\xff\xfe\xfc\xf8\xf0\xe0"
    \param words - the number of words to send (two bytes per word).
*/
auto void LcdPutGfxDouble(register __i1 const u_int16 *d,
			  register __c0 u_int16 words);
/** Puts shifted packed data to LCD. Two data lines must be consequtive,
    i.e. s1[i] and s1[LCD_WIDTH/2+i] are used as source data for the shift.
    \param s1 - start of packed data
    \param words - the number of words to send (two bytes per word).
    \param shift - 0 for no shift, or negative shift amount
 */
auto void LcdPutGfxShifted(register __i1 const u_int16 *s1,
			   register __c0 u_int16 words,
			   register __c1 s_int16 shift);
/** Prints a packed string to LCD. Supports also strings in Y.
    \param p - packed string in X memory to print
    \param maxWords - max number of words to process
    \param xEnd - the line upto xEnd will be filled if the string was shorter.
 */
auto void LcdPutPackedX(register __i1 const u_int16 *p,
			register __c0 u_int16 maxWords,
			register __c1 u_int16 xEnd);
#define LcdPutPackedY(a,b,c) LcdPutPackedX((void *)(a),(b),~(c))
/** Sets the OLED contrast.
    \param contrast the new contrast value (0..255).
*/
auto void LcdContrast(register __c0 u_int16 contrast);
/** Prints the parameter in decimal. At least 2 digits will be printed.
    \param c - the number to print. */
void LcdPutNum(register __c1 u_int16 c);
/** Prints the parameter in decimal. Always prints at least 1 digit.
    \param c - the number to print. */
void LcdPutLong(register __reg_b u_int32 c);
/** Renders as many characters as possible of the packed string to the
    graphics buffer. Uses word wrap if the whole string does not fit.
    The return value can be used for continuing rendering until the whole
    string is processed.
    \code
      memset(&gfx[0][0], 0, LINES*LCD_WIDTH/2);
      t = LcdRenderPackedX(gfx[0], 0, nextLine, 0);
      if (//t < len
          (nextLine[t/2] & 0xff00) &&
	  (nextLine[t/2] & 0x00ff)) {
        // If not all fits, render another line
        t = LcdRenderPackedX(gfx[1], 0, nextFile, t);
      } else {
         LcdRenderDoubled(gfx[0], gfx[1]);
      }
      LcdLocate(0, 1);
      LcdPutGfxShifted(gfx[0], 2*LCD_WIDTH/2, 0); //print two lines
    \endcode

    \param buf  - the output graphics buffer (at least LCD_WIDTH/2 words)
    \param bIdx - render start position in the graphics buffer
    \param str  - packed string to render
    \param sIdex - render start position in the string (in bytes)
    \return the number of characters successfully rendered
 */
auto int LcdRenderPackedX(register __i0 u_int16 *buf,
			  register __c0 u_int16 bIdx,
			  register __i1 const u_int16 *str,
			  register __c1 u_int16 sIdx);
/** Doubles the graphics buffer data in vertical direction.
    \param buf1 - packed input data, top half of packed output
    \param buf2 - low half of packed output
 */
auto void LcdRenderDoubled(register __i0 u_int16 *buf1,
			   register __i2 u_int16 *buf2);
/** 20111005
    Fills until the beginning of a line with 0 (using invertMask).
    Does nothing if lcdControl.X is already 0.
 */
void LcdEol(void);

/** Initializes IRAM vectors to default states. This includes interrupt
    vectors. Does not set variables, so for example currentKeyMap should
    be restored in addition to calling ResetIRAMVectors().
    20110809: trashed a0 and a1 (bug fixed)
 */
void ResetIRAMVectors(void);

/** FLASH write routine for large-page (2048+64 bytes) memories.
    This routine must be used with multi-level-cell memories, because
    writing 512 bytes at a time is not possible with them.
    Three bytes of error correction code is inserted into all quarters
    of the metadata separately. This ECC is compatible with normal
    Read() function, so the 512-byte subsectors can be separately read.
    \param page - 512-byte sector number (2048-byte sector number * 4)
    \param data - 2048 bytes of data, must be non-null!
    \param meta - 64 bytes of meta, must be non-null!
 */
s_int16 LargePageWrite(u_int32 page, u_int16 *data, u_int16 *meta);

/** FLASH read routine for large-page (2048+64 bytes) memories.
    This routine is faster than reading four 512-byte sectors sequentially.
    \param page - 512-byte sector number (2048-byte sector number * 4)
    \param data - 2048 bytes of data, must be non-null!
    \param meta - 64 bytes of meta, must be non-null!
 */
s_int16 LargePageRead(s_int32 page, u_int16 *data, u_int16 *meta);

/** Part of MLC mapper implementation. Uses the fsMapTiny structure fields
    to store data and uses the MLC-specific mapping table to locate sectors.
    This is the minimal version which can only read one sector at a time,
    but currently this is enough for all VS1000 subsystems.
    \param map - not currently used, fsMapTiny is always used
    \param sector - 512-byte sector number
    \param logicalBlocks - must be 1
    \param data - pointer to data buffer
    return number of sectors read (currently always 1)
 */
s_int16 FsMlcMapRead1(struct FsMapper *map, u_int32 sector,
 		     u_int16 logicalBlocks, u_int16 *data);
/** Part of MLC mapper implementation. Uses the fsMapTiny structure fields
    to store data and uses the MLC-specific mapping table to locate sectors.
    This is a larger version which can only read more than one sector at
    a time. Currently all VS1000 subsystems read one sector at a time,
    so see also the shorter version FsMlcMapRead1().
    \param map - not currently used, fsMapTiny is always used
    \param sector - 512-byte sector number
    \param logicalBlocks - must be 1
    \param data - pointer to data buffer
    return number of sectors read (currently always 1)
 */
s_int16 FsMlcMapRead(struct FsMapper *map, u_int32 sector,
 		     u_int16 logicalBlocks, u_int16 *data);

/** Bit array manipulation -- read a bit.
    \param array - storage for the bits
    \param bit - the bit to get
    return 0 for bit unset, non-zero for bit set
 */
u_int16 GetBitArray(register __i0 u_int16 *array, register __a0 int bit);
/** Bit array manipulation -- set a bit.
    \param array - storage for the bits
    \param bit - the bit to set
 */
void SetBitArray(register __i0 u_int16 *array, register __a0 int bit);
/** Bit array manipulation -- clear a bit.
    \param array - storage for the bits
    \param bit - the bit to clear
 */
void ClearBitArray(register __i0 u_int16 *array, register __a0 int bit);

/** Workaround for playback with badly encoded granule positions.
    cs.Read = OggGranulePatch;
    \BUG: This routine does not work! It breaks the time code.
 */
//u_int16 OggGranulePatch(struct CodecServices *cs, u_int16 *data,
//			u_int16 firstOdd, u_int16 bytes);


u_int32 CheckSumBlock(register u_int32 __reg_a accu, register __i0 u_int16 *p,
 		      register __c0 s_int16 words);

int stricmppacked(__near const u_int16 register __i0 *cs, 
		  __near const u_int16 register __i1 *ct);

/** 20091015: General-purpose FIFO routines */
/** Initialize FIFO. The buffer must be allocated by the user, but
    the alignment does not matter.
    \param f - FIFO structure
    \param fifoBuf - storage for size words
    \param size - FIFO buffer size in words
*/
void FIFOInit(register __i2 struct FIFOX *f, register __a1 void *fifoBuf, register __a0 s_int16 size);
/** Put a word to FIFO. Use FIFOSpace() to see if there is space.
    Do not write to a full FIFO, it becomes magically empty.
    Note that a FIFO can hold only size-1 words.
    \param f - FIFO structure
    \param data - a word to put into the FIFO
 */
void FIFOPut(register __i2 struct FIFOX *f, register __a0 u_int16 data);
/** Get a word from FIFO. Use !FIFOEmpty() or FIFOFill() to see if there is
    data available. Do not read from an empty FIFO, or it becomes magically
    filled.
    \param f - FIFO structure
    returns the oldest data
*/
u_int16 FIFOGet(register __i2 struct FIFOX *f);
/** Returns 1 if FIFO is empty, 0 if there is data to read. */
s_int16 FIFOEmpty(register __i2 struct FIFOX *f);
/** Returns the number of words in the FIFO. */
s_int16 FIFOFill(register __i2 struct FIFOX *f);
/** Returns the number of words free space in the FIFO. */
s_int16 FIFOSpace(register __i2 struct FIFOX *f);

/* 20100610 */
/** Does not wait for EarSpeaker to be emptied after playing a file.
    This makes the routine to return faster than the default version.
    See also: PatchPlayCurrentFile.
    \code
      SetHookFunction((u_int16)PlayCurrentFile, PlayCurrentFileFast);
    \endcode
*/
u_int16 PlayCurrentFileFast(void);

/** Calculates ECC bytes for 512-byte sector (256 words) without using
    the ECC calculation logic of the NAND-FLASH interface.
    \param dp - data buffer pointer
    returns ECC2 in bits 23..16, ECC1 in bits 15..8 and ECC0 in bits 7..0.
*/
u_int32 CalcECCBySoftware(register __i2 s_int16 *dp);

/** If cs.playTimeSeconds is >= 0, reverses or advances the specified
    amount of seconds. If cs.playTimeSeconds < 0, returns without
    doing anything.
    \param secs - the number of seconds to jump (negative for backward)
 */
void VorbisSkip(s_int16 secs);

/** 20100630 - 14 words. Allows file data to be read after the ~560MB
    position, if the file after that position is not fragmented.
    Use the following code in your main function initialization:
    \code
      SetHookFunction((u_int16)ReadFile, PatchFatReadFile);
    \endcode
 */
auto s_int16 PatchFatReadFile(register __i3 u_int16 *buf,
			      register __c1 s_int16 byteOff,
			      register __c0 s_int16 byteSize
			      /*<0 for little-endian order*/);


/** 20100708 - Experimental! Speed/pitch shifter.
	       Note: speedshifter is skipped when Vorbis frame > 2048
	             (only when -q -1 and samplerate is 44100Hz or higher)
		     or when earspeaker spatial processing is active.
    20110606 - Slightly smaller, but required some floating-point functions,
               so see SpeedShiftFract.
    \code
    #include <dev1000.h>

    InitSpeedShift(1.0);
    SetHookFunction((u_int16)StereoCopy, SpeedShift);
    ...
    NewSpeedShift(0.95);
    \endcode
 */
auto void SpeedShift(register __i2 s_int16 *p, register __a0 u_int16 n);
void InitSpeedShift(double speed);
void NewSpeedShift(double speed);

/** 20110607 - Experimental Speed/pitch shifter. Smaller (499 words) and
    does not require floating point libraries.
    Note: speedshifter is skipped when Vorbis frame > 2048
    (only when -q -1 and samplerate is 44100Hz or higher)
    or when earspeaker spatial processing is active.
    The parameter is now play speed divided by 2.
    \code
    #include <dev1000.h>

    InitSpeedShiftFract(1.0/2);
    SetHookFunction((u_int16)StereoCopy, SpeedShiftFract);
    ...
    NewSpeedShiftFract(0.95/2);
    \endcode
 */
auto void SpeedShiftFract(register __i2 s_int16 *p, register __a0 u_int16 n);
void InitSpeedShiftFract(f_int16 speedP2);
void NewSpeedShiftFract(f_int16 speedP2);

/** 20110610 - Experimental Speed/pitch shifter for only mono files.
    Smaller (467 words) and does not require floating point libraries,
    and can run with EarSpeaker active. Deactivates automatically for
    stereo files.
    Note: speedshifter is skipped when Vorbis frame > 2048
    (only when -q -1 and samplerate is 44100Hz or higher).
    The parameter is play speed divided by 2.
    \code
    #include <dev1000.h>

    InitSpeedShiftMono(1.0/2);
    SetHookFunction((u_int16)StereoCopy, SpeedShiftFract);
    ...
    NewSpeedShiftMono(0.95/2);
    \endcode
 */
auto void SpeedShiftMono(register __i2 s_int16 *p, register __a0 u_int16 n);
void InitSpeedShiftMono(f_int16 speedP2);
void NewSpeedShiftMono(f_int16 speedP2);

/** 20101207 (Not tested!)
    Replacement for FatOpenFile that is normally in OpenFile hook.
    This version disables subdirectories for all filesystems.

    SetHookFunction((u_int16)OpenFile, FatOpenFileNoSubdirs);
    \param fileNum the same as for OpenFile
    \return the same as OpenFile
*/
auto u_int16 FatOpenFileNoSubdirs(register __c0 u_int16 fileNum);

/** 20110428: (38 words)
    Fixes a decoding problem: right channel is not played if left channel
    is empty (digital zero).
    Also does not wait for EarSpeaker to be emptied after playing a file.
    This makes the routine to return faster than the default version.
    20140123: does not add any zero samples, used to add 32.
    20141121: Bug fix! 20140123 version was not restoring registers correctly!
    \code
      SetHookFunction((u_int16)PlayCurrentFile, PatchPlayCurrentFile);
    \endcode
 */
u_int16 PatchPlayCurrentFile(void);

/** 20110505: 126 words
    Single-sector (512 bytes) read function for NAND FLASH with 8kB page.
    Note: ignores the "sectors" parameter!
    Can replace ph->Read() if your own code does not call it
    with more than one sector at a time.
      ph->Read = Large8kPageReadSingle;
 */
s_int16 Large8kPageReadSingle(struct FsPhysical *ph, u_int32 page,
 			      u_int16 sectors, u_int16 *data, u_int16 *meta);
void NandPut8kPageSpareAddress(register __reg_c u_int32 addr);
void NandPut8kDataAddress(register __reg_c u_int32 addr);
/** 20110505: 133 words, stack usage about 44 words.
    Single-page read function for NAND FLASH with 8kB page.
    Reads 8kB of data and 256 bytes of spare. Both data and
    meta pointers need to be present. Returns 0 for success,
    other values (currently -1) for uncorrectable errors.
 */
s_int16 Large8kPageRead(u_int32 page, u_int16 *data, u_int16 *meta);
/** 20110517: 135 words, stack usage about 44 words.
    Single-page read function for NAND FLASH with 8kB page.
    Reads 8kB of data and 256 bytes of spare. Both data and
    meta pointers need to be present. Returns 0 for success,
    0x8000 if any sector was corrected by ECC (20110516),
    other values (currently -1) for uncorrectable errors.
 */
s_int16 Large8kPageReadForCopyBack(u_int32 page, u_int16 *data, u_int16 *meta);
/** 20110505: 71 words, stack usage about 8 words.
    Single-page write function for NAND FLASH with 8kB page, compatible
    with MLC. Writes 8kB of data and 256 bytes of spare with separate
    ECC for each 512-byte sector. Returns 0 for success.
*/
s_int16 Large8kPageWrite(u_int32 page, u_int16 *data, u_int16 *meta);


/** 20110622
    Replacement for FatOpenFile that is normally in OpenFile hook.
    This version speeds up directory processing by detecting end of
    directory instead of reading to the end of the cluster.
    This is significant when using a large card with a large
    sectors-per-cluster value and there are a lot of directories.

    The code also fixes a cluster chain problem for FAT32 root
    directory (when created in Linux).

    The code also fixes some long-filename problems, but does not
    fix fat12 issues (those are fixed in VS1000D and VS1000E).
    Is compatible with VS1000B/C, VS1000D, VS1000E.

    SetHookFunction((u_int16)OpenFile, PatchOpenFile);
    \param fileNum the same as for OpenFile
    \return the same as OpenFile
*/
auto u_int16 PatchOpenFile(register __c0 u_int16 fileNum);

/** 20110713
    Reads RC circuit down-going delay from a GPIO0 pin.

    This can be used to read potentiometer values, for example
    100k potentiometer in series with 100k resistor, with 0.5nF parallel
    capacitor gives 50us to 100us delays (values 300..600). The used
    pin should not have any other pull-ups or pull-downs.

    Uses timer 1, leaves it running. Requires the gpio pin to be in gpio mode.
    Leaves the specified gpio high and as output after conversion.
    For stable results the gpio should be high for long enough before the call.
    \code
u_int16 ControlAdc(register __c0 u_int16 gpio, register __c1 u_int16 maxTime) {
    register int val;
    PERIP(TIMER_ENABLE) |= 2;
    PERIP(TIMER_T1CNTL) = maxTime;
    PERIP(INT_ORIGIN) = INTF_TIM1;
    PERIP(GPIO0_DDR) &= ~gpio;
    while (1) {
	register int x = (PERIP(GPIO0_IDATA) & gpio);
	if (x==0 || (PERIP(INT_ORIGIN) & INTF_TIM1))
	    break;
    }
    val = maxTime - PERIP(TIMER_T1CNTL);
    PERIP(GPIO0_DDR) |= gpio;
    PERIP(GPIO0_SET_MASK) = gpio;
    return val;
}
    \endcode
    \code
    {
        s_int16 val = ControlAdc(0x4000, 640);
	val -= 0x13c;
	if (val < 0)
	    val = 0;
	if (val > 0x108)
	    val = 0x108;
    }
    \endcode

    \param gpio - GPIO0 pin to use (mask)
    \param maxTime - timeout, slightly larger than max returned value
 */
u_int16 ControlAdc(register __c0 u_int16 gpio, register __c1 u_int16 maxTime);

/** Send 0xffff and receive data from SPI0. */
void SpiReadData(register __i2 u_int16 *data, register __a1 s_int16 words);
/** Receive data, return one additional value as return value. */
u_int16 SpiReadDataDiscardOne(register __i2 u_int16 *data, register __a1 s_int16 words);


/** 20120816:
    Single-sector (512 bytes) read function for NAND FLASH with 4kB page.
    Note: ignores the "sectors" parameter!
    Can replace ph->Read() if your own code does not call it
    with more than one sector at a time.
      ph->Read = Large4kPageReadSingle;
 */
s_int16 Large4kPageReadSingle(struct FsPhysical *ph, u_int32 page,
 			      u_int16 sectors, u_int16 *data, u_int16 *meta);
void NandPut4kPageSpareAddress(register __reg_c u_int32 addr);
void NandPut4kDataAddress(register __reg_c u_int32 addr);
/** 20120816:
    Single-page read function for NAND FLASH with 4kB page.
    Reads 4kB of data and 128 bytes of spare. Both data and
    meta pointers need to be present. Returns 0 for success,
    other values (currently -1) for uncorrectable errors.
 */
s_int16 Large4kPageRead(u_int32 page, u_int16 *data, u_int16 *meta);
/** 20110816:
    Single-page read function for NAND FLASH with 4kB page.
    Reads 4kB of data and 128 bytes of spare. Both data and
    meta pointers need to be present. Returns 0 for success,
    0x8000 if any sector was corrected by ECC,
    other values (currently -1) for uncorrectable errors.
 */
s_int16 Large4kPageReadForCopyBack(u_int32 page, u_int16 *data, u_int16 *meta);
/** 20120816:
    Single-page write function for NAND FLASH with 4kB page, compatible
    with MLC. Writes 4kB of data and 128 bytes of spare with separate
    ECC for each 512-byte sector. Returns 0 for success.
*/
s_int16 Large4kPageWrite(u_int32 page, u_int16 *data, u_int16 *meta);

/** 20121015:
    Calculate ECC from 256 words of data. Data is masked with 16-bit value.
 */
u_int32 EccHw(register __i2 u_int16 *data, register __c0 u_int16 mask);
/** 20121017:
    Fix one-bit error from the data according to xored ECC bytes.
    returns
    0 for no errors (xored=0)
    -1 for fail
    0x8000 for successful correction.
*/
u_int16 EccFixError(register __i2 u_int16 *data, register __c u_int32 xored);
/** 20121017:
    Single-page read function for NAND FLASH with 4kB page.
    Reads 4kB of data and 128 bytes of spare. Both data and
    meta pointers need to be present. Returns 0 for success,
    other values (currently -1) for uncorrectable errors.
    Additional second ECC with mask LP4K_ECC_MASK must be present at...
 */
s_int16 Large4kPageRead2(u_int32 page, u_int16 *data, u_int16 *meta);
/** 20120816:
    Single-sector (512 bytes) read function for NAND FLASH with 4kB page.
    Note: ignores the "sectors" parameter!
    Can replace ph->Read() if your own code does not call it
    with more than one sector at a time.
      ph->Read = Large4kPageReadSingle2;
    Additional second ECC with mask LP4K_ECC_MASK must be present at...
 */
s_int16 Large4kPageReadSingle2(struct FsPhysical *ph, u_int32 page,
 			      u_int16 sectors, u_int16 *data, u_int16 *meta);

auto u_int32 WavSwap32(register __c u_int32 n);

/** 20140123:  */
#define HAS_PATCHCODVORBISDECODE
enum CodecError PatchCodVorbisDecode(struct Codec *cod, struct CodecServices *cs,
				  const char **errorString, int dummy);

/** 20140916:
    Determines the total playtime of the currently opened Ogg Vorbis file.
    Leaves the file position at the beginning of the file. Assumes the
    file position was at the beginning of the file (uses Seek instead of
    cs.Seek).
    \param sc - scaling factor for the output. The length in seconds is
                returned if the parameter is 0. Value 4 means the return
		value is 1/16th of seconds. (Other values can also be used.)
    \return the total playtime of the file in seconds, scaled with sc.
           If the file is not an Ogg Vorbis file, 
*/
u_int32 PlayTimeFile(s_int16 sc/*Scale 0:seconds, 4:1/16th secs*/);

/** A faster named file open routine. Note that only the short filename
    is checked. suffix must be one of the FAT supportedSuffixes.
    \param fname The 8-character name must be in upper case and in packed format padded with spaces. For example "\pMYFILE  ".
    \param suffix The desired suffix, for example FAT_MKID('O','G','G')
    \return The file index within files of supportedSuffixes,
            0xffffU if file is not found.
*/
auto s_int16 OpenFileNamedSupported(const u_int16 *fname, u_int32 suffix);

/** 20150710 A ROM test routine for VS1000D.
    \return 0 for success, bit mask for failure: 4 for IROM, 2 for XROM, 1 for YROM
 */
s_int16 VS1000D_RomTest(void);

#endif/*!ASM*/
#endif /*__DEV1000_H__*/
