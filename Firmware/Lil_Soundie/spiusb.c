/// \file spiusb.c Winbond 25X16 SPI flash/USB/Vorbis player demo project
/*

  Copyright 2008 VLSI Solution, Tampere, Finland. Absolutely no warranty.


  S P I   F L A S H   U S B   D R I V E  +  P L A Y E R   E X A M P L E
  ---------------------------------------------------------------------

  This example code uses the 18 kilobytes of mallocAreaY to implement
  a disk read/write cache and work area that is capable of erasing and
  programming an SPI flash chip such as the Winbond 25X16 in 4 kilobyte
  chunks when the VS1000B is connected to USB. Tested with Windows XP.

  Note: The SPI flash is quite slow to write to (about the speed of
  a floppy disk). Writing small files (a few kilobytes) is fast due
  to the cache system. Large files may cause occasional timeouts with
  slow flash chips.

  Requirements: An SPI flash eeprom with 4 kilobyte erasable blocks.

  Tested with VS1000B, Winbond 25X16 and Windows XP Professional.
  Measured performance: 1 megabyte file was written in 55 seconds when
  the eeprom was blank (no need to erase). When writing over already
  written data, writing the same file took 1 minute 35 seconds.

  Builds with vskit133 build script (BUILD SPIUSB)
  eeprom.img can be then prommed to the eeprom for bootable system.
  Then you can use your development board to make a master EEPROM image,
  which you can copy to the production units using an eeprommer.

  At first connect the USB disk is unformatted.
  Suggestion: use "Quick Format" when formatting the disk with WinXP.


  FAT12 Subdirectory ("Folder") Limitations
  -----------------------------------------
  This version of code does not handle FAT12 subdirectories. Windows
  forces all disks smaller than 16 megabytes to be FAT12. VLSI has
  a binary fix for the problem, but it is not distributable in C code form.
  You can request the FAT12 fix library from support@vlsi.fi for deployment
  of your product.

  The pre-compiled example image (vs1000b_25X16_spiflash_usb_player.img)
  on VLSI web page has the fix installed.

  - Description of FAT12 subdirectory limitation:
    FAT12 subdirectory handling is omitted by choice from the VS1000B ROM,
    because it requires special handling and FAT12 was not seen to be
    necessary for player applications. This has the effect of hanging up the
    ROM code when the ROM code looks for Ogg Vorbis files to be played,
    if the disk has a FAT12 subdirectory. If your code does not try to play
    Vorbis files, then there is no problem.

    Our binary patch allows the ROM functions to successfully skip the
    subdirectories without crashing. Vorbis files are playable from
    the root directory.


*/

#ifdef USE_WAV
#define PLAYFILE PlayCurrentFile
#else
#define PLAYFILE PlayWavOrOggFile
#endif

// The number of 512-byte blocks totally available in the SPI Flash chip
#define CHIP_TOTAL_BLOCKS 4096  /* 4096 * 512 bytes = 2M (Winbond 25X16) */

// Set aside some blocks for VS1000 boot code (and optional parameter data)
#define RESERVED_BLOCKS 32

#define LOGICAL_DISK_BLOCKS  (CHIP_TOTAL_BLOCKS-RESERVED_BLOCKS)

#define PRINT_VS3EMU_DEBUG_MESSAGES 0

#define SPI_CLOCK_DIVIDER 2

// number of 512-blocks in RAM cache (MUST BE 16 DO NOT ALTER! I MEAN IT!)
#define CACHE_BLOCKS 16

// Position of workspace in Y ram, do not change.
#define WORKSPACE (mallocAreaY + 6144)

// storing the disk data inverted is optimal for the system.
// storing the disk data uninverted (as is) makes it easier to debug the SPI image
#define USE_INVERTED_DISK_DATA 1

#include "system.h"

#include <stdio.h> //Standard io
#include <stdlib.h> // VS_DSP Standard Library
#include <vs1000.h> // VS1000B register definitions
#include <vectors.h> // VS1000B vectors (interrupts and services)
#include <minifat.h> // Read Only Fat Filesystem
#include <mapper.h> // Logical Disk
#include <string.h> // memcpy etc
#include <player.h> // VS1000B default ROM player
#include <audio.h> // DAC output
#include <codec.h> // CODEC interface
#include <vsNand.h>
#include <mappertiny.h>
#include <usb.h>

#define DT_LANGUAGES 0
#define DT_VENDOR 1
#define DT_MODEL 2
#define DT_SERIAL 3
#define DT_DEVICE 4
#define DT_CONFIGURATION 5

#define VENDOR_NAME_LENGTH 7
const u_int16 myVendorNameStr[] = {
  ((VENDOR_NAME_LENGTH * 2 + 2)<<8) | 0x03,
  'V'<<8,'l'<<8,'s'<<8,'i'<<8,'F'<<8,'i'<<8,'n'<<8};

#define MODEL_NAME_LENGTH 8
const u_int16 myModelNameStr[] = {
  ((MODEL_NAME_LENGTH * 2 + 2)<<8) | 0x03,
  'S'<<8,'P'<<8,'I'<<8,'S'<<8,'t'<<8,'o'<<8,'r'<<8,'e'<<8};

#define SERIAL_NUMBER_LENGTH 12
u_int16 mySerialNumberStr[] = {
  ((SERIAL_NUMBER_LENGTH * 2 + 2)<<8) | 0x03,
  '0'<<8, // You should put your own serial number here
  '0'<<8,
  '0'<<8,
  '0'<<8,
  '0'<<8,
  '0'<<8,
  '0'<<8,
  '0'<<8,
  '0'<<8,
  '0'<<8,
  '0'<<8,
  '1'<<8, // Serial number should be unique for each unit
};

// This is the new Device Descriptor. See the USB specification!
const u_int16  myDeviceDescriptor [] = "\p"
  "\x12" // Length
  "\x01" // Type (Device Descriptor)
  "\x10" // LO(bcd USB Specification version) (.10)
  "\x01" // HI(bcd USB Specification version) (1.)
  "\x00" // Device Class (will be specified in configuration descriptor)
  "\x00" // Device Subclass
  "\x00" // Device Protocol
  "\x40" // Endpoint 0 size (64 bytes)

  "\xfb" // LO(Vendor ID) (0x19fb=VLSI Solution Oy)
  "\x19" // HI(Vendor ID)

  "\xe0" // LO(Product ID) (0xeee0 - 0xeeef : VLSI Customer Testing)
  "\xee" // HI(Product ID) (customers can request an ID from VLSI)

  "\x00" // LO(Release Number)
  "\x00" // HI(Release Number)
  "\x01" // Index of Vendor (manufacturer) name string
  "\x02" // Index of Product (model) name string (most shown by windows)
  "\x03" // Index of serial number string
  "\x01" // Number of configurations (1)
;

// When a USB setup packet is received, install our descriptors
// and then proceed to the ROM function RealDecodeSetupPacket.
void RealInitUSBDescriptors(u_int16 initDescriptors);
void MyInitUSBDescriptors(u_int16 initDescriptors){
  RealInitUSBDescriptors(1); // ROM set descriptors for mass storage
  USB.descriptorTable[DT_VENDOR] = myVendorNameStr;
  USB.descriptorTable[DT_MODEL]  = myModelNameStr;
  USB.descriptorTable[DT_SERIAL] = mySerialNumberStr;
  USB.descriptorTable[DT_DEVICE] = myDeviceDescriptor;
  //USB.descriptorTable[DT_CONFIGURATION] = myConfigurationDescriptor;
  //USB.configurationDescriptorSize = CONFIG_DESC_SIZE;
}

#if 0
// this can be helpful in making a serial number
const u_int16  bHexChar16[] = { // swapped Unicode hex characters
  0x3000, 0x3100, 0x3200, 0x3300, 0x3400, 0x3500, 0x3600, 0x3700,
  0x3800, 0x3900, 0x4100, 0x4200, 0x4300, 0x4400, 0x4400, 0x4500
};My
void MakeSerialNumber(u_int32 newSerialNumber){
  u_int16 i;
  u_int32 mySerialNumber = newSerialNumber;
  // Put unique serial number to serial number descriptor
  for (i=5; i<13; i++){
    mySerialNumberStr[i]=bHexChar16[mySerialNumber>>28];
    mySerialNumber <<= 4;
  }
}
#endif

extern struct FsNandPhys fsNandPhys;
extern struct FsPhysical *ph;
extern struct FsMapper *map;
extern struct Codec *cod;
extern struct CodecServices cs;
extern u_int16 codecVorbis[];

/* cache info */
u_int16 blockPresent;
u_int16 blockAddress[CACHE_BLOCKS];
s_int16 lastFoundBlock = -1;
u_int16 shouldFlush = 0;

enum CodecError PlayWavOrOggFile(void);

// Do we want to get debug screen output? It's available if the code
// is loaded with vs3emu (build script) with RS-232 cable.

#if PRINT_VS3EMU_DEBUG_MESSAGES
__y const char hex[] = "0123456789abcdef";
void puthex(u_int16 a) {
  char tmp[6];
  tmp[0] = hex[(a>>12)&15];
  tmp[1] = hex[(a>>8)&15];
  tmp[2] = hex[(a>>4)&15];
  tmp[3] = hex[(a>>0)&15];
  tmp[4] = ' ';
  tmp[5] = '\0';
  fputs(tmp, stdout);
}
void PrintCache(){
  register u_int16 i;
  for (i=0; i<CACHE_BLOCKS; i++){
    if (blockPresent & 1 << i){
      puthex(blockAddress[i]);
    }else{
      fputs("- ",stdout);
    }
  }
  puts ("=cache");
}
#define do__not__puts(x) puts(x)
#define do__not__puthex(x) puthex(x)
#else
#define do__not__puts(a)
#define do__not__puthex(a)
#define PrintCache()
#endif

#define SPI_EEPROM_COMMAND_WRITE_ENABLE  0x06
#define SPI_EEPROM_COMMAND_WRITE_DISABLE  0x04
#define SPI_EEPROM_COMMAND_READ_STATUS_REGISTER  0x05
#define SPI_EEPROM_COMMAND_WRITE_STATUS_REGISTER  0x01
#define SPI_EEPROM_COMMAND_READ  0x03
#define SPI_EEPROM_COMMAND_WRITE 0x02
#define SPI_EEPROM_COMMAND_CLEAR_ERROR_FLAGS 0x30
#define SPI_EEPROM_COMMAND_ERASE_BLOCK 0xD8
#define SPI_EEPROM_COMMAND_ERASE_SECTOR 0x20
#define SPI_EEPROM_COMMAND_ERASE_CHIP 0xC7

//macro to set SPI to MASTER; 8BIT; FSYNC Idle => xCS high
#define SPI_MASTER_8BIT_CSHI   PERIP(SPI0_CONFIG) = \
        SPI_CF_MASTER | SPI_CF_DLEN8 | SPI_CF_FSIDLE1

//macro to set SPI to MASTER; 8BIT; FSYNC not Idle => xCS low
#define SPI_MASTER_8BIT_CSLO   PERIP(SPI0_CONFIG) = \
        SPI_CF_MASTER | SPI_CF_DLEN8 | SPI_CF_FSIDLE0

//macro to set SPI to MASTER; 16BIT; FSYNC not Idle => xCS low
#define SPI_MASTER_16BIT_CSLO  PERIP(SPI0_CONFIG) = \
        SPI_CF_MASTER | SPI_CF_DLEN16 | SPI_CF_FSIDLE0

void SingleCycleCommand(u_int16 cmd){
  SPI_MASTER_8BIT_CSHI;
  SPI_MASTER_8BIT_CSLO;
  SpiSendReceive(cmd);
  SPI_MASTER_8BIT_CSHI;
}

/// Wait for not_busy (status[0] = 0) and return status
u_int16 SpiWaitStatus(void) {
  u_int16 status;
  SPI_MASTER_8BIT_CSHI;
  SPI_MASTER_8BIT_CSLO;
  SpiSendReceive(SPI_EEPROM_COMMAND_READ_STATUS_REGISTER);
  do {
    status = SpiSendReceive(0);
    if (PERIP(USB_STATUS) & USB_STF_BUS_RESET){
      USBHandler();
      SPI_MASTER_8BIT_CSHI;
      return -1; /* USB HAS BEEN RESET */
    }
  } while (status & 0x01);
    ; //Wait until chip is ready or return -1 if USB bus reset

  SPI_MASTER_8BIT_CSHI;

  return status;
}


void EeUnprotect(){
  SingleCycleCommand(SPI_EEPROM_COMMAND_WRITE_ENABLE);
  SPI_MASTER_8BIT_CSLO;
  SpiSendReceive(SPI_EEPROM_COMMAND_WRITE_STATUS_REGISTER);
  SpiSendReceive(0x02); //Sector Protections Off
  SPI_MASTER_8BIT_CSHI;
  SpiWaitStatus();
  SingleCycleCommand(SPI_EEPROM_COMMAND_WRITE_ENABLE);
}

void EePutReadBlockAddress(register u_int16 blockn){
  SPI_MASTER_8BIT_CSLO;
  SpiSendReceive(SPI_EEPROM_COMMAND_READ);
  SpiSendReceive(blockn>>7);            // Address[23:16] = blockn[14:7]
  SpiSendReceive((blockn<<1)&0xff);     // Address[15:8]  = blockn[6:0]0
  SpiSendReceive(0);                    // Address[7:0]   = 00000000
  SPI_MASTER_16BIT_CSLO;
}

// Check is a 4K block completely blank
u_int16 EeIsBlockErased(u_int16 blockn){
  SpiWaitStatus();
  EePutReadBlockAddress(blockn);
  {
    register u_int16 n;
    for (n=0; n<2048; n++){
      if (SpiSendReceive(0) != 0xffff) {
	SPI_MASTER_8BIT_CSHI;
	return 0;
      }
    }
    SPI_MASTER_8BIT_CSHI;
    return 1;
  }
}

s_int16 EeProgram4K(u_int16 blockn, __y u_int16 *dptr){

  PERIP(USB_EP_ST3) |= (0x0001); //Force NAK on EP3 (perhaps not needed?)

  do__not__puthex(blockn);
  do__not__puts("= write 4K");

  if (!EeIsBlockErased(blockn)){ //don't erase if not needed
    // Erase 4K sector
    SingleCycleCommand(SPI_EEPROM_COMMAND_WRITE_ENABLE);
    SingleCycleCommand(SPI_EEPROM_COMMAND_CLEAR_ERROR_FLAGS);
    EeUnprotect();
    SPI_MASTER_8BIT_CSLO;
    SpiSendReceive(SPI_EEPROM_COMMAND_ERASE_SECTOR);
    SpiSendReceive(blockn>>7);            // Address[23:16] = blockn[14:7]
    SpiSendReceive((blockn<<1)&0xff);     // Address[15:8]  = blockn[6:0]0
    SpiSendReceive(0);                    // Address[7:0]   = 00000000
    SPI_MASTER_8BIT_CSHI;
  }

  if (SpiWaitStatus() == -1) return -1; /* USB HAS BEEN RESET */
  // Write 8 512-byte sectors
  {
    u_int16 i;
    for (i=0; i<8; i++){

      // Put first page (256 bytes) of sector.
      EeUnprotect();
      SPI_MASTER_8BIT_CSLO;
      SpiSendReceive(SPI_EEPROM_COMMAND_WRITE);
      SpiSendReceive(blockn>>7);            // Address[23:16] = blockn[14:7]
      SpiSendReceive((blockn<<1)&0xff);     // Address[15:8]  = blockn[6:0]0
      SpiSendReceive(0);                    // Address[7:0]   = 00000000
      SPI_MASTER_16BIT_CSLO;
      {
	u_int16 n;
	for (n=0; n<128; n++){
#if USE_INVERTED_DISK_DATA
	  SpiSendReceive(~(*dptr++));
#else
	  SpiSendReceive((*dptr++));
#endif
	}
      }
      SPI_MASTER_8BIT_CSHI;
      if (SpiWaitStatus() == -1) return -1; /* USB HAS BEEN RESET */

      // Put second page (256 bytes) of sector.
      EeUnprotect();
      SPI_MASTER_8BIT_CSLO;
      SpiSendReceive(SPI_EEPROM_COMMAND_WRITE);
      SpiSendReceive(blockn>>7);            // Address[23:16] = blockn[14:7]
      SpiSendReceive(((blockn<<1)+1)&0xff);     // Address[15:8]  = blockn[6:0]1
      SpiSendReceive(0);                    // Address[7:0]   = 00000000
      SPI_MASTER_16BIT_CSLO;
      {
	u_int16 n;
	for (n=0; n<128; n++){
#if USE_INVERTED_DISK_DATA
	  SpiSendReceive(~(*dptr++));
#else
	  SpiSendReceive((*dptr++));
#endif
	}
      }
      SPI_MASTER_8BIT_CSHI;
      if (SpiWaitStatus() == -1) return -1; /* USB HAS BEEN RESET */
      blockn++;
    }
  }
  do__not__puts("written");

  PERIP(USB_EP_ST3) &= ~(0x0001); //Un-Force NAK on EP3
  return 0;

}



// Block Read for SPI EEPROMS with 24-bit address e.g. up to 16MB
u_int16 EeReadBlock(u_int16 blockn, u_int16 *dptr) {
  SpiWaitStatus();

  EePutReadBlockAddress(blockn);
  {
    int n;
    for (n=0; n<256; n++){
#if USE_INVERTED_DISK_DATA
      *dptr++ = ~SpiSendReceive(0);
#else
      *dptr++ = SpiSendReceive(0);
#endif
    }
  }
  SPI_MASTER_8BIT_CSHI;
  return 0;
}

// Returns 1 if block differs from data, 0 if block is the same
u_int16 EeCompareBlock(u_int16 blockn, u_int16 *dptr) {
  SpiWaitStatus();

  EePutReadBlockAddress(blockn);
  {
    int n;
    for (n=0; n<256; n++){

#if USE_INVERTED_DISK_DATA
      if ((*dptr++) != (~SpiSendReceive(0)))
#else
      if ((*dptr++) != (SpiSendReceive(0)))
#endif
	{
	  SPI_MASTER_8BIT_CSHI;
	  return 1;
	}
    }
  }
  SPI_MASTER_8BIT_CSHI;
  return 0;
}

u_int16 EeRead4KSectorYToWorkspace(u_int16 blockn) {
  register __y u_int16 *dptr;
  dptr = WORKSPACE;
  SpiWaitStatus();
  blockn &= 0xfff8; //always point to first block of 4k sector


  EePutReadBlockAddress(blockn);
  {
    int n;
    for (n=0; n<2048; n++){
#if USE_INVERTED_DISK_DATA
      *dptr++ = ~SpiSendReceive(0);
#else
      *dptr++ = SpiSendReceive(0);
#endif
    }
  }
  SPI_MASTER_8BIT_CSHI;
  return 0;
}

void InitSpi(u_int16 clockDivider){
  SPI_MASTER_8BIT_CSHI;
  PERIP(SPI0_FSYNC) = 0;
  PERIP(SPI0_CLKCONFIG) = SPI_CC_CLKDIV * (clockDivider-1);
  PERIP(GPIO1_MODE) |= 0x1f; /* enable SPI pins */
}


__y u_int16 *FindCachedBlock(u_int16 blockNumber){
  register int i;
  lastFoundBlock = -1;
  for (i=0; i<CACHE_BLOCKS; i++){
    if ((blockPresent & 1/*was L*/<<i) && (blockAddress[i] == blockNumber)){
      lastFoundBlock = i;
      return mallocAreaY+256*i;
    }
  }
  return NULL;
}


u_int16 WriteContinuous4K(){
  u_int16 i,k;
  for (i=0; i<CACHE_BLOCKS-7; i++){ //for all cached blocks...
    if ((blockPresent & 1/*was L*/<<i) && ((blockAddress[i] & 0x0007) == 0)){
      //cached block i contains first 512 of 4K
      for (k=1; k<8; k++){
	if (blockAddress[i+k] != blockAddress[i]+k) goto ohi;
      }
      do__not__puthex(blockAddress[i]); do__not__puts(" starts continuous 4K ");

      if (-1 != EeProgram4K (blockAddress[i], mallocAreaY+256*i)){
	for (k=0; k<8; k++){
	  blockPresent &= ~(1/*was L*/<<(i+k));
	}
      } else {
	return 0; /* USB HAS BEEN RESET */
      }
      return 1;
    }
  ohi:
    {}
  }
  return 0;
}

__y u_int16 *GetEmptyBlock(u_int16 blockNumber){
  register int i;
  for (i=0; i<CACHE_BLOCKS; i++){
    if (!(blockPresent & 1/*was L*/<<i)) {
      blockPresent |= 1/*was L*/<<i;
      blockAddress[i] = blockNumber;
      return mallocAreaY+256*i;
    }
  }
  //do__not__puts("cannot allocate empty block");
  return NULL;
}



struct FsMapper *FsMapSpiFlashCreate(struct FsPhysical *physical,
				u_int16 cacheSize);
s_int16 FsMapSpiFlashRead(struct FsMapper *map, u_int32 firstBlock,
		     u_int16 blocks, u_int16 *data);
s_int16 FsMapSpiFlashWrite(struct FsMapper *map, u_int32 firstBlock,
		      u_int16 blocks, u_int16 *data);
s_int16 FsMapSpiFlashFlush(struct FsMapper *map, u_int16 hard);


const struct FsMapper spiFlashMapper = {
    0x010c, /*version*/
    256,    /*blocksize*/
    LOGICAL_DISK_BLOCKS,      /*blocks*/
    0,      /*cacheBlocks*/
    FsMapSpiFlashCreate,
    FsMapFlNullOk,//RamMapperDelete,
    FsMapSpiFlashRead,
    FsMapSpiFlashWrite,
    NULL,//FsMapFlNullOk,//RamMapperFree,
    FsMapSpiFlashFlush,//RamMapperFlush,
    NULL /* no physical */
};
struct FsMapper *FsMapSpiFlashCreate(struct FsPhysical *physical,
				     u_int16 cacheSize) {

  do__not__puts("CREATE");
  InitSpi(SPI_CLOCK_DIVIDER);
  blockPresent = 0;
  shouldFlush = 0;
  return &spiFlashMapper;
}



s_int16 FsMapSpiFlashRead(struct FsMapper *map, u_int32 firstBlock,
			  u_int16 blocks, u_int16 *data) {
  register s_int16 bl = 0;

  if (shouldFlush) return 0;
  firstBlock += RESERVED_BLOCKS;

  while (bl < blocks) {
    __y u_int16 *source = FindCachedBlock(firstBlock);
#if 0
    do__not__puthex(firstBlock);
    do__not__puthex((u_int16)source);
    do__not__puts("=rd_lba, addr");
#endif
    if (source) {
      memcpyYX(data, source, 256);
    } else {
      EeReadBlock(firstBlock, data);
      //memset(data, 0, 256);
    }
    data += 256;
    firstBlock++;
    bl++;
  }
  return bl;
}


s_int16 FsMapSpiFlashWrite(struct FsMapper *map, u_int32 firstBlock,
			   u_int16 blocks, u_int16 *data) {
  s_int16 bl = 0;

  firstBlock += RESERVED_BLOCKS;

  if (shouldFlush){
    do__not__puts("flush-reject");
    return 0; // don't accept write while flushing
  }
  while (bl < blocks) {
    // Is the block to be written different than data already in EEPROM?
    if (EeCompareBlock(firstBlock, data)){
      __y u_int16 *target = FindCachedBlock(firstBlock);
      if (target) {
	do__not__puthex(firstBlock);
	do__not__puthex((u_int16)target);
	do__not__puts("=rewrite_lba");
      } else {
	target = GetEmptyBlock(firstBlock);
	do__not__puthex(firstBlock);
	do__not__puts("=write_lba");
      }
      if (!target){ //cache is full
	//must do a cache flush to get cache space
	FsMapSpiFlashFlush(NULL,1);
	target = GetEmptyBlock(firstBlock);
      }
      if (target){
	memcpyXY(target, data, 256);
	PrintCache();
      }else{
	puts("FATAL ERROR: NO CACHE SPACE. THIS NEVER HAPPENS.");
	while(1)
	  ;
      }
      WriteContinuous4K();
    } else {
      do__not__puthex(firstBlock);
      do__not__puts("=lba; Redundant write skipped");
    }

    if (PERIP(USB_STATUS) & USB_STF_BUS_RESET){
      do__not__puts("USB: Reset");
    };
    data += 256;
    firstBlock++;
    bl++;
  }
  return bl;
}


s_int16 FsMapSpiFlashFlush(struct FsMapper *map, u_int16 hard){
  u_int16 i,j,lba;
  u_int16 __y *dptr;
  u_int16 newBlockPresent;

  do__not__puts("FLUSH");
  PrintCache();

  if (shouldFlush>1) return 0;
  shouldFlush = 2; //flushing

  for (i=0; i<CACHE_BLOCKS; i++){
    if (blockPresent & (1/*was L*/<<i)){
      do__not__puthex(i);do__not__puthex(blockAddress[i]);do__not__puts("slot, lba  is dirty");
      lba = blockAddress[i] & 0xfff8;
      EeRead4KSectorYToWorkspace (lba);
      newBlockPresent = blockPresent;
      for (j=0; j<8; j++) {
	do__not__puthex (lba+j);
	if (dptr = FindCachedBlock(lba+j)){
	  memcpyYY (WORKSPACE+(256*j), dptr, 256);
	  newBlockPresent &= ~(1/*was L*/<<lastFoundBlock);

	  do__not__puts("from cache");
	} else {
	  do__not__puts("from disk");
	}
      }
      if (-1 != EeProgram4K (lba, WORKSPACE)){
	blockPresent = newBlockPresent;
	shouldFlush = 0;
      } else {
	shouldFlush = 1;
	return 0; /* USB HAS BEEN RESET */
      }
    }
  }
  shouldFlush = 0;
  return 0;

}


auto void MyMassStorage(void) {
  register __b0 int usbMode = 0;
  do__not__puts("MyMassStorage");

  voltages[voltCoreUSB] = 31; //30:ok
  voltages[voltIoUSB] = 31; // set maximum IO voltage (about 3.6V)
  do__not__puthex (voltages[voltCoreUSB]);
  do__not__puts("=USB Core Voltage"); //default:27
  PowerSetVoltages(&voltages[voltCoreUSB]);
  BusyWait10();
  LoadCheck(NULL, 1); /* Set 48 MHz Clock */
  SetRate(44100U);

  SetHookFunction((u_int16)InitUSBDescriptors, MyInitUSBDescriptors);
  do__not__puts("before usb init");
  InitUSB(USB_MASS_STORAGE);
  do__not__puts("after usb init");

  while (USBIsAttached()) {
    USBHandler();
    if (shouldFlush){
	FsMapSpiFlashFlush(NULL,1);
    }
    if (USBWantsSuspend()) {
      if (USBIsDetached()) {
	break;
      }
    }
    if (usbMode == 1) {
      if (AudioBufFill() < 32) {
	memset(tmpBuf, 0, sizeof(tmpBuf));
	AudioOutputSamples(tmpBuf, sizeof(tmpBuf)/2);
      }
      Sleep();
    }
  }
  hwSampleRate = 1;
  PERIP(SCI_STATUS) &= ~SCISTF_USB_PULLUP_ENA;
  PERIP(USB_CONFIG) = 0x8000U;
  map->Flush(map, 1);
  PowerSetVoltages(&voltages[voltCorePlayer]);
}



// FAT12 binary patch
auto u_int16 Fat12OpenFile(register __c0 u_int16 n);

void main(void) {

  do__not__puts("Hello.");

  InitAudio();

  PERIP(INT_ENABLEL) = INTF_RX | INTF_TIM0;
  PERIP(INT_ENABLEH) = INTF_DAC;

  PERIP(SCI_STATUS) &= ~SCISTF_USB_PULLUP_ENA;
  PERIP(USB_CONFIG) = 0x8000U;

  PERIP(GPIO1_ODATA) |=  LED1|LED2;
  PERIP(GPIO1_DDR)   |=  LED1|LED2;
  PERIP(GPIO1_MODE)  &= ~(LED1|LED2);

  player.volumeOffset = -24;
  player.pauseOn = 0;

  keyOld = KEY_POWER;
  keyOldTime = -32767;

  SetHookFunction((u_int16)OpenFile, Fat12OpenFile);
  SetHookFunction((u_int16)IdleHook, NullHook); //Disable keyboard scanning


  // Use our SPI flash mapper as logical disk
  map = FsMapSpiFlashCreate(NULL, 0);
  player.volume = 0;
  PlayerVolume();

  // Use custom main loop to take over total control of chip
  while (1) {
    // When USB is attached, go to mass storage handler
    if (USBIsAttached()) {
      do__not__puts("MassStorage");
      MyMassStorage();
      do__not__puts("From MassStorage");
    }

    // Try to use a FAT filesystem on logical disk
    if (InitFileSystem() == 0) {
      minifatInfo.supportedSuffixes = defSupportedFiles;
      
      // Look for playable files
      player.totalFiles = OpenFile(0xffffU);
      do__not__puthex(player.totalFiles);
      do__not__puts("");
      if (player.totalFiles == 0) {
	goto noFSnorFiles;
      }

      // Playable file(s) found. Play.
      player.nextStep = 1;
      player.nextFile = 0;
      while (1) {
	if (player.randomOn) {
	  register s_int16 nxt;
	retoss:
	  nxt = rand() % player.totalFiles;
	  if (nxt == player.currentFile && player.totalFiles > 1)
	    goto retoss;
	  player.currentFile = nxt;
	} else {
	  player.currentFile = player.nextFile;
	}
	if (player.currentFile < 0)
	  player.currentFile += player.totalFiles;
	if (player.currentFile >= player.totalFiles)
	  player.currentFile -= player.totalFiles;
	player.nextFile = player.currentFile + 1;
	if (OpenFile(player.currentFile) < 0) {
	  player.ffCount = 0;
	  cs.cancel = 0;
	  cs.goTo = -1;
	  cs.fileSize = cs.fileLeft = minifatInfo.fileSize;
	  cs.fastForward = 1;
	  {
	    register s_int16 oldStep = player.nextStep;
	    register s_int16 ret;

      do__not__puts("Current playing file");
      do__not__puthex(player.currentFile);
      do__not__puts("");
	    ret = PLAYFILE(); // Decode and Play.
      do__not__puts("Player return value");
      do__not__puthex(ret);
      do__not__puts("");
	    // See separate examples about keyboard handling.

	    if (ret == ceFormatNotFound)
	      player.nextFile = player.currentFile + player.nextStep;
	    if (ret == ceOk && oldStep == -1)
	      player.nextStep = 1;
	  }
	} else {
	  player.nextFile = 0;
	}

	// If USB is attached, return to main loop
	if (USBIsAttached()) {
	  break;
	}
      }
    } else {
    noFSnorFiles:
      LoadCheck(&cs, 32);
      memset(tmpBuf, 0, sizeof(tmpBuf)); /* silence */
      AudioOutputSamples(tmpBuf, sizeof(tmpBuf)/2); /* silence */
    }
  }
}
