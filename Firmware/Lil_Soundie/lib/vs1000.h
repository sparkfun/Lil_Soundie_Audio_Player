/**
   \file vs1000.h VS1000 hardware register definitions and general functions.
*/
#ifndef __VS1000_H__
#define __VS1000_H__


/*
  Testit:
    + RAM:
    + ROM:
    + Muxtest: reg->mem is in scan, only mem->regs need to be tested.

  DAC test: sin, interpolated?
 */

#include <vstypes.h>

#define IROM_START 0x4000
#define IROM_SIZE  0x4a80
#define XROM_START 0x4000
#define XROM_SIZE  0x0c00
#define YROM_START 0x4000 /* 8k and 2k */
#define YROM_SIZE  0x2800

#define XRAM_START    0x0
#define XRAM_SIZE  0x3400
#define YRAM_START    0x0
#define YRAM_SIZE  0x4000
#define IRAM_START    0x0
#define IRAM_SIZE   0x800


//#define STACK_START 0x300 /* these are used by FPGA emulator/monitor */
//#define STACK_SIZE  0x100
//#define DEBUG_STACK 0xe0

#define DCT_START   0x1000
#define STACK_START 0x1800 /* these may not be final.. */
#define STACK_SIZE  0x200 /* 0x100 is not quite enough! */
#define DEBUG_STACK (STACK_START+STACK_SIZE-32)  /* 78..98.. need 18 + tmp */
#define OTHERS_START 0x0800 /* 2048 */
#define YPREV0_START 0x0000 /* 1024 */
#define YPREV1_START 0x0400 /* 1024 */
#define AUDIO_START  0x0000 /* 4096 */

#define INTV_GPIO1 10
#define INTV_GPIO0 9
#define INTV_REGU  8
#define INTV_TIM1  7
#define INTV_TIM0  6 
#define INTV_RX    5
#define INTV_TX    4
#define INTV_NFLSH 3
#define INTV_USB   2
#define INTV_SPI   1
#define INTV_DAC   0

#define INTF_GPIO1 (1<<INTV_GPIO1)
#define INTF_GPIO0 (1<<INTV_GPIO0)
#define INTF_REGU  (1<<INTV_REGU)
#define INTF_TIM1  (1<<INTV_TIM1)
#define INTF_TIM0  (1<<INTV_TIM0)
#define INTF_RX    (1<<INTV_RX)
#define INTF_TX    (1<<INTV_TX)
#define INTF_NFLSH (1<<INTV_NFLSH)
#define INTF_USB   (1<<INTV_USB)
#define INTF_SPI   (1<<INTV_SPI)
#define INTF_DAC   (1<<INTV_DAC)

#define INT_EN_NONE  0
#define INT_EN_GPIO1 INTV_GPIO1
#define INT_EN_GPIO0 INTV_GPIO0
#define INT_EN_REGU  INTV_REGU
#define INT_EN_TIM1  INTV_TIM1
#define INT_EN_TIM0  INTV_TIM0
#define INT_EN_RX    INTV_RX
#define INT_EN_TX    INTV_TX
#define INT_EN_NFLSH INTV_NFLSH
#define INT_EN_USB   INTV_USB
#define INT_EN_SPI   INTV_SPI
#define INT_EN_DAC   INTV_DAC


//#define SCI_BASE        0xC000
#define SCI_SYSTEM      0xC000
#define SCISYSF_CLKDIV 0x8000
#define SCISYSF_AVDD   (1<<10)
#define SCISYSF_IOVDD  (1<<5)
#define SCISYSF_CVDD   (1<<0)
#define SCI_STATUS      0xC001

#define SCISTF_SLOW_CLKMODE   (1<<15)
#define SCISTF_USB_DN_OUT     (1<<14)
#define SCISTF_USB_DP_OUT     (1<<13)
#define SCISTF_USB_DDR        (1<<12)

#define SCISTF_VCM_OVERLOAD   (1<<11) /* VCM in overload */
#define SCISTF_VCM_DISABLE    (1<<10) /* Disable VCM protection */

#define SCISTF_USB_DP         (1<<9) /* attached with  controllable pull-up */
#define SCISTF_USB_DN         (1<<8) /* attached with fixed 100k-1M pull-up */
#define SCISTF_USB_DIFF_ENA   (1<<7)
#define SCISTF_USB_PULLUP_ENA (1<<6) /* control 1.5kOhm DP pull-up */
#define SCISTF_REGU_POWERLOW  (1<<5)
#define SCISTF_REGU_POWERBUT  (1<<4)
#define SCISTF_ANADRV_PDOWN   (1<<3)
#define SCISTF_ANA_PDOWN      (1<<2)
#define SCISTF_REGU_CLOCK     (1<<1)
#define SCISTF_REGU_SHUTDOWN  (1<<0)

#define SCI_DEBUG       0xC002 /* for debug outputs */

#define GPIO0_MODE      0xC010 /*default='1' = PERIP*/
#define GPIO1_MODE      0xC011

#define DAC_VOL         0xC012

#define FREQCTLL        0xC013
#define FREQCTLH        0xC014

#define FCH_MUL0_B 4
#define FCH_MUL1_B 5
#define FCH_MUL2_B 6
#define FCH_MUL3_B 7
#define FCH_DIV_INCLK_B 8
#define FCH_FORCE_PLL_B 9
//#define FCH_AUTOMODE_B 10
#define FCH_VCO_OUT_ENA_B 11
#define FCH_PLL_SET_LOCK_B 12
#define FCH_PLL_LOCKED_B 13

#define FCH_MUL0 (1<<FCH_MUL0_B)
#define FCH_MUL1 (1<<FCH_MUL1_B)
#define FCH_MUL2 (1<<FCH_MUL2_B)
#define FCH_MUL3 (1<<FCH_MUL3_B)
#define FCH_DIV_INCLK (1<<FCH_DIV_INCLK_B)
#define FCH_FORCE_PLL (1<<FCH_FORCE_PLL_B)
//#define FCH_AUTOMODE (1<<FCH_AUTOMODE_B)
#define FCH_VCO_OUT_ENA (1<<FCH_VCO_OUT_ENA_B)
#define FCH_PLL_SET_LOCK (1<<FCH_PLL_SET_LOCK_B)
#define FCH_PLL_LOCKED (1<<FCH_PLL_LOCKED_B)


#define DAC_LEFT        0xC015
#define DAC_RIGHT       0xC016

#define WDOG_CONFIG     0xC020
#define WDOG_RESET      0xC021
#define WDOG_DUMMY      0xC022
#define WDOG_RESET_VAL  0x4ea9

#define UART_STATUS     0xC028
#define UART_DATA       0xC029
#define UART_DATAH      0xC02A
#define UART_DIV        0xC02B

#define UART_ST_RXORUN    (1<<3)
#define UART_ST_RXFULL    (1<<2)
#define UART_ST_TXFULL    (1<<1)
#define UART_ST_TXRUNNING (1<<0)


#define TIMER_CONFIG    0xC030
#define TIMER_ENABLE    0xC031
#define TIMER_T0L       0xC034
#define TIMER_T0H       0xC035
#define TIMER_T0CNTL    0xC036
#define TIMER_T0CNTH    0xC037
#define TIMER_T1L       0xC038
#define TIMER_T1H       0xC039
#define TIMER_T1CNTL    0xC03A
#define TIMER_T1CNTH    0xC03B

#define GPIO0_DDR       0xC040 /* GPIO0 15 bits */
#define GPIO0_ODATA     0xC041
#define GPIO0_IDATA     0xC042
#define GPIO0_INT_FALL  0xC043
#define GPIO0_INT_RISE  0xC044
#define GPIO0_INT_PEND  0xC045
#define GPIO0_SET_MASK  0xC046
#define GPIO0_CLEAR_MASK 0xC047
#define GPIO0_BIT_CONF  0xC048
#define GPIO0_BIT_ENG0  0xC049
#define GPIO0_BIT_ENG1  0xC04A

#define GPIO0_READY 0x0100
#define GPIO0_RD    0x0200
#define GPIO0_CS1   0x0400
#define GPIO0_WR    0x0800
#define GPIO0_CLE   0x1000
#define GPIO0_ALE   0x2000

#define GPIO1_DDR       0xC050 /* GPIO1 5 bits */
#define GPIO1_ODATA     0xC051
#define GPIO1_IDATA     0xC052
#define GPIO1_INT_FALL  0xC053
#define GPIO1_INT_RISE  0xC054
#define GPIO1_INT_PEND  0xC055
#define GPIO1_SET_MASK  0xC056
#define GPIO1_CLEAR_MASK 0xC057
#define GPIO1_BIT_CONF  0xC058
#define GPIO1_BIT_ENG0  0xC059
#define GPIO1_BIT_ENG1  0xC05A

#define SPI0_CONFIG     0xC068
#define SPI0_CLKCONFIG  0xC069
#define SPI0_STATUS     0xC06A
#define SPI0_DATA       0xC06B
#define SPI0_FSYNC      0xC06C

#define SPI_CF_SRESET     (1<<11)
#define SPI_CF_RXFIFOMODE (1<<10) /*1=interrupt when fifo full or CS deassert*/
#define SPI_CF_RXFIFO     (1<<9)  /*rx FIFO enable*/
#define SPI_CF_TXFIFO     (1<<8)  /*tx FIFO enable*/
#define SPI_CF_INTXCS  (0<<6)
#define SPI_CF_FALLXCS (2<<6)
#define SPI_CF_RISEXCS (3<<6)
#define SPI_CF_MASTER  (1<<5)
#define SPI_CF_SLAVE   (0<<5)
#define SPI_CF_DLEN    (1<<1)
#define SPI_CF_DLEN8   (7<<1)
#define SPI_CF_DLEN16 (15<<1)
#define SPI_CF_FSIDLE1 (1<<0)
#define SPI_CF_FSIDLE0 (0<<0)

//#define SPI_CC_CLKDIV  (1<<8)
//#define SPI_CC_CLKRPOL (1<<7)
//#define SPI_CC_CLKOPOL (1<<6)
//#define SPI_CC_CLKIPOL (1<<5)
//#define SPI_CC_CLKIDEL (1<<0)
#define SPI_CC_CLKDIV  (1<<2)

#define SPI_ST_BREAK     (1<<5)
#define SPI_ST_RXORUN    (1<<4)
#define SPI_ST_RXFULL    (1<<3)
#define SPI_ST_TXFULL    (1<<2)
#define SPI_ST_TXRUNNING (1<<1)
#define SPI_ST_TXURUN    (1<<0)


#define NFLSH_CTRL      0xC060
#define NFLSH_CF_LCD_CE_MODE (1<<8)
#define NFLSH_CF_INT_ENABLE (1<<7)
#define NFLSH_CF_NF_RESET   (1<<6)
#define NFLSH_CF_WAITSTATES (1<<0)
#define NFLSH_LPL       0xC061
#define NFLSH_CP_LPH    0xC062
#define NFLSH_DATA      0xC063
#define NFLSH_NFIF      0xC064
#define NFLSH_NB_BYTECNT  (8)
#define NFLSH_NF_BYTECNT  (1<<NFLSH_NB_BYTECNT)
#define NFLSH_NF_USE_DBUF (1<<7)
#define NFLSH_NF_POINTER  (1<<2)
#define NFLSH_NF_START    (1<<1)
#define NFLSH_NF_READ     (1<<0)
#define NFLSH_DSPIF     0xC065
#define NFLSH_DB_POINTER   (4)
#define NFLSH_DF_POINTER   (1<<NFLSH_DB_POINTER)
#define NFLSH_DF_ENA_DBUF  (1<<3)
#define NFLSH_DF_READ      (1<<2)
#define NFLSH_DF_ECC_CALC  (1<<1)
#define NFLSH_DF_ECC_RESET (1<<0)
#define NFLSH_ECC_CNT   0xC066


#define INT_ENABLE      0xC070
#define INT_ENABLEL     0xC070
#define INT_ENABLEH     0xC072
#define INT_ORIGIN      0xC074
#define INT_VECTOR      0xC076
#define INT_ENCOUNT     0xC077
#define INT_GLOB_DIS    0xC078
#define INT_GLOB_EN     0xC079

/*
 * Usb peripheral regs (vs1000)
 */
#define USB_BASE     0xC080U
/*
 * Usb memory buffer positions (at X-address)
 */
#define USB_RECV_MEM 0x2C00 /**< USB receive memory, ring buffer. */
#define USB_SEND_MEM 0x3000 /**< USB send memory */
#define PERIP_IN_X

#ifdef PERIP_IN_X
#define PERIP(x) USEX(x)
#else
#define PERIP(x) USEY(x)
#endif

#ifdef ASM

#ifdef PERIP_IN_X
#define STP stx
#define LDP ldx
#else
#define STP sty
#define LDP ldy
#endif
#define MR_FRACT 0x000

#else

/*
  Boot-related functions.
 */
/* SpiBoot() and SpiLoad() do not return!
   m24 = 0 for 16-bit SPI EEPROM address.
 */
void SpiBoot(register __a0 short clkConf, register __i2 short addr,
	     register __i0 short m24);
/* SpiBoot(SPI_CC_CLKDIV*7 | SPI_CC_CLKOPOL, 0, 0); */
void SpiLoad(register __i2 short startAddr, register __i0 short m24);
void SpiDelay(register __a0 u_int16 wait);
auto u_int16 SpiSendReceive(register __a0 u_int16 data);

void Restart(void);

/* These are called through IRAM vectors, by default FAT16/32. */
/** Hook: Called by Sleep() before halt state is entered. Default: UserInterfaceIdleHook().
*/
void IdleHook(void);
/**
  Hook: Initializes filesystem. Default: FatInitFileSystem().
  \return 0 for success.
*/
auto u_int16 InitFileSystem(void);
/**
  Hook: Opens a specified file. Default: FatOpenFile().
  \param fileNum the number of the file to open.
  \return -1 for success, the number of files otherwise.
*/
auto s_int16 OpenFile(register __c0 u_int16 fileNum);
/**
  Hook: Reads bytes from the currently opened file. Default: FatReadFile().
  \param buf Packed buffer to read bytes to.
  \param byteOff Packed byte offset of the first byte. Even = high part of word, odd = low part of word.
  \param byteSize The number of bytes to read.
  \return the number of bytes actually read.
  \example tmpBuf[0] = 0; FatReadFile(tmpBuf, 1, 1); reads one byte to the low part of tmpBuf[0].
*/
auto s_int16 ReadFile(register __i3 u_int16 *buf,
		      register __c1 s_int16 byteOff,
		      register __c0 s_int16 byteSize
		      /*<0 for little-endian target buffer order*/);
/**
  Hook: Change the read position of a file. Default: FatSeek().
  \param pos Byte position to find.
  \return the FAT sector that corresponds to the pos.
*/
u_int32 Seek(register __reg_a u_int32 pos); /**< Sets pos, returns old pos */
/**
  Hook: Return the current read position. Default: FatTell().
  \return Current read position.
*/
u_int32 Tell(void); /**< Gets pos */
/**
  Hook: Read a sector. Default: MapperReadDiskSector().
  \param buffer the buffer for the sector data.
  \param sector the sector to read.
*/
auto u_int16 ReadDiskSector(register __i0 u_int16 *buffer,
			    register __reg_a u_int32 sector);

/**
  Reads one sector through the mapper interface (map->Read()).
  \param buffer the buffer for the sector data.
  \param sector the sector to read.
*/
auto u_int16 MapperReadDiskSector(register __i0 u_int16 *buffer,
				  register __reg_a u_int32 sector);

/** Disable interrupts.
    Call Enable() an equal number of time to enable interrupts.
    Should only be used for critical code sections when other exclusion
    methods are not possible.
 */
void Disable(void);
/** Enable interrupts.
 */
void Enable(void);
/** Call the idle hook, then wait for the next interrupt (HALT mode).
 */
void Sleep(void);
/** IdleHook that does nothing.
 */
void NullHook(void);
/** Sets new IRAM Hook function.
    \param hook    The address of the Hook in IRAM
    \param newFunc The new hook function address.
    \return The old hook function address is returned.
 */
void *SetHookFunction(register __i0 u_int16 hook, register __a0 void *newFunc);

//void BootFromX(register __i0 u_int16 *start);
/** Reads and handles I, X, Y, and execute records from X memory.
    \bug: B0 is destroyed if the call returns. The function prototype
          with a return register set to b0 provides a workaround.
 */
register __b0 u_int16 BootFromX(register __i0 u_int16 *start);
//extern s_int16 __y const cos64NewTab[];
void SinTest(void); /* uses g_yprev0 for parameters */
void MemTests(register short __b0 muxTestResult);

/** enumerations for voltage array members. */
enum voltIdx {
    /* for VS1000B */
    voltCorePlayer = 0, /**< core voltage in player mode */
    voltIoPlayer,       /**< IO voltage in player mode */
    voltAnaPlayer,      /**< Analog voltage in player mode */
    voltCoreUSB,        /**< core voltage in USB mode */
    voltIoUSB,          /**< IO voltage in USB mode */
    voltAnaUSB,         /**< Analog voltage in USB mode */
    voltCoreSuspend,    /**< core voltage in suspend/low-power pause mode */
    voltIoSuspend,      /**< IO voltage in suspend/low-power pause mode */
    voltAnaSuspend,     /**< Analog voltage in suspend/low-power pause mode */
    voltCoreUser,       /**< core voltage in user-defined mode */
    voltIoUser,         /**< IO voltage in user-defined mode */
    voltAnaUser,        /**< Analog voltage in user-defined mode */
    voltEnd
};
extern u_int16 voltages[voltEnd];
/** Waits 120000 cycles, which is 10ms if clock is 1.0x.*/
void BusyWait10(void);
/** VS1000D: Waits 12000 cycles, which is 1ms if clock is 1.0x.
    In VS1000B/C is the same as BusyWait10(). */
void BusyWait1(void);
/** Sets voltages according to parameter values. */
void PowerSetVoltages(u_int16 volt[3]);
/** Hook: Turns power off. Default: RealPowerOff().
 */
void PowerOff(void);
/** Power off routine.
    Sets 1.0x mode (PLL off), 100Hz samplerate, disables interrupts,
    turns off analog drivers and LED's. Then waits until the power
    button is released. Then shuts off the regulators.
 */
void RealPowerOff(void);
/** Hook: play the currently open file. Default: RealPlayCurrentFile().
 */
u_int16 PlayCurrentFile(void);
/** Tries to play the current file.
    Turns on maximum player mode clock (by default 3.5x),
    and tries to decode the file with the Ogg Vorbis decoder.
    After decoding has ended feeds zeros to audio buffer to ensure
    earspeaker echo has decayed.
    If decoding returned with ceFormatNotFound, calls the UnsupportedFile hook.
    Before returning turns on maximum player mode clock to speed up
    locating the next file.
    See enum CodecError from codec.h for return values.
 */
u_int16 RealPlayCurrentFile(void);
/** Hook: Decreases or increases the system clock. Default: RealLoadCheck().
 */
void LoadCheck(struct CodecServices *cs, s_int16 n);
/** Decreases or increases the system clock. Also handles Replay Gain.
    If cs == NULL turns on maximum allowed clock (if n is zero,
    player.maxClock for player mode, otherwise 4.0x for USB mode).
    Otherwise audio underflows or too empty audio buffer (<256 stereo
    samples) cause an increase in clock, and too much idle time
    \param cs CodecServices struct pointer or NULL.
    \param n The number of samples or 0 or 1.
    \example LoadCheck(NULL, 0); will turn on maximum player-mode clock.
 */
void RealLoadCheck(struct CodecServices *cs, s_int16 n);
/** Hook: called when file is not Ogg Vorbis. Default: DefUnsupportedFile().
 */
u_int16 UnsupportedFile(struct CodecServices *cs);
/** Called when file is not Ogg Vorbis. Dummy function.
 */
u_int16 DefUnsupportedFile(struct CodecServices *cs);

extern u_int16 g_dctlo[2048];
extern __y u_int16 g_dcthi[2048];
extern s_int16 g_others[2048];
extern s_int16 g_yprev0[1024];
extern s_int16 g_yprev1[1024];

/** Creates a mapper for a RAM disk using mallocAreaY.
    \param physical is a pointer to mallocAreaY
    \param cacheSize not used, RAM disk is always 36 sectors.
    \return pointer to mapper structure
 */
struct FsMapper *FsMapRamCreate(struct FsPhysical *physical,
				u_int16 cacheSize);

void putch(register __a0 s_int16 ch); /**< raw polled UART send */
s_int16 getch(void);                  /**< raw polled UART receive */
void putword(register __a0 s_int16 ch); /**< raw polled UART send, high byte first */
s_int16 getword(void);                  /**< raw polled UART receive, high byte first */
#endif


#endif/*__VS1000_H__*/
