/**
   \file player.h Routines related to the default player implementation.
*/
#ifndef __PLAYER_H__
#define __PLAYER_H__

#include <vstypes.h>

#define LED1 4 //SI
#define LED2 8 //SO

#ifndef ASM
void putch(register __a0 short dat); /**< Waits for UART TX ready, then writes next byte. */

extern s_int16 tmpBuf[2*32];

/** Structure used by the default firmwares play loop. */
extern struct Player {
    s_int16 totalFiles;  /**< total number of matching files */
    s_int16 currentFile; /**< current playing file */
    s_int16 nextFile;    /**< next file to play, changed by key events like ke_next */
    s_int16 nextStep;    /**< skip direction, 1 = next, -1 = previous */
    s_int16 pauseOn;     /**< non-zero for pause mode */
    s_int16 randomOn;    /**< non-zero for random play mode */
    s_int16 volume;      /**< volume -24 to 180 in 0.5dB steps */
    s_int16 volumeOffset;/**< volume offset, set by replayGain, default -12 */
    u_int16 offDelay;    /**< pause timeout in 5 sec increments -> power off */
    u_int16 ffCount;     /**< fast forward/rewind counter */
    u_int16 maxClock;    /**< max clock used by player: 7=3.5x, 6=3.0x etc. Default = 7, depends on core voltage. */
} player;

/** Actions for keypresses. */
enum keyEvent {
  ke_null = 0,
  ke_previous,   /**< skip to previous song or start of song (if >= 5 seconds played) */
  ke_next,       /**< goto next song */
  ke_rewind,     /**< skip 5 seconds back */
  ke_forward,    /**< skip 5 seconds forward */
  ke_volumeUp,   /**< increase volume by 0.5dB */
  ke_volumeDown, /**< decrease volume by 0.5dB */
  ke_earSpeaker, /**< rotate earSpeaker setting: 0, 16000U, 38000U, 54000U */
  ke_earSpeakerToggle, /**< toggle earSpeaker setting: 0, 38000U */
  ke_randomToggle, /**< toggle random play mode */
  ke_randomToggleNewSong, /**< toggle random play mode, start new song immediately when activated */
  ke_pauseToggle, /**< toggle pause mode */
  ke_powerOff,    /**< power off the unit */
  ke_ff_faster,  /**< increase play speed (needs ke_ff_off as release event) */
  ke_ff_slower,  /**< decrease play speed (needs ke_ff_off as release event) */
  ke_ff_off,     /**< back to normal play speed */
  ke_volumeUp2,  /**< increase volume by 1.0dB */
  ke_volumeDown2,/**< decrease volume by 1.0dB */
};

struct KeyMapping {
  u_int16 key;         /**< bitmask for key combination */
  enum keyEvent event; /**< event to trigger when key detected */
};
extern const struct KeyMapping *currentKeyMap;
extern const struct KeyMapping sixKeyMap[];
extern const struct KeyMapping fiveKeyMap[];
extern const struct KeyMapping shiftFourKeyMap[];
extern const struct KeyMapping threeKeyMap[];

#define KEY_1 1 /* prev/vol down     -> vol down 1 / vol down */
#define KEY_2 2 /* random/earspeaker -> vol up 1   / vol up */
#define KEY_3 4 /* next/vol up       -> prev / rew */
#define KEY_4 8 /*                   -> next / ff */
#define KEY_5 16 /*                  -> earspeaker / random */
#define KEY_POWER 256 /* pause / power off/on */
#define KEY_RELEASED   0x4000
#define KEY_LONG_PRESS 0x8000
#define KEY_LONG_ONESHOT 0x8000

extern u_int16 keyOld;
extern s_int16 keyOldTime;
#define SHORT_LIMIT 16
#define OFF_LIMIT 32


extern __y u_int16 mallocAreaY[]; /* for ramdisk */
extern u_int16 mallocAreaX[];     /* for ramboot */

extern const u_int32 *supportedFiles;
extern const u_int32 defSupportedFiles[];
extern u_int16 keyCheck;

/*
  Function Prototypes
 */
/** Performs key scanning of 5 keys on GPIO0 and the power key.
   Processes the results to generate short and long key press events,
   then calls the KeyEventHandler with the appropriate key events.
   Short press is generated if press was shorter than 16 ui ticks (1 second).
   Long presses are generated when press lasts longer than 16 ui ticks.
   Long press generates events 16 times a second unless the KEY_LONG_ONESHOT
   is added.
   Any key combination containing power key requires longer press (2 seconds).

   KeyScan also cancels play and resets pause mode if USB is attached.
 */
void KeyScan(void);
/** Cleans unused sectors from FAT16/FAT32 disks.
    If tryBoot is non-zero, checks for VS1000_B.RUN. */
auto void CleanDisk(register __c1 u_int16 tryBoot);
/** Updates volume using player.volume and player.volumeOffset. */
void PlayerVolume(void);
/** Non-zero if USBN is low for at least two cycles in 20000 cycles.
    Note: this routine may be too slow in some applications, so you
    can check (PERIP(SCI_STATUS) & SCISTF_USB_DN) == 0 directly. */
auto u_int16 USBIsAttached(void);
/** Hook: Implements MassStorage. Default: RealMassStorage.
 */
auto void MassStorage(void);
/** Implements MassStorage.
 */
auto void RealMassStorage(void);
/** Hook: performs actions for key events. Default: RealKeyEventHandler.
 */
void KeyEventHandler(enum keyEvent event);
/** Performs actions for key events.
 */
void RealKeyEventHandler(enum keyEvent event);
/** Read GPIO0[0:7] values. Turns the bits to GPIO mode and restores
    the state afterwards.
 */
auto u_int16 ReadGPIO(void);
/** Hook: Activate low-power mode. Default: RealUSBSuspend.
    \param timeOut 0, or calls PowerOff() when timeout reached.
 */
void USBSuspend(u_int16 timeOut);
/** Activates low-power mode. If timeOut is non-zero will blink
    the power/pause LED.
    Wakes up if changes in USB pins or GPIO0[0:4] pins are detected.
    Restores USB voltages.
    \param timeOut 0, or calls PowerOff() when timeout reached.
 */
void RealUSBSuspend(u_int16 timeOut);
/** Scans the keys and generates key events for long and short presses.
    Calls the KeyEventHandler.
 */
void KeyScan(void);
/** The default idle hook function that scans keys 16 timer per second.
 */
void UserInterfaceIdleHook(void);
/** Implements codec service Read function. Uses the Read() hook function.
 */
u_int16 CsRead(struct CodecServices *cs, u_int16 *data,
	       u_int16 firstOdd, u_int16 bytes);
/** Implements codec service Seek function. Uses Seek() and Tell() hook functions.
 */
s_int16 CsSeek(struct CodecServices *cs, s_int32 offset, s_int16 whence);
/** Implements codec service Output function.
    Implements pause mode (player.pauseOn) by inserting zero samples,
    and low-power pause mode (USBSuspend()) between key presses.
    Updates audio sample rate.
    Converts mono input to stereo for AudioOutputSamples().
 */
s_int16 CsOutput(struct CodecServices *cs, s_int16 *data, s_int16 n);


/** Additions to VS1000D, contain zeros in VS1000B/VS1000C.
 */
/** Helper array to quickly reverse bits in a byte. */
extern __y u_int16 vs1000d_BitReverse[256];
/** Latin1-compatible font stored in Vertical format, LSb is the top pixel.
    3 words (6 bytes) per character, high 8 bits first, add one empty line
    as space between characters. Contains block characters and special
    symbols in codes 0..31 and 128..159. */
extern __y u_int16 vs1000d_Latin1[256*3];

#endif/*!ASM*/
#endif /*__PLAYER_H__*/
