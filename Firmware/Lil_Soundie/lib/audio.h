/**
   \file audio.h Routines related to audio.
*/
#ifndef __AUDIO_H__
#define __AUDIO_H__

#include "vs1000.h"

#define WITH_EARSPEAKER
#define DIRECT_VORBIS_BLOCKSIZE

#define USE_TIMER
#ifdef USE_TIMER
#define TIMER_TICKS 100 /* 100Hz */
#ifndef ASM
/** timeCount counts TIMER_TICKS regardless of clockX. */
extern __y volatile u_int32 timeCount;
/** ReadTimeCount Reads the timecount variable safely,
    no interrupt disable is needed.
 */
u_int32 ReadTimeCount(void);
#endif
#endif

#define DEFAULT_AUDIO_BUFFER_SAMPLES 2048
#define DAC_DEFAULT_SAMPLERATE 8000
#define DAC_DRIVER_ON_DELAY (DAC_DEFAULT_SAMPLERATE/10) /* 100ms In samples */

#define APPL_RESET	 0
#define APPL_AUDIO	 1
#define APPL_BITSTREAM	10


#ifdef ASM
	.import _audioPtr
#define AUDIOPTR_WR 0
#define AUDIOPTR_RD 1
#define AUDIOPTR_FORWARD_MODULO 2
#define AUDIOPTR_LEFTVOL 3
#define AUDIOPTR_RIGHTVOL 4
#define AUDIOPTR_UNDERFLOW 5
#else

/**
   applAddr A hook function to process samples before they are
                      put into the audio buffer.
   \param d    A pointer to pointer to interleaved stereo samples
   \param mode If APPL_AUDIO, samples available
   \param n    Number of stereo samples
   \return For APPL_AUDIO the number of output stereo samples, return n otherwise.
 */
extern s_int16 (*applAddr)(s_int16 register __i0 **d,
			   s_int16 register __a1 mode,
			   s_int16 register __a0 n);
/**
   Audio FIFO. The length of the area used for audio can change
   depending on the state of the earSpeaker setting. Earspeaker
   can not be active when long vorbis frames are used.
 */
extern __y s_int16 audioBuffer[2*DEFAULT_AUDIO_BUFFER_SAMPLES];
struct AUDIOPTR {
    __y s_int16 *wr;       /* 0: write pointer */
    __y s_int16 *rd;       /* 1: read pointer */
    u_int16 forwardModulo; /* 2: 0x8000 + size - 1 */
    s_int16 leftVol;       /* 3: left volume,  default     -32768 =  1.0 */
    s_int16 rightVol;      /* 4: right volume, differential 32767 = -1.0 */
    s_int16 underflow;     /* 5: set if underflow in dac interrupt */
};
/**
   Audio structure containing the audio FIFO read and write pointers,
   FIFO size, software volume settings, and the FIFO underflow flag.
   By negating the other volume field you can get mono differential
   drive from the DAC for connecting a speaker between LEFT and RIGHT
   outputs.
 */
extern __y struct AUDIOPTR audioPtr;

extern u_int16 earSpeakerReg; /**< Current EarSpeaker setting. */
extern __y u_int16 earSpeakerDisable; /**< Long vorbis frames disable EarSpeaker automatically. */
extern u_int16 volumeReg; /**< Left and right volume 0x00(loudest)..0xff (off). */
extern u_int16 bassReg;   /**< Bass and treble controls, not used by rom firmware. See VS10xx datasheets for details. */
extern __y u_int16 extClock4KHz; /**< Crystal/4000. Normally 3000 (for 12MHz). */
extern __y u_int16 clockX; /**< Current clock multiplier in 0.5x steps. Is used to program the PLL at the next SetRate(). */
extern u_int32 __y curFctl; /**< Current DAC adder value. */
extern __y u_int16 hwSampleRate; /**< Current samplerate. */
extern __y u_int16 uiTime; /**< Free-running counter for UI. */
extern __y u_int16 uiTrigger; /**< Is non-zero 16 times per second when audio is played. */
extern s_int16 __y timeToRemovePDown2; /**< Delay until analog drivers are enabled. */
extern u_int32 __y haltTime; /**< The number of cycles spent in HALT since last check. Used for automatic clock management. */
extern __y u_int16 uartByteSpeed; /**< UART speed in bps / 10. UART divider is automatically updated when PLL value is changed. */
extern u_int16 bassTrebleFreq; /**< bassTrebleFreq = hwSampleRate; must be always set in SetVolume() */


extern __y struct EARSPEAKER {
    u_int16 Freq;
    u_int16 Disable;
    u_int16 Setting;
    s_int16 Old;
    u_int16 longFrames;
} earSpeaker;


/** Initializes audio structures and configures the PLL.
 */
void InitAudio(void);
/** Internal low-level audio output routine that puts samples into the
    audio buffer. Applies audioPtr.leftVol and audioPtr.rightVol to
    the data. Setting audioPtr.rightVol to -audioPtr.leftVol
    after InitAudio() has been called activates differential
    output mode, where the phase of right channel is inverted
    compared to the left channel.
    This routine does not check the buffer fullness.
    \param s Pointer to interleaved stereo samples
    \param n The number of stereo samples
 */
auto void StereoCopy(register __i2 s_int16 *s, register __a0 u_int16 n);
/** Tells the fill state of audio buffer in stereo samples.
 */
s_int16 AudioBufFill(void); /* how many STEREO samples to play */
/** Tells how many stereo samples still fits into the audio buffer without
    waiting. Note: the buffer should never be completely filled because
    the same state means empty.
 */
s_int16 AudioBufFree(void); /* how many STEREO samples fit */

/** Hook: Sets new samplerate and/or new PLL setting. Default: RealSetRate.
    \param rate New samplerate.
 */
auto void SetRate(register __c1 u_int16 rate);
/** Sets new samplerate and/or new PLL setting (according to clockX variable).
    \param rate New samplerate.
 */
auto void RealSetRate(register __c1 u_int16 rate);
/** Hook: Sets the hardware volume according to volumeReg. Default: RealSetVolume
 */
auto void SetVolume(void);
/** Sets the hardware volume according to volumeReg.
 */
auto void RealSetVolume(void);
/** High-level audio output routine. If the samples do not fit into
    the audio buffer, this routine automatically waits for some room
    (calls Sleep()).
    \param p Pointer to interleaved stereo samples.
    \param samples The number of stereo samples.
*/
auto void AudioOutputSamples(s_int16 *p, s_int16 samples);

/** Calculates UART divider from clockX and uartByteSpeed.
    When SetRate() changes PLL settings, the uart divider
    is automatically changed.
    \return UART divider for the current clockX and uartByteSpeed.
 */
u_int16 UartDiv(void);

#endif/*!ASM*/


#endif/*__AUDIO_H__*/

