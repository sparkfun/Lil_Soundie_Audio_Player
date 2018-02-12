/**
   \file codec.h Codec interfaces.
*/

#ifndef CODEC_H
#define CODEC_H

#include <vstypes.h>
#ifndef ASM
#if 0
#include "matrixer.h"
#else
#define MAX_SOURCE_CHANNELS 6
#ifndef __VSDSP__
/** Channel positions */
enum ChannelMatrix {
  cmUnknown,
  cmLeft,
  cmCenter,
  cmRight,
  cmRearRight,
  cmRearCenter,
  cmRearLeft,
  cmLFE
};
#endif/*!__VSDSP__*/
#endif
#endif /* !ASM */

/**
   Current version number. 8 MSBs contain version number, 8 LSBs size of
   the structure in words.

   Version number history:
   <OL>
	<LI>0x0105 First version
   </OL>
*/
#define CODEC_VERSION 0x0105

/**
   Current version number. 8 MSBs contain version number, 8 LSBs size of
   the structure in words.

   Version number history:
   <OL>
	<LI>0x0125 First version
   </OL>
*/
#define FS_CODEC_SERVICES_VERSION 0x0125


#define FS_CODSER_COMMENT_END_OF_LINE 0x4000U
#define FS_CODSER_COMMENT_END_OF_COMMENTS ((u_int16)(0x8000U))

#ifndef ASM

/**
   Codec Services structure. The caller must provide values for
   elements \a Read, \a Seek, \a Tell and \a fileSize.
   The caller may also set \a move and \a cancel for control
   operations. Note that the prototypes for \a Read, \a Seek and \a Tell
   are on purpose the same as they are for stdio.h functions
   \a fread(), \a fseek() and \a ftell().
 */
struct CodecServices {
  /** Version number. 8 MSBs contain version number, 8 LSBs size of
      the structure in words. */
  u_int16 version;
  /** Read data from a file. If \a firstOdd is set, only the LSB of the
      first word is filled. If the last word is not completely filled
      (either \a firstOdd set or \a bytes is odd, but not both),
      only the MSB is changed. */
  u_int16 (*Read)(struct CodecServices *cs, u_int16 *ptr,
		  u_int16 firstOdd, u_int16 bytes);
  /** Skip data in a file. This should also be supported for streams. */
  u_int32 (*Skip)(struct CodecServices *cs, u_int32 bytes);
  /** Seek in a file. \a offset and \a whence are equivalent with their
      fseek() counterparts.
      If \a Seek is NULL, the input is a stream and cannot be seeked. */
  s_int16 (*Seek)(struct CodecServices *cs, s_int32 offset, s_int16 whence);
  /** Tell location in a file (in bytes). */
  s_int32 (*Tell)(struct CodecServices *cs);
  /** Output to audio file.
      \a data is a pointer to the data. There are \a n \a chan channel samples
      (Example: \a n = 32, \a cs->chan = 2, there are a total of 64 samples in
      \a data). */
  s_int16 (*Output)(struct CodecServices *cs, s_int16 *data, s_int16 n);
  /** Offers comments fields one character at the time. Special code
      0x4000U is reserved for end-of-line. 0xC000U means end of comments. */
  void (*Comment)(struct CodecServices *cs, u_int16 c);
  /** Spectrum analyzer hook. */
  void (*Spectrum)(struct CodecServices *cs, s_int16 __y *data, s_int16 n,
		   s_int16 ch);
  /** File size in bytes. If set to 0xFFFFFFFFU, then the file is a
      stream and the file size is not known. */
  u_int32 fileSize;
  /** Bytes left in a file. If set to 0xFFFFFFFFU, then the file is a
      stream and never ends. */
  u_int32 fileLeft;
  /** Point to move to in an audio file in seconds. When set to
      anything other than 0xFFFFU, the codec has the responsibility
      to jump to that point.
      The codec has the freedom of deciding for itself the actual
      landing point in the file. When the codec has reached its
      destination it will clear this variable, and the actual position
      in the file can be read from \a playTimeSeconds. If the codec
      cannot jump to a given point (e.g. the file is a stream and the
      jump point would require jumping backwards), goTo is silently
      cleared to 0xFFFFU.
  */
  u_int16 goTo;
  /** Request codec to cancel playing current file.
      To request cancellation, set this to a non-zero positive value. When the
      codec has finished cancelling, it will clear this value and
      return ceCancelled.
  */
  s_int16 cancel;
  /** Playback time from beginning of file in seconds. Updated by the codec.
      If set to -1, the codec doesn't know where it is in the file. */
  s_int32 playTimeSeconds;
  /** Samples played since last full second. Updated by the codec. If set to
      -1, the codec doesn't know where it is in the file. */
  s_int32 playTimeSamples;
  /** Total playback time in seconds. Updated by the codec. May be a
      changing estimate if an exact figure isn't available. 0xFFFFFFFFU
      if there is no information available. */
  u_int32 playTimeTotal;
  /** Sample rate. Updated by the codec. 0 if unknown. */
  u_int32 sampleRate;
  /** Number of channels. Updated by the codec. */
  u_int16 channels;
  /** Channel matrix. Updated by the codec. */
  enum ChannelMatrix matrix[MAX_SOURCE_CHANNELS];
  /** Average bitrate. Updated by the codec. May change during playback. */
  u_int32 avgBitRate;
  /** Current bitrate. Updated by the codec. May change during playback. */
  u_int32 currBitRate;
  /** Peak bitrate of the file. Updated by the codec.
      May change during playback. */
  u_int32 peakBitRate;
  /** Volume gain recommended by codec in 1/2 dB steps. Updated by the codec.*/
  s_int16 gain;
  /** Request codec to fast forward current file. If set, playback is
      requested at fastForward times normal playback speed.
      To stop fast forwarding, set value to 1 or 0. */
  u_int16 fastForward;
};





/** Codec error codes. */
enum CodecError {
  ceFastForward = -1,	/**< Fast forwarded through file end */
  ceOk = 0,		/**< No errors. */
  ceFormatNotFound,	/**< Data file was not in known format for the codec */
  ceFormatNotSupported,	/**< Data file subformat is not supported. */
  ceUnexpectedFileEnd,	/**< Unexpectedly early end of file */
  ceCancelled,		/**< Playback cancel was requested */
  ceOtherError		/**< Unspecific error */
};


/** Standard Codec wrap-up structure */
struct Codec {
  /** Version number. 8 MSBs contain version number, 8 LSBs size of
      the structure in words. */
  u_int16 version;
  /** Create and allocate space for codec. */
  struct Codec *(*Create)(void);
  /** Decode file. Upon success or a negative number, Codec has succeeded.
      With a positive number, there has been an error. Upon return, an
      error string is also returned. */
  enum CodecError (*Decode)(struct Codec *cod, struct CodecServices *cs,
			    const char **errorString);
  /** Free all resources allocated for codec. */
  void (*Delete)(struct Codec *cod);
  /** A pointer that the codec may or may not fill or use. */
  struct CodecServices *cs;
};

#endif /* !ASM */

#define CS_VERSION_OFFSET		 0
#define CS_READ_OFFSET			 1
#define CS_SKIP_OFFSET			 2
#define CS_SEEK_OFFSET			 3
#define CS_TELL_OFFSET			 4
#define CS_OUTPUT_OFFSET		 5
#define CS_COMMENT_OFFSET		 6
#define CS_SPECTRUM_OFFSET		 7
#define CS_FILE_SIZE_OFFSET		 8
#define CS_FILE_LEFT_OFFSET		10
#define CS_GO_TO_OFFSET			12
#define CS_CANCEL_OFFSET		13
#define CS_PLAY_TIME_SECONDS_OFFSET	14	
#define CS_PLAY_TIME_SAMPLES_OFFSET	16	
#define CS_PLAY_TIME_TOTAL_OFFSET	18	
#define CS_SAMPLE_RATE_OFFSET		20
#define CS_CHANNELS_OFFSET		22
#define CS_MATRIX_OFFSET		23
#define CS_AVG_BIT_RATE_OFFSET		29
#define CS_CURR_BIT_RATE_OFFSET		31
#define CS_PEAK_BIT_RATE_OFFSET		33
#define CS_GAIN_OFFSET			35
#define CS_FAST_FORWARD_OFFSET		36

#define CODEC_VERSION_OFFSET		0
#define CODEC_CREATE_OFFSET		1
#define CODEC_DECODE_OFFSET		2
#define CODEC_DELETE_OFFSET		3
#define CODEC_CS_OFFSET			4

#endif /* !CODEC */
