/**
   \file codecmicrowav.h Really Minimal Wav Codec (size 296 words).
        At this moment the MicroWav Codec supports
	linear 8-bit and 16-bit stereo and mono files.
	Does not check for "RIFF", but checks for "WAVE".
	Does not support goTo nor fast play.
	Always assumes linear PCM.
	Number of channels must be 1 or 2 (does not check).
*/

#ifndef CODEC_MICROWAV_H
#define CODEC_MICROWAV_H

#include <vstypes.h>
#include <codec.h>

/**
   Create and allocate space for codec.

   \return A Wav Codec structure.
*/
struct Codec *CodMicroWavCreate(void);

/**
   Decode file. Upon success or a negative number, Codec has succeeded.
   With a positive number, there has been an error. Upon return, an
   error string is also returned.

   \param cod A Wav Codec structure.
   \param cs User-supplied codec services with appropriate fields
	filled. The fields that are not to be filled by the user
	should be zero initialized.
   \param errorString A pointer to a char pointer. The codec may
	return its error status here.

   \return Error code.
 */
enum CodecError CodMicroWavDecode(struct Codec *cod, struct CodecServices *cs,
				  const char **errorString);


#endif
