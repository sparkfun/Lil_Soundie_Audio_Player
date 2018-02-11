/**
   \file codecminiwav.h MinimalWav Codec (size 730 words).
        At this moment the Wav Codec supports ulaw 8-bit and
	linear 8-bit and 16-bit stereo and mono files.
	Randon access for seekable supported for all audio formats.
*/

#ifndef CODEC_MINIWAV_H
#define CODEC_MINIWAV_H

#include <vstypes.h>
#include <codec.h>

/**
   Create and allocate space for codec.

   \return A Wav Codec structure.
*/
struct Codec *CodMiniWavCreate(void);

/**
   Free all resources allocated for codec.

   \param cod A Wav Codec structure.
*/
void CodMiniWavDelete(struct Codec *cod);

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
enum CodecError CodMiniWavDecode(struct Codec *cod, struct CodecServices *cs,
				 const char **errorString);


#endif
