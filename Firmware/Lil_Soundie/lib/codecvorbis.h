/**
   \file codecvorbis.h Vorbis 1.0 Codec. The Vorbis Codec supports
	windows upto 4096 samples (8192 sample windows are NOT
	supported). Also random access is not supported.
*/

#ifndef CODEC_VORBIS_H
#define CODEC_VORBIS_H

#include <vstypes.h>
#include <codec.h>

#define USE_COMMENTS

#ifndef ASM
/**
   Create and allocate space for codec.

   \return A Vorbis Codec structure.
*/
struct Codec *CodVorbisCreate(void);

/**
   Free all resources allocated for codec.

   \param cod A Vorbis Codec structure.
*/
void CodVorbisDelete(struct Codec *cod);

/**
   Decode file. Upon success or a negative number, Codec has succeeded.
   With a positive number, there has been an error. Upon return, an
   error string is also returned.

   \param cod A Vorbis Codec structure.
   \param cs User-supplied codec services with appropriate fields
	filled. The fields that are not to be filled by the user
	should be zero initialized.
   \param errorString A pointer to a char pointer. The codec may
	return its error status here.

   \return Error code.
 */
enum CodecError CodVorbisDecode(struct Codec *cod, struct CodecServices *cs,
			     const char **errorString);

auto s_int16 CodVBlockSize(register __i0 struct Codec *c,
			   register __a0 s_int16 bType);

#endif /* !ASM */

#endif /* CODEC_VORBIS_H */
