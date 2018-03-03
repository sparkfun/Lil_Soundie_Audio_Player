/*
   Copyright 2008-2014 VLSI Solution Oy. Absolutely no warranty.
*/

#include "system.h"

#include <stdio.h>  // Standard io
#include <stdlib.h> // VS_DSP Standard Library
#include <vs1000.h> // VS1000B register definitions
#include <vectors.h>  // VS1000B vectors (interrupts and services)
#include <minifat.h>  // Read Only Fat Filesystem
#include <mapper.h> // Logical Disk
#include <string.h> // memcpy etc
#include <player.h> // VS1000B default ROM player
#include <audio.h>  // DAC output
#include <codec.h>  // CODEC interface
#include <vsNand.h>
#include <mappertiny.h>
#include <usb.h>

#include <dev1000.h>

extern struct Codec *cod;
extern struct CodecServices cs;

#ifdef GAPLESS
struct Codec *CodVorbisCreate (void);
extern u_int16 ogg[];
//extern struct CodVOgg ogg;
extern s_int16 vFirstFrame;
auto s_int16 OggSeekHeader (register u_int32 pos);
#endif /*GAPLESS*/
#ifdef USE_WAV
#include <codecmicrowav.h>
#include <dev1000.h>
//extern u_int16 codecVorbis[];
//extern u_int16 ogg[];
//extern u_int16 mInt[];
extern struct CodecServices cs;
extern struct Codec *cod;
enum CodecError PlayWavOrOggFile (void)
{
  register enum CodecError ret = ceFormatNotFound;
  const char *eStr;
  LoadCheck (NULL, 0);  /* higher clock, but 4.0x not absolutely required */

#ifdef GAPLESS
  if (codecVorbis.audioBegins)
  {
    u_int16 data[2];
    Seek (codecVorbis.audioBegins);
    ReadFile (data, 0, 4);
    if (data[0] == 0x4f67 && data[1] == 0x6753)
    {
      /* 'OggS', assume the same parameters, skip Vorbis decoder init. */
      const char *eStr;
      register s_int16 res = OggSeekHeader (codecVorbis.audioBegins);
      ogg[1] /* .headerTypeFlag */  = 0;
      ogg[6] /* .streamSerialNumber */  = 0;
      ogg[7] /* .streamSerialNumber */  = 0;
      ogg[14] /* .cont */  = 0;
      // ogg.headerTypeFlag = 0;
      // ogg.cont = 0;
      vFirstFrame = 1;
      cs.playTimeSeconds = 0;
      cs.playTimeSamples = 0;
      // putch('+');
#ifdef HAS_PATCHCODVORBISDECODE
      /* available in latest dev1000.h */
      return PatchCodVorbisDecode (&codecVorbis, &cs, &eStr, 0);
#else
      /* TODO: call Decode() with silentchannelpatches */
      return CodVorbisDecode (&codecVorbis, &cs, &eStr);
#endif
    }
    else
    {
      /* 'OggS' Not found, must be a file with different parameters (or WAV) */
      // putch('-');
      Seek (0);
    }
  }
#endif /*GAPLESS*/
    if ((cod = CodMicroWavCreate ()))
  {
    ret = cod->Decode (cod, &cs, &eStr);
    cod->Delete (cod);
#if 0
    if (earSpeakerReg)
    {
      register int i;
      for (i = 0; i < DEFAULT_AUDIO_BUFFER_SAMPLES / (sizeof (tmpBuf) / 2); i++)
      {
        /* Must be memset each time, because effects like EarSpeaker are
         * performed in-place in the buffer */
        memset (tmpBuf, 0, sizeof (tmpBuf));
        AudioOutputSamples (tmpBuf, sizeof (tmpBuf) / 2);
      }
    }
#endif
    if (ret != ceFormatNotFound)
    {
#ifdef GAPLESS
      codecVorbis.audioBegins = 0;
#endif
      return ret;
    }
    /* If failed, seek to the beginning and try Ogg Vorbis decoding. */
    cs.Seek (&cs, 0, SEEK_SET);
  }
  return PatchPlayCurrentFile ();
}
#else /* USE_WAV */
#ifdef GAPLESS
enum CodecError PlayGaplessOggFile (void)
{
  if (codecVorbis.audioBegins)
  {
    u_int16 data[2];

    Seek (codecVorbis.audioBegins);
    ReadFile (data, 0, 4);
    if (data[0] == 0x4f67 && data[1] == 0x6753)
    {
      /* 'OggS', assume the same parameters, skip Vorbis decoder init. */
      const char *eStr;
      register s_int16 res = OggSeekHeader (codecVorbis.audioBegins);
      ogg[1] /* .headerTypeFlag */  = 0;
      ogg[6] /* .streamSerialNumber */  = 0;
      ogg[7] /* .streamSerialNumber */  = 0;
      ogg[14] /* .cont */  = 0;
      // ogg.headerTypeFlag = 0;
      // ogg.cont = 0;
      vFirstFrame = 1;
      cs.playTimeSeconds = 0;
      cs.playTimeSamples = 0;
      /* Start decoding without recreating the decoding tables from header. */
      // putch('+');
#ifdef HAS_PATCHCODVORBISDECODE
      /* available in latest dev1000.h */
      return PatchCodVorbisDecode (&codecVorbis, &cs, &eStr, 0);
#else
      return CodVorbisDecode (&codecVorbis, &cs, &eStr);
#endif
    }
    /* 'OggS' Not found, must be a file with different parameters (or WAV) */
    // putch('-');
    Seek (0);
    return PatchPlayCurrentFile ();
  }
  // putch('n');
  return PatchPlayCurrentFile ();
}
#endif /*GAPLESS*/
#endif /* elseUSE_WAV */
