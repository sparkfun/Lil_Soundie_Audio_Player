#ifndef __DSPLIB_H__
#define __DSPLIB_H__

#include <vstypes.h>

/** Bi-quad IIR coefficients. */
struct DSP_IIR2COEFF16 {
    s_int16 b0,b1,b2,a1,a2;
};

/** Calculates bi-quad IIR for one sample. 26 cycles.
 */
__near auto s_int32 DspIir2Lq(register __a0 s_int16 val,
			      register __i0 __near const __mem_y struct DSP_IIR2COEFF16 *baCoeff,
			      register __i2 __near s_int16 mem[5]);
/** Calculates bi-quad IIR for a block of samples. 27+10*n cycles.
 */
__near auto void DspIir2nLq(register __i1 __near s_int16 *buffer,
			    register __i0 __near const __mem_y struct DSP_IIR2COEFF16 *baCoeff,
			    register __i2 __near s_int16 mem[5], register __a1 s_int16 n);

/** Calculates 64-bit multiplication result. 12 cycles.
 */
auto void Mul64(register __reg_a s_int32 a, register __reg_d s_int32 d,
		register __i0 s_int32 reslohi[2]);

/** Calculates sine from phase.
    \param phase - fractional value, range [-1.0 1.0) is [-180 180) degrees
    \return value in range of [-1.0 1.0)
 */
s_int32 frsine32(register __reg_c s_int32 phase);

/*
  CRC16 functions for the CRC used for the sector data of MMC/SD cards.
 */
extern const u_int16 sdCrcTable[256];
auto u_int16 SdCrcBlock(register __i2 u_int16 *block, register __a0 s_int16 words);
auto u_int16 SdCrcBlockChained(register __i2 u_int16 *block, register __a0 u_int16 startCrc, register __a1 s_int16 words);

/*
  CRC32 - table and functions for 32-bit CRC, which is used e.g. in Ogg.
 */
extern const u_int32 __mem_y oggCrcTable[256];
//A is 32-bit variable, b is the new byte
#define UPDCRC32(a,b) (oggCrcTable[(((s_int32)(a)>>24) ^ (b)) & 0xff] ^ ((a)<<8))
/* u_int32 crc32 = 0;
   crc32 = UPDCRC32(crc32, *s++);
*/
/* Calculates a CRC-32 from the bytes packed into the words. i.e. only
   even number of bytes can be calculated using this routine. */
auto u_int32 OggCrcBlockChained(register __i2 u_int16 *block, register __c0 s_int16 words, register __reg_a u_int32 startCrc);

#endif /* !__DSPLIB_H__ */
