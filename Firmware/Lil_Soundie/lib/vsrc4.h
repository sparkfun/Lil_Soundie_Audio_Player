#ifndef __VS_RC4_H__
#define __VS_RC4_H__

#include <vstypes.h>

#define RC4_TABLESIZE 256
//#define TARGET_LITTLE_ENDIAN 1
//#define RC4KS_IN_Y

#ifdef ASM

#else /*ASM*/

/* Key structure */
typedef struct RC4_KEYSTRUCT {
    u_int16 S[RC4_TABLESIZE]; /* only low 8 bits are used/valid */
    s_int16 i, j;
} RC4_KEYSTRUCT;

#ifdef RC4KS_IN_Y
/**
  \function RC4_KeySetup
  Generate the key control structure.  Key can be any size.
  \param rc4KS  - A RC4_KEYSTRUCT to be initialized.
  \param szKey  - Size of the key, in bytes.
  \param bKey   - Pointer to the key (bytes).
*/
void RC4_KeySetup(register __i2 __mem_y RC4_KEYSTRUCT  *rc4KS,
		  s_int16 szKey, const unsigned char *bKey );
/**
  \function RC4_KeySetupPacked
  Generate the key control structure.  Key can be any size.
  \param rc4KS     - A RC4_KEYSTRUCT to be initialized.
  \param szKey     - Size of the key, in bytes.
  \param packedKey - Pointer to the key.
*/
void RC4_KeySetupPacked(register __i2 __mem_y RC4_KEYSTRUCT  *rc4KS,
			s_int16 szKey, const u_int16 *packedKey);
/**
   \function RC4_Cipher
   \param rc4KS    - the KEYSTRUCT created using RC4_KeySetup.
   \param szBuffer - Size of buffer, in bytes.
   \param buffer   - Buffer to be encrypted in place.
   \param bufInd   - start byte offset to the buffer
*/
void RC4_Cipher(register __i2 __mem_y RC4_KEYSTRUCT *rc4KS, s_int16 szBuffer,
		unsigned char *bBuffer, s_int16 bufInd);
/**
   \function RC4_CipherPacked
   \param rc4KS    - the KEYSTRUCT created using RC4_KeySetup.
   \param szBuffer - Size of buffer, in bytes.
   \param wBuffer  - Packed buffer to be encrypted in place.
   \param bufInd   - start byte offset to the buffer
*/
void RC4_CipherPacked(register __i2 __mem_y RC4_KEYSTRUCT *rc4KS,
		      s_int16 szBuffer, u_int16 *wBuffer, s_int16 bufInd);
/* Returns the next 8-bit pseudo-random value. */
s_int16 RC4_Output(register __i2 __mem_y RC4_KEYSTRUCT *rc4KS);
#else
/**
  \function RC4_KeySetup
  Generate the key control structure.  Key can be any size.
  \param rc4KS  - A RC4_KEYSTRUCT to be initialized.
  \param szKey  - Size of the key, in bytes.
  \param bKey   - Pointer to the key (bytes).
*/
void RC4_KeySetup(register __i2 RC4_KEYSTRUCT  *rc4KS,
		  s_int16 szKey, const unsigned char *bKey );
/**
  \function RC4_KeySetupPacked
  Generate the key control structure.  Key can be any size.
  \param rc4KS     - A RC4_KEYSTRUCT to be initialized.
  \param szKey     - Size of the key, in bytes.
  \param packedKey - Pointer to the key.
*/
void RC4_KeySetupPacked(register __i2 RC4_KEYSTRUCT  *rc4KS,
			s_int16 szKey, const u_int16 *packedKey);
/**
   \function RC4_Cipher
   \param rc4KS    -- the KEYSTRUCT created using RC4_KeySetup.
   \param szBuffer -- Size of buffer, in bytes.
   \param bBuffer  -- Buffer to be encrypted in place (bytes).
   \param bufInd   -- start byte offset to the buffer
*/
void RC4_Cipher(register __i2 RC4_KEYSTRUCT *rc4KS, s_int16 szBuffer,
		unsigned char *bBuffer, s_int16 bufInd);
/**
   \function RC4_CipherPacked
   \param rc4KS    - the KEYSTRUCT created using RC4_KeySetup.
   \param szBuffer - Size of buffer, in bytes.
   \param wBuffer  - Packed buffer to be encrypted in place.
   \param bufInd   - start byte offset to the buffer
*/
void RC4_CipherPacked(register __i2 RC4_KEYSTRUCT *rc4KS,
		      s_int16 szBuffer, u_int16 *wBuffer, s_int16 bufInd);

/* Returns the next 8-bit pseudo-random value. */
s_int16 RC4_Output(register __i2 RC4_KEYSTRUCT *rc4KS);
#endif



#endif/*elseASM*/

#endif /* __VS_RC4_H__ */
