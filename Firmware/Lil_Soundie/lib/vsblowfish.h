#ifndef __VSBLOWFISH_H__
#define __VSBLOWFISH_H__

#define NUM_SUBKEYS  18
#define NUM_S_BOXES  4
#define NUM_ENTRIES  256

#define MAX_STRING   256
#define MAX_PASSWD   56  // 448bits

// #define BIG_ENDIAN
// #define LITTLE_ENDIAN

#include <vstypes.h>

#ifndef ASM

struct VSBLOWFISH {
    u_int32 SB[NUM_ENTRIES][NUM_S_BOXES];
    u_int32 PA[NUM_SUBKEYS];
};

void Blowfish_Subkeys(struct VSBLOWFISH *bf, const unsigned char *key, int len);
void Blowfish_En(struct VSBLOWFISH *bf, u_int32 *, u_int32 *);
void Blowfish_De(struct VSBLOWFISH *bf, u_int32 *, u_int32 *);
/* The following versions perform word-swap of the u_int32 datas. */
void Blowfish_EnS(struct VSBLOWFISH *bf, u_int32 *, u_int32 *);
void Blowfish_DeS(struct VSBLOWFISH *bf, u_int32 *, u_int32 *);
void Blowfish_Reset(struct VSBLOWFISH *bf);
void Blowfish_SetKey(struct VSBLOWFISH *bf, const unsigned char *key, s_int16 keyLen);
/* bytes must be multiple of 8! */
void Blowfish_Encrypt(struct VSBLOWFISH *bf, void *, s_int16 bytes);
/* bytes must be multiple of 8! */
void Blowfish_Decrypt(struct VSBLOWFISH *bf, void *, s_int16 bytes);

#endif/*!ASM*/
#endif /*!__VSBLOWFISH_H__*/
