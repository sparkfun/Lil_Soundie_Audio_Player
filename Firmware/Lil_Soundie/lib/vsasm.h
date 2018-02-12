/**
   \file vsasm.h Standard VSDSP assembler macros.
*/

#ifndef __VSASM_H__
#define __VSASM_H__

#ifndef ASM

/* Dummy definitions for Doxygen */
/** Creates a modifier register with \a step = -128..127, \a bufsize N*64 */
#define MAKEMOD64(stp, bufsz) (0x4000|(((bufsz)/64-1)&31)|(((stp)&255)<<6))
/** Creates a modifier register with \a step = -64..63, \a bufsz = 1..64. */
#define MAKEMOD(stp, bufsz) (0x2000|(((stp)&127)<<6)|(((bufsz)-1)&63))
/** Creates a forwards modifier register, \a bufsz = 1..8192. */
#define MAKEMODF(bufsz)     (0x8000|(((bufsz)-1)&8191))
/** Creates a backwards modifier register, \a bufsz = 1..8192. */
#define MAKEMODB(bufsz)     (0xA000|(((bufsz)-1)&8191))

#else /* !ASM */

#macro MAKEMOD64 stp,bufsz
        (0x4000|(((bufsz)/64-1)&31)|(((stp)&255)<<6))
#endm


/* stp = -64 .. 63, bufsz 1..64 */
#macro MAKEMOD stp,bufsz
	(0x2000|(((stp)&127)<<6)|(((bufsz)-1)&63))
#endm

/* stp = 1, bufsz 1..8192 */
#macro MAKEMODF bufsz
	(0x8000|(((bufsz)-1)&8191))
#endm

/* stp = -1, bufsz 1..8192 */
#macro MAKEMODB bufsz
	(0xA000|(((bufsz)-1)&8191))
#endm

#endif /* ASM */

#endif /*__VSASM_H__*/

