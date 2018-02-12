/**
   \file vstypes.h Standard VSDSP types and definitions.
	This standard file contains standard definitions
	and types. It also makes it easier to compile the
	same source code with both VSDSP and Unix compilers
	by undefining symbols that are only understood by
	VSDSP C compilers.
 */

#ifndef __VS_TYPES_H__
#define __VS_TYPES_H__

/* Definitions that are common to C and assembly language */
/** Mode register: Zero is fractional, non-saturated mode */
#define MR_NONE 0
/** Mode register: Integer mode */
#define MR_INT  0x200
/** Mode register: Saturation mode */
#define MR_SAT  0x400

#ifdef ASM	/* ASM definitions */
#include <vsasm.h> /* hide preprocessor tokens that only vsa understands */
#endif /* ASM */


#ifndef ASM	/* C only definitions */

/** Default signed 16-bit integer type */
typedef short s_int16;
/** Default unsigned 16-bit integer type */
typedef unsigned short u_int16;

/** printf() format for signed 16-bit value */
#define FMT_S16 "d"
/** printf() format for unsigned 16-bit value */
#define FMT_U16 "u"
/** printf() format for 16-bit hex value */
#define FMT_H16 "x"


#ifndef __VSDSP__

#define memsetY memset

/** Default signed 32-bit integer type */
typedef int s_int32;
/** Default unsigned 32-bit integer type */
typedef unsigned int u_int32;

/** printf() format for signed 32-bit value, UNIX version */
#define FMT_S32 "d"
/** printf() format for unsigned 32-bit value, UNIX version */
#define FMT_U32 "u"
/** printf() format for 32-bit hex value, UNIX version */
#define FMT_H32 "x"


/* Remove VSDSP specific special keywords to make code compatible */
#define register
#define __reg_a
#define __reg_b
#define __reg_c
#define __reg_d
#define __a0
#define __a1
#define __b0
#define __b1
#define __c0
#define __c1
#define __d0
#define __d1
#define __i0
#define __i1
#define __i2
#define __i3
#define __mem_x
#define __mem_y
#define __near
#define __far
#define __align
#define auto

#if 0
typedef float  f_int16;
typedef double f_int32;
#else

/**
   Default 16-bit fractional type. This is the Non-VSDSP definition.
	The VSDSP definition reads typedef __fract short f_int16 .
	Addition and subtraction can be used as normal with this type, but
	some operations must be executed using macros: FMUL16() for
	multiplication, FDIV16() for division and DBL2F16() for
	double-to-fract conversion.
*/
typedef s_int16 f_int16;
/**
   Default 32-bit fractional type. This is the Non-VSDSP definition.
	The VSDSP definition reads typedef __fract long f_int32 .
	Addition and subtraction can be used as normal with this type, but
	some operations must be executed using macros: FMUL32() for
	multiplication, FDIV32() for division and DBL2F32() for
	double-to-fract conversion.
*/
typedef s_int32 f_int32;
/** 32-bit fractional multiplication (Unix version). */
#define FMUL32(a,b) (f_int32)((long long)(f_int32)(a) * (f_int32)(b) >> 31)
/** 16-bit fractional multiplication (Unix version). */
#define FMUL16(a,b) (f_int16)((long)(f_int16)(a) * (f_int16)(b) >> 15)
/** 32-bit fractional division (Unix version). */
#define FDIV32(a,b) (f_int32)(((long long)(f_int32)(a)<<31) / (f_int32)(b))
/** 16-bit fractional division (Unix version). */
#define FDIV16(a,b) (f_int16)(((long)(f_int16)(a)<<15) / (f_int16)(b))
/** 32-bit fractional double-to-fract conversion (Unix version). */
#define DBL2F32(a)  (f_int32)((a)*(32768.0*65536.0))
/** 16-bit fractional double-to-fract conversion (Unix version). */
#define DBL2F16(a)  (f_int16)((a)*32768.0)
#endif


/* for compatibility with math.h */
#define SqrtI(a) ((u_int16)sqrt(a))
#define SqrtI32(a) ((u_int32)(sqrt(a)*65536.0))



#else /*else !__VSDSP__*/

#define __mem_x __x
#define __mem_y __y

/** Default signed 32-bit integer type */
typedef long s_int32;
/** Default unsigned 32-bit integer type */
typedef unsigned long u_int32;

/** printf() format for signed 32-bit value, VSDSP version */
#define FMT_S32 "ld"
/** printf() format for unsigned 32-bit value, VSDSP version */
#define FMT_U32 "lu"
/** printf() format for 32-bit hex value, VSDSP version */
#define FMT_H32 "lx"

#define __reg_a __a
#define __reg_b __b
#define __reg_c __c
#define __reg_d __d

typedef __fract short f_int16;
typedef __fract long f_int32;
#define FMUL32(a,b) ((f_int32)(a) * (f_int32)(b))
#define FMUL16(a,b) ((f_int16)(a) * (f_int16)(b))
#define FDIV32(a,b) ((f_int32)(a) / (f_int32)(b))
#define FDIV16(a,b) ((f_int16)(a) / (f_int16)(b))
#define DBL2F32(a)  (a)
#define DBL2F16(a)  (a)
#endif /*__VSDSP__*/

#ifndef USEX
/** Direct memory write to X memory */
#define USEX(x) (*(__mem_x volatile u_int16 *)(u_int16)(x))
#endif /*!USEX*/
#ifndef USEY
/** Direct memory write to Y memory */
#define USEY(x) (*(__mem_y volatile u_int16 *)(u_int16)(x))
#endif /*!USEY*/

/** overlay entry function return value */
typedef unsigned int entry_u_int16; // Las

#endif /*!ASM*/

#endif /*__VS_TYPES_H__*/
