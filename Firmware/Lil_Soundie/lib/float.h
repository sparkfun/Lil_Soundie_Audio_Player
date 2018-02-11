/**
   \file float.h Standard C header file. The float type should not be use with VCC (only 15-bit accuracy).
*/
#ifndef _FLOAT_H_
#define _FLOAT_H_

/* Note: FLT_DIG must be at least 6 and DBL_DIG must be at least 10 */
/* to be ISO 9899 conformant */

#define FLT_RADIX       2               /* b */
#define FLT_ROUNDS      -1              /* FP addition rounds to nearest */

#define FLT_MANT_DIG    15              /* p */
#define FLT_EPSILON     6.10351562E-5F  /* b**(1-p) */
#define FLT_DIG         4               /* floor((p-1)*log10(b))+(b == 10) */
#define FLT_MIN_EXP     (-1021)         /* emin */
#define FLT_MIN         2.2250738585072014E-308 /* b**(emin-1) */
#define FLT_MIN_10_EXP  (-307)          /* ceil(log10(b**(emin-1))) */
#define FLT_MAX_EXP     1024            /* emax */
#define FLT_MAX         1.7976931348623157E+308 /* (1-b**(-p))*b**emax */
#define FLT_MAX_10_EXP  308             /* floor(log10((1-b**(-p))*b**emax)) */

#define DBL_MANT_DIG    31
#define DBL_EPSILON     4.6566128752458E-10
#define DBL_DIG         9
#define DBL_MIN_EXP     (-1021)
#define DBL_MIN         4.45014771494214E-308  /* $7fffffff*2^(-1021-31) */
#define DBL_MIN_10_EXP  (-307)
#define DBL_MAX_EXP     1024
#define DBL_MAX         1.79769313486232E+308  /* $80000000*2^(1024-31) */
#define DBL_MAX_10_EXP  308

#define LDBL_MANT_DIG   DBL_MANT_DIG
#define LDBL_EPSILON    DBL_EPSILON
#define LDBL_DIG        DBL_DIG
#define LDBL_MIN_EXP    DBL_MIN_EXP
#define LDBL_MIN        DBL_MIN
#define LDBL_MIN_10_EXP DBL_MIN_10_EXP
#define LDBL_MAX_EXP    DBL_MAX_EXP
#define LDBL_MAX        DBL_MAX
#define LDBL_MAX_10_EXP DBL_MAX_10_EXP

#endif /* _FLOAT_H_ */
