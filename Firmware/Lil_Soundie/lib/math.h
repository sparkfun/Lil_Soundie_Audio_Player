#ifndef _MATH_H_
#define _MATH_H_

__near double frexp(double value, __near int *eptr);
__near double ldexp(double value, int exp);
__near double fabs(double value);
#define frexp __builtin_frexp
#define ldexp __builtin_ldexp
#define fabs  __builtin_fabs

__near double _modf(double value, __near double *iptr);
#define modf(v,p) _modf(v,p)
#define _oldmodf(v,p) (*(p)=(long)(v),(v)-(double)(long)(v))
__near double fmod(double x, double y);

/* NOTE: Check what this really should be !!!*/
#define HUGE_VAL      3.402823466385288598e38

#define EDOM        33
#define ERANGE      34

#define M_E		2.7182818284590452354
#define M_LOG2E		1.4426950408889634074
#define M_LOG10E	0.43429448190325182765
#define M_LN2		0.69314718055994530942
#define M_LN10		2.3025850929940456840
#define M_PI		3.1415926535897932384
#define M_PI_2		1.5707963267948966192
#define M_PI_4		0.78539816339744830962
#define M_1_PI		0.31830988618379067154
#define M_2_PI		0.63661977236758134308
#define M_2_SQRTPI	1.12837916709551257390
#define M_SQRT2		1.4142135623730950488
#define M_SQRT1_2	0.70710678118654752440

__near double acos(double);
__near double asin(double);
__near double atan(double);
__near double cos(double);
__near double cosh(double);
__near double exp(double);
__near double log10(double);
__near double log(double);
__near double sin(double);
__near double sinh(double);
__near double sqrt(double);
__near double tan(double);
__near double tanh(double);

__near double floor(double);
__near double ceil(double);
__near double pow(double, double);
__near double atan2(double, double);

__near auto double SqrtF(double); /* result with 16-bit precision */
__near auto unsigned short SqrtI(register __c unsigned long x);

__near auto double SqrtF32(double); /* result with 32-bit precision */
__near auto unsigned long SqrtI32(register __c unsigned long x); /* 16.16 result */
__near double ISqrt(double x); /* x^(-1/2) */
__near unsigned short LongLog2(register __a long x);
#ifdef __VSDSP__
__near double log2(double x); /*about 480 cycles*/
__near short logdb(register __b0 unsigned short x); /*returns approximate decibels, -96 for 0 and 1, about 45 cycles */
__near short logdb32(register __b unsigned long x); /*returns approximate decibels, 0 for 0 and 1, 192 for max, uses logdb, about 62 cycles*/
#else
#define log2(x) (log(x)/log(2.))
#define logdb(x) (x ? (int)(log(x)/log(2.)*6.0)-96 : -96) /*not bit-exact!*/
#define logdb32(x) (x ? (int)(log(x)/log(2.)*6.0) : 0) /*not bit-exact!*/
#endif

#ifndef __VSDSP__
#define MulSh24(a,b) ((long)((long long)(a) * (b) >> 24))
#define MulSh20(a,b) ((long)((long long)(a) * (b) >> 20))
#else
/* These functions should be called in integer mode */
auto long MulSh24(register __a long c, register __d long d);
auto long MulSh20(register __a long c, register __d long d);
#endif
/* Prototypes for 16-bit divisions to get both division result (low word)
   and remainder (high word) in one call. */
unsigned long _divide16unsigned(register __b0 unsigned short dividend, register __a0 unsigned short D);
unsigned long _divide16signed(register __b0 unsigned short dividend, register __a0 unsigned short D);


#endif /* _MATH_H_ */
