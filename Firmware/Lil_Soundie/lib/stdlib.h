#ifndef _STDLIB_H_
#define _STDLIB_H_

#include <vstypes.h>

#ifndef _SIZE_T
#define _SIZE_T
typedef unsigned int size_t;
#endif /* _SIZE_T_ */

#ifndef NULL
#define NULL    0
#endif

#define RAND_MAX        0x7fff

extern __mem_y short rand_seed; /*allocate this yourself to prevent init_y*/
int rand(void);
void srand(register __a0 unsigned int seed);
void exit(register __a0 int exitValue);
#define abort() exit(-1)

#define EXIT_FAILURE    1
#define EXIT_SUCCESS    0

#define abs(a) __builtin_abs(a)
#define labs(a) __builtin_labs(a)

__near int atoi(__near const char *s);
__near unsigned short atou(__near register __i2 const char *s);
__near long strtol(__near const char *s, __near char * __near *endp, int base);
#define strtoul(a,b,c) (unsigned long)strtol(a,b,c)

__near double strtod(__near const char *nptr, __near char * __near *endptr);
#define atof(a) strtod(a,NULL)

#define RANDOM_MAX 0x7fffffffL
extern __near long random_state; /*allocate this yourself to prevent init_x*/
__near long random(void);
__near void srandom(register __a unsigned long x);

unsigned short QsortLog2(register __a0 short x);

#ifdef VS1000_QSORT
void qsortxy(int ymem, void *base, int  nmemb, int  size,
 	     int (*compar)(const void *, const void *));
#define qsort(b,n,s,c)  qsortxy(0,(b),(n),(s),(c))
#define qsorty(b,n,s,c) qsortxy(1,(b),(n),(s),(c))
#else
void qsort(void *base, int/*size_t*/ nmemb, int/*size_t*/ size,
	   int (*compar)(const void *, const void *));
#define qsorty(b,n,s,c) qsort((b),(n),-(s),(c))
#endif

short CountBitsLong(register __a unsigned long val); /**< Counts the number of 1-bits in a 32-bit value. Takes 12 to 84 cycles. */

#endif /* _STDLIB_H_ */
