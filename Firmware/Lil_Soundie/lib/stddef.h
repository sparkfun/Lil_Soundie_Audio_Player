#ifndef _STDDEF_H_
#define _STDDEF_H_

#ifndef _SIZE_T
#define _SIZE_T
typedef unsigned int size_t;
#endif /* _SIZE_T */
typedef signed int ptrdiff_t;
typedef short wchar_t;

#ifndef NULL
#define NULL    0
#endif

#define offsetof(type, member)  ((size_t)(&((type *)0)->member))

#endif /* _STDDEF_H_ */
