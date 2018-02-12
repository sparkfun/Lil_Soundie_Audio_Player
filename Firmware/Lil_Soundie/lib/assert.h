/**
   \file assert.h Standard C header file.
 */
#undef assert

#ifdef NDEBUG
#define assert(exp)      (0)
#else
#define assert(exp)      ((exp) ? 0 : __assert(#exp,__FILE__, __LINE__))
#endif

