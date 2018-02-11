#ifndef _STRING_H_
#define _STRING_H_

#include <vstypes.h>
#include <stddef.h>

/* only one locale supported, strcoll is reduced to strcmp */
#define strcoll strcmp

__near char register __i0 *strcpy(__near char register __i0 *s, 
				  __near const char register __i1 *ct);
__near char register __i0 *strncpy(__near char register __i0 *s, 
				   __near const char register __i1 *ct, 
				   size_t register __a0 n);
__near char register __i0 *strcat(__near char register __i0 *s, 
				  __near const char register __i1 *ct);
__near char register __i0 *strncat(__near char register __i0 *s, 
				   __near const char register __i1 *ct, 
				   size_t register __a0 n);
int register __a0 strcmp(__near const char register __i0 *cs, 
			 __near const char register __i1 *ct);
int register __a0 strncmp(__near const char register __i0 *cs, 
			  __near const char register __i1 *ct, 
			  size_t register __a0 n);
__near char register __i0 *strchr(__near const char register __i0 *cs,
				  int register __a0 c);
__near char register __i0 *strrchr(__near const char register __i0 *cs,
				   int register __a0 c);
size_t register __a0 strspn(__near const char register __i0 *cs, 
			    __near const char register __i1 *ct);
size_t register __a0 strcspn(__near const char register __i0 *cs, 
			     __near const char register __i1 *ct);
__near char register __i0 *strpbrk(__near const char register __i0 *cs, 
				   __near const char register __i1 *ct);
__near char register __i0 *strstr(__near const char register __i0 *cs, 
				  __near const char register __i1 *ct);
size_t register __a0 strlen(__near const char register __i0 *cs);
__near char register __i0 *strerror(int register __a0 n);
__near char register __i0 *strtok(__near char register __i0 *s, 
				  __near const char register __i1 *ct);

__near void register __i0 *memcpy(__near void register __i0 *d,
				  __near const void register __i1 *s, 
				  size_t register __a0 n); /**< Copies n words from s to d. The memory areas should not overlap. Returns a pointer to d. */

__near __mem_y void register __i0 *memcpyXY(__near __mem_y void register __i0 *d,
				      __near const void register __i1 *s,
				      size_t register __a0 n); /**< Copies n words from s to d, where the destination is in Y memory. Returns a pointer to d. */
__near void register __i0 *memcpyYX(__near void register __i0 *d,
				    __near __mem_y const void register __i1 *s,
				    size_t register __a0 n); /**< Copies n words from s to d, where the source is in Y memory. Returns a pointer to d. */
__near __mem_y void register __i0 *memcpyYY(__near __mem_y void register __i0 *d, 
					__near __mem_y const void register __i1 *s, 
					size_t register __a0 n); /**< Copies n words from s to d, where both the source and the destination is in Y memory. The memory areas should not overlap. Returns a pointer to d. */
__near void memcpyii(__near void register __i0 *d, 
		     __near const void register __i1 *s, 
		     size_t register __a0 n); /**< Copies n instruction words from s to d, where both the source and the destination is in I memory. The memory areas should not overlap. */
__near void memmoveii(__near void register __i0 *d, 
		     __near const void register __i1 *s, 
		     size_t register __a0 n); /**< Copies n instruction words from s to d, where both the source and the destination is in I memory. The memory areas can overlap. */


__near void register __i0 *memmove(__near void register __i0 *s,
				   const __near void register __i1 *ct,
				   unsigned short register __a0 n);
int register __a0 memcmp(__near const void register __i0 *cs, 
			 __near const void register __i1 *ct,
			 size_t register __a0 n);
int register __a0 memcmpY(__mem_y __near const void register __i0 *cs, 
			  __mem_y __near const void register __i1 *ct,
			 size_t register __a0 n);
int register __a0 memcmpXY(__near const void register __i0 *cs, 
			   __mem_y __near const void register __i1 *ct,
			   size_t register __a0 n);
#define memcmpYX(a,b,c) memcmpXY((b),(a),(c))
__near void register __i0 *memchr(__near const void register __i0 *cs,
				  int register __a0 c, 
				  size_t register __a1 n);
__near void register __i0 *memset(__near void register __i0 *s,
				  int register __a1 c,
				  size_t register __a0 n);
__near void register __i0 *memsetY(__near __mem_y void register __i0 *s,
				  int register __a1 c,
				  size_t register __a0 n);
__near void register __i0 *memseti(__near void register __i0 *s,
				  register __a unsigned long c,
				  size_t register __c0 n);
__near void memclearXY(register __i0 unsigned short *p, register __a0 short c); /**< Clears c words in both X and Y memory starting from p. */
size_t register __a0 strxfrm(__near char register __i0 *s1, 
			     __near const char register __i1 *s2, 
			     size_t register __a0 n);

void memswap(register __i0 void *a, register __i1 void *b,
	     register __a0 size_t size); /**< Swaps two memory areas. The size gives the length of the area. */
void memswapy(register __i0 __mem_y void *a, register __i1 __mem_y void *b,
	      register __a0 size_t size); /**< Swaps two memory ares, both are in Y memory. The size gives the length of the area. */
void memswapxy(register __i0 void *a, register __i1 __mem_y void *b,
	     register __a0 size_t size); /**< Swaps two memory ares, the secondis in Y memory. The size gives the length of the area. */
#define memswapyx(a,b,c) memswapxy((b),(a),(c))
/** Copies bytes from one word buffer to another. The bytes are packed in
    big-endian format, i.e. the first byte to the high bits of the word
    and the next byte to the low bits of the word. When using non-aligned
    start or end, the other half of the word retains its old value. */
__near void MemCopyPackedBigEndian(register __i0 __near unsigned short *dst,
				   register __a0 unsigned short dstbyteidx,
				   register __i1 __near unsigned short *src,
				   register __a1 unsigned short srcbyteidx,
				   register __b0 unsigned short byteSize);
/** Copies bytes from one word buffer in Y to another in X memory. The bytes
    are packed in big-endian format, i.e. the first byte to the high bits of
    the word and the next byte to the low bits of the word.
    When using non-aligned start or end, the other half of the word retains
    its old value.  */
__near void MemCopyPackedBigEndianYX(register __i0 __near unsigned short *dst,
				   register __a0 unsigned short dstbyteidx,
				   register __i1 __near __mem_y unsigned short *src,
				   register __a1 unsigned short srcbyteidx,
				   register __b0 unsigned short byteSize);
/** Copies bytes from one word buffer in X to another in Y memory. The bytes
    are packed in big-endian format, i.e. the first byte to the high bits of
    the word and the next byte to the low bits of the word.
    When using non-aligned  start or end, the other half of the word retains
    its old value.  */
__near void MemCopyPackedBigEndianXY(register __i0 __near __mem_y unsigned short *dst,
				   register __a0 unsigned short dstbyteidx,
				   register __i1 __near unsigned short *src,
				   register __a1 unsigned short srcbyteidx,
				   register __b0 unsigned short byteSize);
/** Copies bytes from one word buffer in Y to another in Y memory. The bytes
    are packed in big-endian format, i.e. the first byte to the high bits of
    the word and the next byte to the low bits of the word. When using
    non-aligned start or end, the other half of the word retains its old
    value.  */
__near void MemCopyPackedBigEndianYY(register __i0 __near __mem_y unsigned short *dst,
				   register __a0 unsigned short dstbyteidx,
				   register __i1 __near __mem_y unsigned short *src,
				   register __a1 unsigned short srcbyteidx,
				   register __b0 unsigned short byteSize);
/** Scales a vector by shifting the amount specified. If shift is positive,
    the shift is up, with saturation. If shift is negative, the shift is
    signed shift down. Nothing is done if the number of elements is negative
    or zero. */
void ShiftSatVector(register __i0 short *vector, register __a0 short elements, register __a1 short shift);
#endif /* _STRING_H_ */
