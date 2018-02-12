/*
  VCC Stdio package using the VSEMU/VSSIM stdio enhancement.
  
 */

#ifndef _STDIO_H_
#define _STDIO_H_

#include <stddef.h>

#ifndef NULL
#define NULL    0
#endif

/* non-ANSI */
/* typedef long long fpos_t; */
typedef struct __sfpos {
    short _pos[4]; /* 64 bits of storage needed */
} fpos_t;
typedef void FILE;

#define EOF          (-1)
/* FOPEN_MAX depends on the host system and the following value is therefore */
/* meaningless */
/* The absolute (unguaranteed) maximum of open files is 252 (256-1-3). */
#define FOPEN_MAX    20
#define FILENAME_MAX 256 /* 255 bytes of name and one '\0' */

/* fseek/ftell offset */
#ifndef SEEK_SET
#define SEEK_SET        0       /* beginning of the file */
#endif
#ifndef SEEK_CUR
#define SEEK_CUR        1       /* current position */
#endif
#ifndef SEEK_END
#define SEEK_END        2       /* end of the file */
#endif

#define stdin   (FILE *)1
#define stdout  (FILE *)2
#define stderr  (FILE *)3

/* Function prototypes */
__near FILE register __a0 *fopen(__near const char register __i0 *filename, 
				 __near const char register __i1 *mode);
__near FILE register __a0 *freopen(__near const char register __i0 *filename,
				   __near const char register __i1 *mode,
				   __near FILE register __a0 *stream);
int register __a0 fflush(__near FILE register __a0 *stream);
int register __a0 fclose(__near FILE register __a0 *stream);
int register __a0 remove(__near const char register __i0 *filename);
int register __a0 rename(__near const char register __i0 *oldname, 
			 __near const char register __i1 *newname);

int register __a0 fgetc(__near FILE register __a1 *stream);
__near char register __a0 *fgets(__near char register __i0 *s,
				 int register __a0 n,
				 __near FILE register __a1 *stream); /**< reads in at most one less than n from stream to buffer s. Reading stops at a newline or EOF. Newline is stored into the buffer. A NUL is stored after the last character in the buffer. */
int register __a0 fputc(int register __a0 c,
			__near FILE register __a1 *stream); /**< writes the character c to stream. */
int register __a0 fputs(__near const char register __i0 *s,
			__near FILE register __a1 *stream); /**< writes the string s to stream without the trailing NUL. */
int register __a0 getc(__near FILE register __a1 *stream); /**< equivalent to fgetc(), but may be macro. */
int register __a0 getchar(void);
__near char register __a0 *gets(__near char register __i0 *s);
int register __a0 putc(int register __a0 c, __near FILE register __a1 *stream);
int register __a0 putchar(int register __a0 c);
int register __a0 puts(__near const char register __i0 *s);
int register __a0 ungetc(int register __a0 c,
			 __near FILE register __a1 *stream);

size_t register __a0 fread(__near void register __i0 *ptr, 
			   size_t register __b0 size,
			   size_t register __b1 nobj, 
			   __near FILE register __a1 *stream); /**< Reads pairs of bytes from the stream and stores them into the word buffer. The first byte goes to the high bits of a word. */
size_t register __a0 fwrite(__near const void register __i0 *ptr, 
			    size_t register __b0 size, 
			    size_t register __b1 nobj, 
			    __near FILE register __a1 *stream); /**< Writes pairs of bytes to the stream. The high bits of a word are written to the first byte. */

int register __a0 fseek(__near FILE register __a0 *stream, 
			long register __b offset, 
			int register __a1 origin);
long register __a ftell(__near FILE register __a0 *stream);
void rewind(__near FILE register __a0 *stream);
int register __a0 fgetpos(__near FILE register __a0 *stream, 
			  __near fpos_t register __i0 *ptr);
int register __a0 fsetpos(__near FILE register __a0 *stream, 
			  __near const fpos_t register __i0 *ptr);

void clearerr(__near FILE register __a0 *stream);
int register __a0 feof(__near FILE register __a0 *stream);
int register __a0 ferror(__near FILE register __a0 *stream);
void perror(__near const char register __i0 *s);

int fprintf(__near FILE *stream, __near const char *fmt, ...);
int printf(__near const char *fmt, ...);
int sprintf(__near char *s, __near const char *fmt, ...);

int sscanf(const char *str, const char *fmt, ...);

int tinysprintf(char *str, const char *fmt, ...);
int tinyprintf(const char *fmt, ...);
int tinyfprintf(FILE *fp, const char *fmt, ...);

#endif /* _STDIO_H_ */
