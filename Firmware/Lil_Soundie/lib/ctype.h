/**
   \file ctype.h Standard C header file.
*/
#ifndef CTYPE_H
#define CTYPE_H

int isalnum(int register __i0 c); /**< return TRUE for alphanumerical. */
int isalpha(int register __i0 c); /**< return TRUE for alphabetic character. */
int iscntrl(int register __i0 c); /**< return TRUE for control characters. */
int isdigit(int register __i0 c); /**< return TRUE for digits 0-9. */
int isgraph(int register __i0 c); /**< return TRUE for printable characters except space. */
int islower(int register __i0 c); /**< return TRUE for lowercase characters. */
int isprint(int register __i0 c); /**< return TRUE for printable characters, including space. */
int ispunct(int register __i0 c); /**< return TRUE for printable characters that is not a space or alphanumeric. */
int isspace(int register __i0 c); /**< return TRUE for white-space characters. */
int isupper(int register __i0 c); /**< return TRUE for uppercase characters. */
int isxdigit(int register __i0 c); /**< return TRUE for hexadecimal digits. */
int register __i0 tolower(int register __i0 c); /**< convert letter c to lower case if possible. */
int register __i0 toupper(int register __i0 c); /**< convert letter c to upper case if possible. */

#endif /* CTYPE_H */
