#ifndef _STDARG_H_
#define _STDARG_H_
typedef char *va_list;

#define va_start(ap, last) (ap = ((va_list)&(last)))
#define va_arg(ap, type)   ((type *)(ap -= sizeof(type)))[0]
#define va_end(ap)

#endif /* _STDARG_H_ */
