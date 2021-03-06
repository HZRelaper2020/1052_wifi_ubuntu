/*
 * prototypes for functions defined in bcmstdlib.c
 * $Copyright Open Cypress Semiconductor$
 * $Id: bcmstdlib.h,v 13.31.14.1 2010-12-08 14:28:41 harishv Exp $:
 */

/*
 * bcmstdlib.h file should be used only to construct an OSL or alone without any OSL
 * It should not be used with any orbitarary OSL's as there could be a conflict
 * with some of the routines defined here.
*/

#ifndef	_BCMSTDLIB_H
#define	_BCMSTDLIB_H

#include <typedefs.h>
#include <bcmdefs.h>
#include <stdarg.h>

#ifndef INT_MAX
#define INT_MAX 2147483647 /* from limits.h */
#endif


/* For backwards compatibility, define "BWL_NO_INTERNAL_STDLIB_SUPPORT" to
 * exclude support for the BRCM stdlib APIs. This should be cleaned-up such
 * that platforms that require the BRCM stdlib API should simply define
 * "BWL_INTERNAL_STDLIB_SUPPORT". This would eliminate the need for the
 * following #ifndef check.
 */
#ifndef BWL_NO_INTERNAL_STDLIB_SUPPORT
#define BWL_INTERNAL_STDLIB_SUPPORT
#endif

#ifdef BWL_INTERNAL_STDLIB_SUPPORT
/* This should be cleaned-up such that platforms that require the BRCM stdlib
 * API should simply define "BWL_INTERNAL_STDLIB_SUPPORT". This would eliminate
 * the need for the following #ifdef check.
 */
#if !defined(vxworks) && !defined(_WIN32) && !defined(_CFE_) && !defined(EFI)

typedef int FILE;
#define stdout ((FILE *)1)
#define stderr ((FILE *)2)

/* i/o functions */
extern int fputc(int c, FILE *stream);
extern void putc(int c);
/* extern int putc(int c, FILE *stream); */
#define putchar(c) putc(c)
extern int fputs(const char *s, FILE *stream);
extern int puts(const char *s);
extern int getc(void);
extern bool keypressed(void);

/* bcopy, bcmp, and bzero */
#define	bcopy(src, dst, len)	memcpy((dst), (src), (len))
#define	bcmp(b1, b2, len)	memcmp((b1), (b2), (len))
#define	bzero(b, len)		memset((b), '\0', (len))

extern unsigned long rand(void);

#define	atoi(s)	((int)(strtoul((s), NULL, 10)))

#endif /* !vxworks && !_WIN32 && !_CFE_ && !EFI */

#if !defined(_WIN32) || defined(EFI)
/* string functions */
#define PRINTF_BUFLEN	512
extern int printf(const char *fmt, ...);
extern int BCMROMFN(sprintf)(char *buf, const char *fmt, ...);

extern int BCMROMFN(strcmp)(const char *s1, const char *s2);
extern size_t BCMROMFN(strlen)(const char *s);
extern char *BCMROMFN(strcpy)(char *dest, const char *src);
extern char *BCMROMFN(strstr)(const char *s, const char *find);
extern char *BCMROMFN(strncpy)(char *dest, const char *src, size_t n);
extern char *BCMROMFN(strcat)(char *d, const char *s);

extern int BCMROMFN(strncmp)(const char *s1, const char *s2, size_t n);
extern char *BCMROMFN(strchr)(const char *str, int c);
extern char *BCMROMFN(strrchr)(const char *str, int c);
extern size_t BCMROMFN(strspn)(const char *s1, const char *s2);
extern size_t BCMROMFN(strcspn)(const char *s1, const char *s2);
extern unsigned long BCMROMFN(strtoul)(const char *cp, char **endp, int base);
#define strtol(nptr, endptr, base) ((long)strtoul((nptr), (endptr), (base)))

extern void *BCMROMFN(memmove)(void *dest, const void *src, size_t n);
extern void *BCMROMFN(memchr)(const void *s, int c, size_t n);

extern int BCMROMFN(vsprintf)(char *buf, const char *fmt, va_list ap);
/* mem functions */
/* For EFI, using EFIDriverLib versions */
/* Cannot use memmem in ROM because of character array initialization wiht "" in gcc */
extern void *memset(void *dest, int c, size_t n);
/* Cannot use memcpy in ROM because of structure assignmnets in gcc */
extern void *memcpy(void *dest, const void *src, size_t n);
extern int BCMROMFN(memcmp)(const void *s1, const void *s2, size_t n);

#endif /* !_WIN32 || EFI */
#endif   /* BWL_INTERNAL_STDLIB_SUPPORT */

extern int BCMROMFN(snprintf)(char *str, size_t n, char const *fmt, ...);
extern int BCMROMFN(vsnprintf)(char *buf, size_t size, const char *fmt, va_list ap);

#endif 	/* _BCMSTDLIB_H */
