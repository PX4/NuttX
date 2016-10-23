/****************************************************************************
 * include/ctype.h
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_CTYPE_H
#define __INCLUDE_CTYPE_H

/* There is no consistent ctype implementation, just a smattering of
 * functions.  Individually, they are okay, but a more standard, data lookup
 * approach would make more sense if used extensively.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __cplusplus

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define _U  01
#define _L  02
#define _N  04
#define _S  010
#define _P  020
#define _C  040
#define _X  0100
#define _B  0200

/****************************************************************************
 * Name: isblank (since C99 / C++11)
 *
 * Description:
 *   Checks for blank characters.  In the "C" and "POSIX" locales,
 *   these are: space and horizontal tab ('\t').
 *
 ****************************************************************************/

#define isblank(c)   ((c) == ' ' || (c) == '\t')

/****************************************************************************
 * Name: isspace
 *
 * Description:
 *   Checks  for  white-space characters.  In the "C" and "POSIX" locales,
 *   these are: space, form-feed ('\f'), newline ('\n'), carriage return
 *   ('\r'), horizontal tab ('\t'), and vertical tab ('\v').
 *
 ****************************************************************************/

#define isspace(c) \
  ((c) == ' '  || (c) == '\t' || (c) == '\n' || \
   (c) == '\r' || (c) == '\f' || (c) == '\v')

/****************************************************************************
 * Name: isascii
 *
 * Description:
 *   Checks whether c is a 7-bit unsigned char value that fits into the
 *   ASCII character set.
 *
 ****************************************************************************/

#define isascii(c)   ((c) >= 0 && (c) <= 0x7f)

/****************************************************************************
 * Name: isprint
 *
 * Description:
 *   Checks for a printable character (including space)
 *
 ****************************************************************************/

#define isprint(c)   ((c) >= 0x20 && (c) < 0x7f)

/****************************************************************************
 * Name: isgraph
 *
 * Description:
 *   Checks for a printable character (excluding space)
 *
 ****************************************************************************/

#define isgraph(c)   ((c) > 0x20 && (c) < 0x7f)

/****************************************************************************
 * Name: iscntrl
 *
 * Description:
 *   Checks for control character.
 *
 ****************************************************************************/

#define iscntrl(c) (!isprint(c))

/****************************************************************************
 * Name: islower
 *
 * Description:
 *   Checks for an lowercase letter.
 *
 ****************************************************************************/

#define islower(c)   ((c) >= 'a' && (c) <= 'z')

/****************************************************************************
 * Name: isupper
 *
 * Description:
 *   Checks for an uppercase letter.
 *
 ****************************************************************************/

#define isupper(c)   ((c) >= 'A' && (c) <= 'Z')

/****************************************************************************
 * Name: isalpha
 *
 * Description:
 *   Checks for an alphabetic character
 *
 ****************************************************************************/

#define isalpha(c)   (islower(c) || isupper(c))

/****************************************************************************
 * Name: isdigit
 *
 * Description:
 *   ANSI standard isdigit implementation.
 *
 ****************************************************************************/

#define isdigit(c)   ((c) >= '0' && (c) <= '9')

/****************************************************************************
 * Name: isalnum
 *
 * Description:
 *   Checks for an alphanumeric character
 *
 ****************************************************************************/

#define isalnum(c)   (isalpha(c) || isdigit(c))

/****************************************************************************
 * Name: ispunct
 *
 * Description:
 *   Checks for a printable character which is not a space or an
 *   alphanumeric character
 *
 ****************************************************************************/

#define ispunct(c)   (isgraph(c) && !isalnum(c))

/****************************************************************************
 * Name: isxdigit
 *
 * Description:
 *   isxdigit() checks for a hexadecimal digits, i.e. one of {0-9,a-f,A-F}
 *
 ****************************************************************************/

#define isxdigit(c) \
  (((c) >= '0' && (c) <= '9') || \
   ((c) >= 'a' && (c) <= 'f') || \
   ((c) >= 'A' && (c) <= 'F'))

/****************************************************************************
 * Name: toupper
 *
 * Description:
 *   toupper() converts the letter c to upper case, if possible.
 *
 ****************************************************************************/

#define toupper(c) \
  (((c) >= 'a' && (c) <= 'z') ? ((c) - 'a' + 'A') : (c))

/****************************************************************************
 * Name: tolower
 *
 * Description:
 *   tolower() converts the letter c to lower case, if possible.
 *
 ****************************************************************************/

#define tolower(c) \
  (((c) >= 'A' && (c) <= 'Z') ? ((c) - 'A' + 'a') : (c))

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#else /* __cplusplus */

/* When C++, define the same as above but as functions instead of macro's. */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

inline int islower(int c) { return c >= 'a' && c <= 'z'; }
inline int isupper(int c) { return c >= 'A' && c <= 'Z'; }
inline int isdigit(int c) { return c >= '0' && c <= '9'; }
inline int isalpha(int c) { return islower(c) || isupper(c); }
inline int isalnum(int c) { return isalpha(c) || isdigit(c); }
inline int isprint(int c) { return c >= 0x20 && c < 0x7f; }
inline int iscntrl(int c) { return c >= 0 && !isprint(c); }
inline int isgraph(int c) { return c > 0x20 && c < 0x7f; }
inline int ispunct(int c) { return isgraph(c) && !isalnum(c); }
inline int isblank(int c) { return c == ' ' || c == '\t'; }
inline int isspace(int c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f' || c == '\v'; }
inline int isxdigit(int c) { return isdigit(c) || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'); }
inline int tolower(int c) { return isupper(c) ? c - 'A' + 'a' : c; }
inline int toupper(int c) { return islower(c) ? c - 'a' + 'A' : c; }

#endif /* __cplusplus */

#endif /* __INCLUDE_CTYPE_H */
