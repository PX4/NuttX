/****************************************************************************
 * pexec.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "keywords.h"
#include "pdefs.h"
#include "pinsn16.h"
#include "pfdefs.h"
#include "pxdefs.h"
#include "pedefs.h"

#include "paslib.h"
#include "pexec.h"

#ifdef CONFIG_HAVE_LIBM
#include <math.h>
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define PTRUE   ((ustack_t)-1)
#define PFALSE  ((ustack_t) 0)

/****************************************************************************
 * Macros
 ****************************************************************************/

/* Remove the value from the top of the stack */

#define POP(st, dest) \
  do { \
    dest = (st)->dstack.i[BTOISTACK((st)->sp)]; \
    (st)->sp -= BPERI; \
  } while (0)

/* Add the value to top of the stack */

#define PUSH(st, src) \
  do { \
    (st)->sp += BPERI; \
    (st)->dstack.i[BTOISTACK((st)->sp)] = src; \
  } while (0)

/* Return an rvalue for the (word) offset from the top of the stack */

#define TOS(st, off) \
  (st)->dstack.i[BTOISTACK((st)->sp)-(off)]

/* Save the src (word) at the dest (word) stack position */

#define PUTSTACK(st, src, dest)  \
  do { \
    (st)->dstack.i[BTOISTACK(dest)] = src; \
  } while (0)

/* Return an rvalue for the (word) from the absolute stack position */

#define GETSTACK(st, src) \
  (st)->dstack.i[BTOISTACK(src)]

/* Store a byte to an absolute (byte) stack position */

#define PUTBSTACK(st, src,dest) \
  do { \
    (st)->dstack.b[dest] = dest; \
  } while (0)

/* Return an rvalue for the absolute (byte) stack position */

#define GETBSTACK(st, src) \
  (st)->dstack.b[src]

/* Return the address for an absolute (byte) stack position. */

#define ATSTACK(st, src) \
  &(st)->dstack.b[src]

/* Discard n words from the top of the stack */

#define DISCARD(st, n) \
  do { \
    (st)->sp -= BPERI*(n); \
  } while (0)

/* Release a C string */

#define free_cstring(a) \
  free(a)

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

union fparg_u
{
  float64 f;
  uint16 hw[4];
};

typedef union fparg_u fparg_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint16   pexec_sysio(struct pexec_s *st, ubyte fileno, uint16 subfunc);
static uint16   pexec_libcall(struct pexec_s *st, ubyte fileno, uint16 subfunc);
static uint16   pexec_execfp(struct pexec_s *st, ubyte fpop);
static void     pexec_getfparguments(struct pexec_s *st, ubyte fpop, fparg_t *arg1, fparg_t *arg2);
static ustack_t pexec_readinteger(ubyte *ioptr);
static void     pexec_readreal(uint16 *dest, ubyte *ioptr);
static ustack_t pexec_getbaseaddress(struct pexec_s *st, level_t leveloffset);
static ubyte   *pexec_mkcstring(ubyte *buffer, int buflen);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

static ubyte ioline[LINE_SIZE+1];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pexec_sysio
 *
 * Description:
 *   This function process a system I/O operation.
 *
 ****************************************************************************/

static uint16 pexec_sysio(struct pexec_s *st, ubyte fileno, uint16 subfunc)
{
  ustack_t uparm1;
  fparg_t  fp;

  ubyte *ptr;

  switch (subfunc)
    {
    case xEOF :
/* FINISH ME -- > */
      break;
    case xEOLN :
/* FINISH ME -- > */
      break;
    case xRESET :
/* FINISH ME -- > */
      break;
    case xREWRITE :
/* FINISH ME -- > */
      break;

    case xREADLN :
/* FINISH ME -- > */
      break;
    case xREAD_BINARY :
/* FINISH ME -- > */
      break;

      /* xREAD_INT:
       * STACK INPUTS: TOS(st, 0) = address to store integer */
    case xREAD_INT :
      (void)fgets((char*)ioline, LINE_SIZE, stdin);
      PUTSTACK(st, pexec_readinteger(ioline),TOS(st, 0));
      break;

      /* xREAD_CHAR:
       * STACK INPUTS: TOS(st, 0) = address to store integer */

    case xREAD_CHAR:
      (void)fgets((char*)ioline, LINE_SIZE, stdin);
      PUTBSTACK(st, ioline[0],TOS(st, 0));
      break;

      /* XREAD_STRING:

       * STACK INPUTS:
       *   TOS = Number of bytes to read
       *   TOS-1 = Address to store byte(s) */
    case xREAD_STRING :
      (void)fgets((char*)ATSTACK(st, TOS(st, 1)), TOS(st, 0), stdin);
      break;

      /* xREAD_REAL:
       * STACK INPUTS: TOS = address to store REAL */

    case xREAD_REAL :
      (void)fgets((char*)ioline, LINE_SIZE, stdin);
      pexec_readreal((uint16*)ATSTACK(st, TOS(st, 0)), ioline);
      break;

    case xWRITELN :
      putchar('\n');
      break;
    case xWRITE_PAGE :
      putchar('\f');
      break;
    case xWRITE_BINARY :
/* FINISH ME -- > */
      break;

      /* xWRITE_INT:
       * STACK INPUTS: TOS = integer value to write. */

    case xWRITE_INT :
      printf("%ld", signExtend16(TOS(st, 0)));
      break;

      /* xWRITE_CHAR:
       * STACK INPUTS: TOS = char value to write. */

    case xWRITE_CHAR :
      putchar(TOS(st, 0));
      break;

      /* xWRITE_STRING:
       * STACK INPUTS:
       *   TOS = Number of bytes to write
       *   TOS-1 = Address of src data */

    case xWRITE_STRING :
      uparm1 = TOS(st, 0);
      for (ptr = (ubyte*)ATSTACK(st, TOS(st, 1)); uparm1; uparm1--, ptr++)
	putchar(*ptr);
      break;

      /* xWRITE_REAL:
       * STACK INPUTS: TOS = value of float64 */

    case xWRITE_REAL :
      fp.hw[0] = TOS(st, 3);
      fp.hw[1] = TOS(st, 2);
      fp.hw[2] = TOS(st, 1);
      fp.hw[3] = TOS(st, 0);;
      printf("%f", fp.f);
      break;

    default :
      return eBADSYSIOFUNC;

    } /* end switch */

  return eNOERROR;

} /* end pexec_sysio */

/****************************************************************************
 * Name: pexec_libcall
 *
 * Description:
 *   This function process a system I/O operation 
 *
 ****************************************************************************/

static uint16 pexec_libcall(struct pexec_s *st, ubyte fileno, uint16 subfunc)
{
  ustack_t  uparm1;
  ustack_t  uparm2;
  addr_t    addr1;
  addr_t    addr2;
  uint16   *tmp;
  uint16   *ref;
  ubyte    *src;
  ubyte    *dest;
  ubyte    *name;
  int       len;
  int       value;

  switch (subfunc)
    {
      /* Get the value of an environment string
       *
       * ON INPUT:
       *   TOS(st, 0) = Number of bytes in environment identifier string
       *   TOS(st, 1) = Address environment identifier string
       * ON RETURN (above replaced with):
       *   TOS(st, 0) = MS 16-bits of 32-bit C string pointer
       *   TOS(st, 1) = LS 16-bits of 32-bit C string pointer
       */

    case lbGETENV :
      len = TOS(st, 0);                    /* Number of bytes in string */
      src = (ubyte*)&GETSTACK(st, TOS(st, 1));  /* Pointer to string */
		       
      /* Make a C string out of the pascal string */

      name = pexec_mkcstring(src, len);
      if (name == NULL)
	{
	  return eNOMEMORY;
	}

      /* Make the C-library call and free the string copy */

      src = (ubyte*)getenv((char*)name);
      free_cstring(name);

      /* Save the returned pointer in the stack */

      TOS(st, 0) = (ustack_t)((uint32)src >> 16);
      TOS(st, 1) = (ustack_t)((uint32)src & 0x0000ffff);
      break;

      /* Copy pascal string to a pascal string
       * 
       * ON INPUT:
       *   TOS(st, 0) = address of dest string hdr
       *   TOS(st, 1) = length of source string
       *   TOS(st, 2) = pointer to source string
       * ON RETURN (input consumed):
       */

    case lbSTR2STR :
      /* "Pop" in the input parameters from the stack */

      POP(st, addr1);  /* addr of dest string header */
      POP(st, uparm1); /* length of source data */
      POP(st, addr2);  /* addr of source string data */

      /* Do nothing if the source and destinations are the same
       * string.  This happens normally on cases like:
       *   string name;
       *   char   c;
       *   name := name + c;
       */

      if (addr1 != addr2)
	{
	  /* The source and destination strings are different.
	   * Make sure that the string length will fit into the destination.
	   */

	  if (uparm1 >= sSTRING_MAX_SIZE)
	    {
	      /* Clip to the maximum size */

	      uparm1 = sSTRING_MAX_SIZE;
	      len    = sSTRING_MAX_SIZE;
	    }
	  else
	    {
	      /* We have space */

	      len = (int)uparm1;
	    }

	  /* Get proper string pointers */

	  dest = ATSTACK(st, addr1);
	  src  = ATSTACK(st, addr2);

	  /* Transfer the (16-bit) string length (must be aligned!) */

	  tmp    = (uint16*)dest;
	  *tmp++ = uparm1;
	  dest   = (ubyte*)tmp;

	  /* Then transfer the string contents */

	  memcpy(dest, src, len);
	}
      break;

      /* Copy C string to a pascal string
       * 
       * ON INPUT:
       *   TOS(st, 0) = address of dest hdr
       *   TOS(st, 1) = MS 16-bits of 32-bit C string pointer
       *   TOS(st, 2) = LS 16-bits of 32-bit C string pointer
       * ON RETURN (input consumed):
       */
    case lbCSTR2STR :
      /* "Pop" in the input parameters from the stack */

      POP(st, addr1);  /* addr of dest string header */
      POP(st, uparm1); /* MS 16-bits of 32-bit C string pointer */
      POP(st, uparm2); /* LS 16-bits of 32-bit C string pointer */

      /* Get proper string pointers */

      dest = ATSTACK(st, addr1);
      src  = (ubyte*)((unsigned long)uparm1 << 16 | (unsigned long)uparm2);

      /* Handle null src pointer */

      if (src == NULL)
	{
	  *dest = 0;
	}
      else
	{
	  /* Get the length of the string */

	  uparm1 = strlen((char*)src);

	  /* Make sure that the string length will fit into the
	   * destination. */

	  if (uparm1 >= sSTRING_MAX_SIZE)
	    {
	      /* Clip to the maximum size */

	      uparm1 = sSTRING_MAX_SIZE;
	      len    = sSTRING_MAX_SIZE;
	    }
	  else
	    {
	      /* We have space */

	      len = (int)uparm1;
	    }

	  /* Transfer the (16-bit) string length (must be aligned!) */

	  tmp    = (uint16*)dest;
	  *tmp++ = uparm1;
	  dest   = (ubyte*)tmp;

	  /* Then transfer the string contents */

	  memcpy(dest, src, len);
	}
      break;

      /* Copy pascal string to a pascal string reference
       *   procedure str2rstr(src : string; var dest : rstring)
       * ON INPUT:
       *   TOS(st, 0)=address of dest string reference
       *   TOS(st, 1)=length of source string
       *   TOS(st, 2)=pointer to source string
       * ON RETURN: actual parameters released.
       */

    case lbSTR2RSTR :
      /* "Pop" in the input parameters from the stack */

      POP(st, addr1);  /* addr of dest string reference */
      POP(st, uparm1); /* length of source data */
      POP(st, addr2);  /* addr of source string data */

      /* Make sure that the string length will fit into the destination. */

      if (uparm1 >= sSTRING_MAX_SIZE)
	{
	  return eSTRSTKOVERFLOW;
	}

      /* Get a pointer to the destination reference */

      ref = (uint16*)ATSTACK(st, addr1);

      /* Get proper string pointers */

      dest = ATSTACK(st, ref[0] - 2);
      src  = ATSTACK(st, addr2);

      /* Transfer the (16-bit) string length (must be aligned!) */

      tmp    = (uint16*)dest;
      *tmp++ = uparm1;
      dest   = (ubyte*)tmp;

      /* Then transfer the string contents and save the new size */

      memcpy(dest, src, uparm1);
      ref[1] = uparm1;
      break;

      /* Copy C string to a pascal string reference
       *   procedure cstr2str(src : cstring; var dest : string)
       * ON INPUT:
       *   TOS(st, 0)=address of dest string reference
       *   TOS(st, 0)=MS 16-bits of 32-bit C source string pointer
       *   TOS(st, 1)=LS 16-bits of 32-bit C source string pointer
       * ON RETURN: actual parameters released
       */

    case lbCSTR2RSTR :
      /* "Pop" in the input parameters from the stack */

      POP(st, addr1);  /* addr of dest string reference */
      POP(st, uparm1); /* MS 16-bits of 32-bit C string pointer */
      POP(st, uparm2); /* LS 16-bits of 32-bit C string pointer */

      /* Get a pointer to the destination reference */

      ref = (uint16*)ATSTACK(st, addr1);

      /* Get proper string pointers */

      dest = ATSTACK(st, ref[0] - 2);
      src  = (ubyte*)((unsigned long)uparm1 << 16 | (unsigned long)uparm2);

      /* Handle null src pointer */

      if (src == NULL)
	{
	  *dest = 0;
	}
      else
	{
	  /* Get the length of the string */

	  uparm1 = strlen((char*)src);

	  /* Make sure that the string length will fit into the
	   * destination. */

	  if (uparm1 >= sSTRING_MAX_SIZE)
	    {
	      return eSTRSTKOVERFLOW;
	    }

	  /* Transfer the (16-bit) string length (must be aligned!) */

	  tmp    = (uint16*)dest;
	  *tmp++ = uparm1;
	  dest   = (ubyte*)tmp;

	  /* Then transfer the string contents */

	  memcpy(dest, src, uparm1);
	  ref[1] = uparm1;
	}
      break;

      /* Convert a string to a numeric value
       *   procedure val(const s : string; var v; var code : word); 
       *
       * Description:
       * val() converts the value represented in the string S to a numerical
       * value, and stores this value in the variable V, which can be of type
       * Longint, Real and Byte. If the conversion isn't succesfull, then the
       * parameter Code contains the index of the character in S which
       * prevented the conversion. The string S is allowed to contain spaces
       * in the beginning.
       *
       * The string S can contain a number in decimal, hexadecimal, binary or
       * octal format, as described in the language reference.
       *
       * Errors:
       * If the conversion doesn¡Çt succeed, the value of Code indicates the
       * position where the conversion went wrong.
       *
       * ON INPUT
       *   TOS(st, 0)=address of code
       *   TOS(st, 1)=address of value
       *   TOS(st, 2)=length of source string
       *   TOS(st, 3)=pointer to source string
       * ON RETURN: actual parameters released
       */

    case lbVAL :
      /* Get the string information */

      len = TOS(st, 2);                    /* Number of bytes in string */
      src = (ubyte*)&GETSTACK(st, TOS(st, 3));  /* Pointer to string */
		       
      /* Make a C string out of the pascal string */

      name = pexec_mkcstring(src, len);
      if (name == NULL)
	{
	  return eNOMEMORY;
	}

      /* Convert the string to an integer */

      value = atoi((char*)name);
      if ((value < MININT) || (value > MAXINT))
	{
	  return eINTEGEROVERFLOW;
	}
      PUTSTACK(st, TOS(st, 0), 0);
      PUTSTACK(st, TOS(st, 1), value);
      DISCARD(st, 4);
      break;

      /* Create an empty string
       *   function mkstk : string;
       * ON INPUT
       * ON RETURN
       *   TOS(st, 0)=length of new string
       *   TOS(st, 1)=pointer to new string
       */

    case lbMKSTK :
      /* Allocate space on the string stack for the new string
       * FIXME:  This logic does not handle strings with other than the
       * default size!
       */

      addr1             = ((st->csp + 1) & ~1);
      st->csp += sSTRING_SIZE;    /* Allocate max size */

      /* Save the length at the beginning of the copy */

      tmp    = (uint16*)&GETSTACK(st, addr1);  /* Pointer to new string */
      *tmp++ = 0;                          /* Save current size */

      /* Update the stack content */

      PUSH(st, addr1 + sSTRING_HDR_SIZE);      /* Pointer to new string */
      PUSH(st, 0);                             /* Current size */
      break;

      /* Replace a string with a duplicate string residing in allocated
       * string stack.
       *   function mkstkstr(name : string) : string;
       * ON INPUT
       *   TOS(st, 0)=length of original string
       *   TOS(st, 1)=pointer to original string data
       * ON RETURN
       *   TOS(st, 0)=length of new string (unchanged)
       *   TOS(st, 1)=pointer to new string data
       */

    case lbMKSTKSTR :
      /* Get the parameters from the stack (leaving the string reference
       * in place.
       */

      uparm1 = TOS(st, 0);     /* Original string size */
      addr1  = TOS(st, 1);     /* Original string data pointer */
 
      /* Check if there is space on the string stack for the new string
       * FIXME:  This logic does not handle strings with other than the
       * default size!
       */

      if (st->csp + sSTRING_SIZE >= st->spb)
	{
	  return eSTRSTKOVERFLOW;
	}

      /* Allocate space on the string stack for the new string */

      addr2             = ((st->csp + 1) & ~1);
      st->csp += sSTRING_SIZE;    /* Allocate max size */

      /* Save the length at the beginning of the copy */

      tmp    = (uint16*)&GETSTACK(st, addr2);  /* Pointer to new string */
      *tmp++ = uparm1;                     /* Save current size */
      dest   = (ubyte*)tmp;                 /* Pointer to string data */

      /* Copy the string into the string stack */

      src  = (ubyte*)&GETSTACK(st, addr1);  /* Pointer to original string */
      memcpy(dest, src, uparm1);

      /* Update the stack content */

      TOS(st, 1) = addr2 + sSTRING_HDR_SIZE;
      break;

      /* Replace a character with a string residing in allocated string stack.
       *   function mkstkc(c : char) : string;
       * ON INPUT
       *   TOS(st, 0)=Character value
       * ON RETURN
       *   TOS(st, 0)=length of new string
       *   TOS(st, 1)=pointer to new string
       */

    case lbMKSTKC :
      /* Check if there is space on the string stack for the new string
       * FIXME:  This logic does not handle strings with other than the
       * default size!
       */

      if (st->csp + sSTRING_SIZE >= st->spb)
	{
	  return eSTRSTKOVERFLOW;
	}

      /* Allocate space on the string stack for the new string */

      addr2             = ((st->csp + 1) & ~1);
      st->csp += sSTRING_SIZE;    /* Allocate max size */

      /* Save the length at the beginning of the copy */

      tmp    = (uint16*)&GETSTACK(st, addr2);  /* Pointer to new string */
      *tmp++ = 1;                          /* Save initial size */
      dest   = (ubyte*)tmp;                 /* Pointer to string data */

      /* Copy the character into the string stack */

      *dest++ = TOS(st, 0);                    /* Save character as string */

      /* Update the stack content */

      TOS(st, 0) = addr2 + sSTRING_HDR_SIZE;   /* String address */
      PUSH(st, 1);                             /* String length */
      break;

      /* Concatenate a string to the end of a string.
       *   function strcat(name : string, c : char) : string;
       *
       * ON INPUT
       *   TOS(st, 0)=length of string1
       *   TOS(st, 1)=pointer to string1 data
       *   TOS(st, 2)=length of string2
       *   TOS(st, 3)=pointer to string2 data
       * ON OUTPUT
       *   TOS(st, 1)=new length of string2
       *   TOS(st, 2)=pointer to string2
       */

    case lbSTRCAT :
      /* Get the parameters from the stack (leaving the string reference
       * in place.
       */

      POP(st, uparm1);      /* string1 size */
      POP(st, addr1);       /* string1 data stack addr */
      uparm2 = TOS(st, 0);  /* string2 size */

      /* Check for string overflow.  FIXME:  This logic does not handle
       * strings with other than the default size!
       */

      if (uparm1 + uparm2 > sSTRING_MAX_SIZE)
	  return eSTRSTKOVERFLOW;
      else
	{
	  /* Get a pointer to string1 data */

	  src    = ATSTACK(st, addr1);

	  /* Get a pointer to string2 header, set new size then, get
	   * a pointer to string2 data.
	   */

	  tmp    = ((uint16*)&GETSTACK(st, TOS(st, 1))) - 1;
	  *tmp++ = uparm1 + uparm2;
	  dest   = (ubyte*)tmp;

	  memcpy(&dest[uparm2], src, uparm1); /* cat strings */
	  TOS(st, 0) = uparm1 + uparm2;           /* Save new size */
	}
      break;

      /* Concatenate a character  to the end of a string.
       *   function strcatc(name : string, c : char) : string;
       *
       * ON INPUT
       *   TOS(st, 0)=character to concatenate
       *   TOS(st, 1)=length of string
       *   TOS(st, 2)=pointer to string
       * ON OUTPUT
       *   TOS(st, 1)=new length of string
       *   TOS(st, 2)=pointer to string
       */

    case lbSTRCATC :
      /* Get the parameters from the stack (leaving the string reference
       * in place.
       */

      POP(st, uparm1);      /* Character to concatenate */
      uparm2 = TOS(st, 0);  /* Current length of string */

      /* Check for string overflow.  FIXME:  This logic does not handle
       * strings with other than the default size!
       */

      if (uparm2 >= sSTRING_MAX_SIZE)
	  return eSTRSTKOVERFLOW;
      else
	{
	  /* Get a pointer to string header, set size new size then, get
	   * a pointer to string data.
	   */

	  tmp          = ((uint16*)&GETSTACK(st, TOS(st, 1))) - 1;
	  *tmp++       = uparm2 + 1;
	  dest         = (ubyte*)tmp;

	  /* Add the new charcter */

	  dest[uparm2] = (ubyte)uparm1;

	  /* Save the new string size */

	  TOS(st, 0)       = uparm2 + 1;
	}
      break;

      /* Compare two pascal strings
       *   function strcmp(name1 : string, name2 : string) : integer;
       * ON INPUT
       *   TOS(st, 1)=length of string2
       *   TOS(st, 2)=address of string2 data
       *   TOS(st, 3)=length of string1
       *   TOS(st, 4)=address of string1 data
       * ON OUTPUT
       *   TOS(st, 0)=(-1=less than, 0=equal, 1=greater than} 
       */

    case lbSTRCMP :
      {
	int result;

	/* Get the parameters from the stack (leaving space for the
	 * return value);
	 */

	POP(st, uparm2);     /* length of string2 */
	POP(st, addr2);      /* address of string2 data */
	POP(st, uparm1);     /* length of string1 */
	addr1 = TOS(st, 0);  /* address of string1 data */

	/* Get full address */

	dest   = ATSTACK(st, addr1);
	src    = ATSTACK(st, addr2);

	/* If name1 is shorter than name2, then we can only return
	 * -1 (less than) or +1 greater than.  If the substrings
	 * of length of name1 are equal, then we return less than.
	 */

	if (uparm1 < uparm2)
	  {
	    result = memcmp(dest, src, uparm1);
	    if (result == 0) result = -1;
	  }

	/* If name1 is longer than name2, then we can only return
	 * -1 (less than) or +1 greater than.  If the substrings
	 * of length of name2 are equal, then we return greater than.
	 */

	else if (uparm1 > uparm2)
	  {
	    result = memcmp(dest, src, uparm2);
	    if (result == 0) result = 1;
	  }

	/* The strings are of equal length. Return the result of
	 * the comparison.
	 */

	else
	  {
	    result = memcmp(dest, src, uparm1);
	  }
	TOS(st, 0) = result;
      }
      break;

    default :
      return eBADSYSLIBCALL;

    } /* end switch */

  return eNOERROR;

} /* end pexec_libcall */

/****************************************************************************
 * Name: pexec_execfp
 *
 * Description:
 *   This function processes a floating point operation.
 *
 ****************************************************************************/

static uint16 pexec_execfp(struct pexec_s *st, ubyte fpop)
{
  sint16 intValue;
  fparg_t arg1;
  fparg_t arg2;
  fparg_t result;

  switch (fpop & fpMASK)
    {
      /* Floating Pointer Conversions (On stack argument:  FP or Integer) */

    case fpFLOAT :
      POP(st, intValue);
      result.f = (float64)intValue;
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;

    case fpTRUNC :
    case fpROUND :
      pexec_getfparguments(st, fpop, &arg1, NULL);
      intValue = (sint16)arg1.f;
      PUSH(st, intValue);
      break;

      /* Floating Point arithmetic instructions (Two FP stack arguments) */

    case fpADD :
      pexec_getfparguments(st, fpop, &arg1, &arg2);
      result.f = arg1.f + arg2.f;
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
    case fpSUB :
      pexec_getfparguments(st, fpop, &arg1, &arg2);
      result.f = arg1.f - arg2.f;
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
    case fpMUL :
      pexec_getfparguments(st, fpop, &arg1, &arg2);
      result.f = arg1.f * arg2.f;
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
    case fpDIV :
      pexec_getfparguments(st, fpop, &arg1, &arg2);
      result.f = arg1.f / arg2.f;
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
    case fpMOD :
      return eBADFPOPCODE;
#if 0 /* Not yet */
      pexec_getfparguments(st, fpop, &arg1, &arg2);
      result.f = arg1.f % arg2.f;
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
#endif

      /* Floating Point Comparisons (Two FP stack arguments) */

    case fpEQU :
      pexec_getfparguments(st, fpop, &arg1, &arg2);
      intValue = PFALSE;
      if (arg1.f == arg2.f)
	intValue = PTRUE;
      PUSH(st, intValue);
      break;
    case fpNEQ :
      pexec_getfparguments(st, fpop, &arg1, &arg2);
      intValue = PFALSE;
      if (arg1.f != arg2.f)
	intValue = PTRUE;
      PUSH(st, intValue);
      break;
    case fpLT :
      pexec_getfparguments(st, fpop, &arg1, &arg2);
      intValue = PFALSE;
      if (arg1.f < arg2.f)
	intValue = PTRUE;
      PUSH(st, intValue);
      break;
    case fpGTE :
      pexec_getfparguments(st, fpop, &arg1, &arg2);
      intValue = PFALSE;
      if (arg1.f >= arg2.f)
	intValue = PTRUE;
      PUSH(st, intValue);
      break;
    case fpGT :
      pexec_getfparguments(st, fpop, &arg1, &arg2);
      intValue = PFALSE;
      if (arg1.f > arg2.f)
	intValue = PTRUE;
      PUSH(st, intValue);
      break;
    case fpLTE :
      pexec_getfparguments(st, fpop, &arg1, &arg2);
      intValue = PFALSE;
      if (arg1.f <= arg2.f)
	intValue = PTRUE;
      PUSH(st, intValue);
      break;

      /* Floating Point arithmetic instructions (One FP stack arguments) */

    case fpNEG :
      pexec_getfparguments(st, fpop, &arg1, NULL);
      result.f = -arg1.f;
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
#ifdef CONFIG_HAVE_LIBM
    case fpABS :
      pexec_getfparguments(st, fpop, &arg1, NULL);
      result.f = fabs(arg1.f);
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
#endif
    case fpSQR :
      pexec_getfparguments(st, fpop, &arg1, NULL);
      result.f = arg1.f * arg1.f;
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
#ifdef CONFIG_HAVE_LIBM
    case fpSQRT :
      pexec_getfparguments(st, fpop, &arg1, NULL);
      result.f = sqrt(arg1.f);
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
    case fpSIN :
      pexec_getfparguments(st, fpop, &arg1, NULL);
      result.f = sin(arg1.f);
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
    case fpCOS :
      pexec_getfparguments(st, fpop, &arg1, NULL);
      result.f = cos(arg1.f);
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
    case fpATAN :
      pexec_getfparguments(st, fpop, &arg1, NULL);
      result.f = atan(arg1.f);
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
    case fpLN :
      pexec_getfparguments(st, fpop, &arg1, NULL);
      result.f = log(arg1.f);
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
    case fpEXP :
      pexec_getfparguments(st, fpop, &arg1, NULL);
      result.f = exp(arg1.f);
      PUSH(st, result.hw[0]);
      PUSH(st, result.hw[1]);
      PUSH(st, result.hw[2]);
      PUSH(st, result.hw[3]);
      break;
#endif

    default :
      return eBADFPOPCODE;

    } /* end switch */
  return eNOERROR;

} /* end pexec_execfp */

/****************************************************************************
 * Name: pexec_getfparguments
 *
 * Description:
 *   This function retrieves the floating point arguments and performs
 *   integer to REAL conversions as necessary
 *
 ****************************************************************************/

static void pexec_getfparguments(struct pexec_s *st, ubyte fpop, fparg_t *arg1, fparg_t *arg2)
{
  sint16 sparm;

  /* Extract arg2 from the stack */

  if (arg2)
    {
      /* Convert an integer argument to type REAL */

      if ((fpop & fpARG2) != 0)
	{
	  POP(st, sparm);
	  arg2->f = (float64)sparm;
	} /* end if */
      else
	{
	  POP(st, arg2->hw[3]);
	  POP(st, arg2->hw[2]);
	  POP(st, arg2->hw[1]);
	  POP(st, arg2->hw[0]);
	} /* end else */
    } /* end if */

  /* Extract arg1 from the stack */

  if (arg1)
    {
      /* Convert an integer argument to type REAL */

      if ((fpop & fpARG1) != 0)
	{
	  POP(st, sparm);
	  arg1->f = (float64)sparm;
	} /* end if */
      else
	{
	  POP(st, arg1->hw[3]);
	  POP(st, arg1->hw[2]);
	  POP(st, arg1->hw[1]);
	  POP(st, arg1->hw[0]);
	} /* end else */
    } /* end if */

} /* end pexec_getfparguments */

/****************************************************************************
 * Name: pexec_readinteger
 *
 * Description:
 *   This function parses a decimal integer from ioptr
 ****************************************************************************/

static ustack_t pexec_readinteger(ubyte *ioptr)
{
  sstack_t value = 0;

  while (isspace(*ioptr)) ioptr++;
  while ((*ioptr >= '0') && (*ioptr <= '9'))
    {
      value = 10*value
	+ (sstack_t)(*ioptr)
	- (sstack_t)'0';
      ioptr++;
    } /* end while */

  return (ustack_t)value;

} /* end pexec_readinteger */

/****************************************************************************
 * Name: pexec_readreal
 *
 * Description:
 *    This function parses a decimal integer from ioptr.
 *
 ****************************************************************************/

static void pexec_readreal(uint16 *dest, ubyte *inPtr)
{
  sint32    intpart;
  fparg_t   result;
  float64   fraction;
  ubyte     unaryop;

  intpart = 0;
  unaryop = '+';

  /* Check for a leading unary - */

  if ((*inPtr == '-') || (*inPtr == '+'))
    unaryop = *inPtr++;

  /* Get the integer part of the real */

  while ((*inPtr >= '0') && (*inPtr <= '9'))
    intpart = 10*intpart + ((sint32)*inPtr++) - ((sint32)'0');

  result.f = ((float64)intpart);

  /* Check for the a fractional part */

  if (*inPtr == '.')
    {
      inPtr++;
      fraction = 0.1;
      while ((*inPtr >= '0') && (*inPtr <= '9'))
	{
	  result.f += fraction * (float64)(((sint32)*inPtr++) - ((sint32)'0'));
	  fraction /= 10.0;
	} /* end while */
    } /* end if */

  /* Correct the sign of the result */

  if (unaryop == '-')
    result.f = -result.f;

  /* Return the value into the P-Machine stack */

  *dest++ = result.hw[0];
  *dest++ = result.hw[1];
  *dest++ = result.hw[2];
  *dest   = result.hw[3];

} /* end pexec_readreal */

/****************************************************************************
 * Name: pexec_getbaseaddress
 *
 * Description:
 *   This function binds the base address corresponding to a given level
 *   offset.
 *
 ****************************************************************************/

static ustack_t pexec_getbaseaddress(struct pexec_s *st, level_t leveloffset)
{
  /* Start with the base register of the current frame */

  ustack_t baseAddress = st->fp;

  /* Search backware "leveloffset" frames until the correct frame is
   * found
   */

   while (leveloffset > 0)
     {
       baseAddress = st->dstack.i[BTOISTACK(baseAddress)];
       leveloffset--;
     } /* end while */

   /* Offset that value by two words (one for the st->fp and one for the
    * return value
    */

   return baseAddress + 2*BPERI;

} /* end pexec_getbaseaddress */

/****************************************************************************
 * Name: pexec_mkcstring
 ****************************************************************************/

static ubyte *pexec_mkcstring(ubyte *buffer, int buflen)
{
  ubyte *string;

  string = malloc(buflen + 1);
  if (string != NULL)
    {
      memcpy(string, buffer, buflen);
      string[buflen] = '\0';
    }
  return string;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pexec_init
 ****************************************************************************/

FAR struct pexec_s *pexec_init(struct pexec_attr_s *attr)
{
  struct pexec_s *st;
  addr_t stacksize;
  addr_t adjusted_rosize;

  /* Allocate the p-machine state stucture */

  st = (struct pexec_s *)malloc(sizeof(struct pexec_s));
  if (!st)
    {
      return NULL;
    }

  /* Set up I-Space */

  st->ispace = attr->ispace;
  st->maxpc  = attr->maxpc;

  /* Align size of read-only data to 16-bit boundary. */

  adjusted_rosize = (attr->rosize + 1) & ~1;

  /* Allocate the pascal stack.  Organization is string stack, then
   * constant data, then "normal" pascal stack.
   */

  stacksize = attr->varsize + adjusted_rosize + attr->strsize;
  st->dstack.b = (ubyte*)malloc(stacksize);
  if (!st->dstack.b)
    {
      free(st);
      return NULL;
    }

  /* Copy the rodata into the stack */

  if (attr->rodata && attr->rosize)
    {
      memcpy(&st->dstack.b[attr->strsize], attr->rodata, attr->rosize);
    }

  /* Set up info needed to perform a simulated reset */

  st->strsize   = attr->strsize;
  st->rosize    = adjusted_rosize;
  st->entry     = attr->entry;
  st->stacksize = stacksize;

  /* Then perform a simulated reset */

  pexec_reset(st);
  return st;
}

/****************************************************************************
 * Name: pexec
 ****************************************************************************/

int pexec(FAR struct pexec_s *st)
{
  ubyte    opcode;
  ubyte    arg8;
  uint16   arg16;
  ubyte    opsize;
  sstack_t sparm1;
  sstack_t sparm2;
  ustack_t uparm1;
  ustack_t uparm2;
  ustack_t uparm3;

  /* Make sure that the program counter is within range */

  if (st->pc >= st->maxpc)
    return eBADPC;

  else
    {
      /* Get the instruction to execute */

      opcode = st->ispace[st->pc];
      arg8   = 0;
      arg16  = 0;
      opsize = 1;
      if ((opcode & o8) != 0)
	{
	  arg8 = st->ispace[st->pc + opsize];
	  opsize++;
	} /* end if */
      if ((opcode & o16) != 0)
	{
	  arg16  = ((st->ispace[st->pc + opsize]) << 8);
	  arg16 |= st->ispace[st->pc + opsize + 1];
	  opsize += 2;
	} /* end if */

      switch (opcode)
	{

/**---------------------------------------------------------------------
 OPCODES WITH NO ARGUMENTS
 ---------------------------------------------------------------------**/
          /* Arithmetic & logical & and integer conversions (One stack argument) */
	case oNEG  :
	  TOS(st, 0) = (ustack_t)(-(sstack_t)TOS(st, 0));
	  st->pc += opsize;
	  break;
	case oABS  :
	  if (signExtend16(TOS(st, 0)) < 0)
	    TOS(st, 0) = (ustack_t)(-signExtend16(TOS(st, 0)));
	  st->pc += opsize;
	  break;
	case oINC  :
	  TOS(st, 0)++;
	  st->pc += opsize;
	  break;
	case oDEC  :
	  TOS(st, 0)--;
	  st->pc += opsize;
	  break;
	case oNOT  :
	  TOS(st, 0) = ~TOS(st, 0);
	  st->pc += opsize;
	  break;

          /* Arithmetic & logical (Two stack arguments) */

	case oADD :
	  POP(st, sparm1);
	  TOS(st, 0) = (ustack_t)(((sstack_t)TOS(st, 0)) + sparm1);
	  st->pc += opsize;
	  break;
	case oSUB :
	  POP(st, sparm1);
	  TOS(st, 0) = (ustack_t)(((sstack_t)TOS(st, 0)) - sparm1);
	  st->pc += opsize;
	  break;
	case oMUL :
	  POP(st, sparm1);
	  TOS(st, 0) = (ustack_t)(((sstack_t)TOS(st, 0)) * sparm1);
	  st->pc += opsize;
	  break;
	case oDIV :
	  POP(st, sparm1);
	  TOS(st, 0) = (ustack_t)(((sstack_t)TOS(st, 0)) / sparm1);
	  st->pc += opsize;
	  break;
	case oMOD :
	  POP(st, sparm1);
	  TOS(st, 0) = (ustack_t)(((sstack_t)TOS(st, 0)) % sparm1);
	  st->pc += opsize;
	  break;
	case oSLL :
	  POP(st, sparm1);
	  TOS(st, 0) = (ustack_t)(((sstack_t)TOS(st, 0)) << sparm1);
	  st->pc += opsize;
	  break;
	case oSRL :
	  POP(st, sparm1);
	  TOS(st, 0) = (TOS(st, 0) >> sparm1);
	  st->pc += opsize;
	  break;
	case oSRA :
	  POP(st, sparm1);
	  TOS(st, 0) = (ustack_t)(((sstack_t)TOS(st, 0)) >> sparm1);
	  st->pc += opsize;
	  break;
	case oOR  :
	  POP(st, uparm1);
	  TOS(st, 0) = (TOS(st, 0) | uparm1);
	  st->pc += opsize;
	  break;
	case oAND :
	  POP(st, uparm1);
	  TOS(st, 0) = (TOS(st, 0) & uparm1);
	  st->pc += opsize;
	  break;
	case oBIT :
	  POP(st, uparm1);
	  uparm2 = TOS(st, 0);
	  if ((uparm1 & (1 << uparm2)) != 0)
	    TOS(st, 0) = PTRUE;
	  else
	    TOS(st, 0) = PFALSE;
	  st->pc += opsize;
	  break;

	  /* Comparisons (One stack argument) */

	case oEQUZ :
	  POP(st, sparm1);
	  uparm1 = PFALSE;
	  if (sparm1 == 0)
	    uparm1 = PTRUE;
	  PUSH(st, uparm1);
	  st->pc += opsize;
	  break;
	case oNEQZ :
	  POP(st, sparm1);
	  uparm1 = PFALSE;
	  if (sparm1 != 0)
	    uparm1 = PTRUE;
	  PUSH(st, uparm1);
	  st->pc += opsize;
	  break;
	case oLTZ  :
	  POP(st, sparm1);
	  uparm1 = PFALSE;
	  if (sparm1 < 0)
	    uparm1 = PTRUE;
	  PUSH(st, uparm1);
	  st->pc += opsize;
	  break;
	case oGTEZ :
	  POP(st, sparm1);
	  uparm1 = PFALSE;
	  if (sparm1 >= 0)
	    uparm1 = PTRUE;
	  PUSH(st, uparm1);
	  st->pc += opsize;
	  break;
	case oGTZ  :
	  POP(st, sparm1);
	  uparm1 = PFALSE;
	  if (sparm1 > 0)
	    uparm1 = PTRUE;
	  PUSH(st, uparm1);
	  st->pc += opsize;
	  break;
	case oLTEZ :
	  POP(st, sparm1);
	  uparm1 = PFALSE;
	  if (sparm1 <= 0)
	    uparm1 = PTRUE;
	  PUSH(st, uparm1);
	  st->pc += opsize;
	  break;

	  /* Comparisons (Two stack arguments) */

	case oEQU  :
	  POP(st, sparm1);
	  uparm1 = PFALSE;
	  if (sparm1 == (sstack_t)TOS(st, 0))
	    uparm1 = PTRUE;
	  TOS(st, 0) = uparm1;
	  st->pc += opsize;
	  break;
	case oNEQ  :
	  POP(st, sparm1);
	  uparm1 = PFALSE;
	  if (sparm1 != (sstack_t)TOS(st, 0))
	    uparm1 = PTRUE;
	  TOS(st, 0) = uparm1;
	  st->pc += opsize;
	  break;
	case oLT   :
	  POP(st, sparm1);
	  uparm1 = PFALSE;
	  if (sparm1 < (sstack_t)TOS(st, 0))
	    uparm1 = PTRUE;
	  TOS(st, 0) = uparm1;
	  st->pc += opsize;
	  break;
	case oGTE  :
	  POP(st, sparm1);
	  uparm1 = PFALSE;
	  if (sparm1 >= (sstack_t)TOS(st, 0))
	    uparm1 = PTRUE;
	  TOS(st, 0) = uparm1;
	  st->pc += opsize;
	  break;
	case oGT   :
	  POP(st, sparm1);
	  uparm1 = PFALSE;
	  if (sparm1 > (sstack_t)TOS(st, 0))
	    uparm1 = PTRUE;
	  TOS(st, 0) = uparm1;
	  st->pc += opsize;
	  break;
	case oLTE  :
	  POP(st, sparm1);
	  uparm1 = PFALSE;
	  if (sparm1 <= (sstack_t)TOS(st, 0))
	    uparm1 = PTRUE;
	  TOS(st, 0) = uparm1;
	  st->pc += opsize;
	  break;

	  /* Load (One stack argument) */

	case oLDI  :
	  POP(st, uparm1);                   /* Address */
	  PUSH(st, GETSTACK(st, uparm1));
	  PUSH(st, GETSTACK(st, uparm1 + BPERI));
	  st->pc += opsize;
	  break;
	case oLDIH  :
	  TOS(st, 0) = GETSTACK(st, TOS(st, 0));
	  st->pc += opsize;
	  break;
	case oLDIB :
	  TOS(st, 0) = GETBSTACK(st, TOS(st, 0));
	  st->pc += opsize;
	  break;
	case oLDIM :
 /* FIX ME --> Need to handle the unaligned case */
	  POP(st, uparm1); /* Size */
	  POP(st, uparm2); /* Stack offset */
	  while (uparm1 > 0)
	    {
	      if (uparm1 >= BPERI)
		{
		  PUSH(st, GETSTACK(st, uparm2));
		  uparm2 += BPERI;
		  uparm1 -= BPERI;
		} /* end if */
	      else
		{
		  PUSH(st, GETBSTACK(st, uparm2));
		  uparm2++;
		  uparm1--;
		} /* end else */
            } /* end while */
	  st->pc += opsize;
	  break;
	case oDUP :
	  uparm1 = TOS(st, 0);
	  uparm2 = TOS(st, 1);
	  PUSH(st, uparm2);
	  PUSH(st, uparm1);
	  st->pc += opsize;
	  break;
	case oDUPH :
	  uparm1 = TOS(st, 0);
	  PUSH(st, uparm1);
	  st->pc += opsize;
	  break;
	case oPUSHS :
	  PUSH(st, st->csp);
	  st->pc += opsize;
	  break;
	case oPOPS :
	  POP(st, st->csp);
	  st->pc += opsize;
	  break;

	  /* Store (Two stack arguments) */

	case oSTIH  :
	  POP(st, uparm1);
	  POP(st, uparm2);
	  PUTSTACK(st, uparm1,uparm2);
	  st->pc += opsize;
	  break;
	case oSTIB :
	  POP(st, uparm1);
	  POP(st, uparm2);
	  PUTBSTACK(st, uparm1, uparm2);
	  st->pc += opsize;
	  break;
	case oSTIM :
 /* FIX ME --> Need to handle the unaligned case */
	  POP(st, uparm1);                /* Size in bytes */
          uparm3 = uparm1;            /* Save for stack discard */
	  sparm1 = ROUNDBTOI(uparm1); /* Size in words */
	  uparm2 = TOS(st, sparm1);       /* Stack offset */
	  sparm1--;
	  while (uparm1 > 0)
	    {
	      if (uparm1 >= BPERI)
		{
		  PUTSTACK(st, TOS(st, sparm1), uparm2);
		  uparm2 += BPERI;
		  uparm1 -= BPERI;
		  sparm1--;
		} /* end if */
	      else
		{
		  PUTBSTACK(st, TOS(st, sparm1), uparm2);
		  uparm2++;
		  uparm1--;
		} /* end else */
	    } /* end while */

	  /* Discard the stored data + the stack offset */

	  DISCARD(st, (ROUNDBTOI(uparm3) + 1));
	  st->pc += opsize;
	  break;

	  /* Program control (No stack arguments) */

	case oNOP   :
	  st->pc += opsize;
	  break;
	case oRET   :
	  POP(st, st->pc);
	  POP(st, st->fp);
	  DISCARD(st, 1);
	  break;

	  /* System Functions (No stack arguments) */

	case oEND   :
	  return eEXIT;

/**---------------------------------------------------------------------
 OPCODES WITH SINGLE BYTE ARGUMENT (arg8)
 ---------------------------------------------------------------------**/
	  /* Data stack:  arg8 = 8 bit unsigned data (no stack arguments) */

	case oPUSHB  :
	  PUSH(st, arg8);
	  st->pc += opsize;
	  break;

	  /* Floating Point:  arg8 = FP op-code (varying number of stack arguments) */
	case oFLOAT  :
	  st->pc += opsize;
	  return pexec_execfp(st, arg8);

/**---------------------------------------------------------------------
 OPCODES WITH SINGLE 16-BIT ARGUMENT (arg16)
 ---------------------------------------------------------------------**/
	  /* Program control:  arg16 = unsigned label (no stack arguments) */

	case oJMP   :
	  st->pc = (addr_t)arg16;
	  break;

	  /* Program control:  arg16 = unsigned label (One stack argument) */

	case oJEQUZ :
	  POP(st, sparm1);
	  if (sparm1 == 0)
	    st->pc = (addr_t)arg16;
	  else
	    st->pc += opsize;
	  break;
	case oJNEQZ :
	  POP(st, sparm1);
	  if (sparm1 != 0)
	    st->pc = (addr_t)arg16;
	  else
	    st->pc += opsize;
	  break;
	case oJLTZ  :
	  POP(st, sparm1);
	  if (sparm1 < 0)
	    st->pc = (addr_t)arg16;
	  else
	    st->pc += opsize;
	  break;
	case oJGTEZ :
	  POP(st, sparm1);
	  if (sparm1 >= 0)
	    st->pc = (addr_t)arg16;
	  else
	    st->pc += opsize;
	  break;
	case oJGTZ  :
	  POP(st, sparm1);
	  if (sparm1 > 0)
	    st->pc = (addr_t)arg16;
	  else
	    st->pc += opsize;
	  break;
	case oJLTEZ :
	  POP(st, sparm1);
	  if (sparm1 <= 0)
	    st->pc = (addr_t)arg16;
	  else
	    st->pc += opsize;
	  break;

	  /* Program control:  arg16 = unsigned label (Two stack arguments) */

	case oJEQU :
	  POP(st, sparm1);
	  POP(st, sparm2);
	  if (sparm2 == sparm1)
	    st->pc = (addr_t)arg16;
	  else
	    st->pc += opsize;
	  break;
	case oJNEQ :
	  POP(st, sparm1);
	  POP(st, sparm2);
	  if (sparm2 != sparm1)
	    st->pc = (addr_t)arg16;
	  else
	    st->pc += opsize;
	  break;
	case oJLT  :
	  POP(st, sparm1);
	  POP(st, sparm2);
	  if (sparm2 < sparm1)
	    st->pc = (addr_t)arg16;
	  else
	    st->pc += opsize;
	  break;
	case oJGTE :
	  POP(st, sparm1);
	  POP(st, sparm2);
	  if (sparm2 >= sparm1)
	    st->pc = (addr_t)arg16;
	  else
	    st->pc += opsize;
	  break;
	case oJGT  :
	  POP(st, sparm1);
	  POP(st, sparm2);
	  if (sparm2 > sparm1)
	    st->pc = (addr_t)arg16;
	  else
	    st->pc += opsize;
	  break;
	case oJLTE :
	  POP(st, sparm1);
	  POP(st, sparm2);
	  if (sparm2 <= sparm1)
	    st->pc = (addr_t)arg16;
	  else
	    st->pc += opsize;
	  break;

	  /* Load:  arg16 = usigned offset (no stack arguments) */

	case oLD :
	  uparm1 = st->spb + arg16;
	  PUSH(st, GETSTACK(st, uparm1));
	  PUSH(st, GETSTACK(st, uparm1 + BPERI));
	  st->pc += opsize;
	  break;
	case oLDH :
	  uparm1 = st->spb + arg16;
	  PUSH(st, GETSTACK(st, uparm1));
	  st->pc += opsize;
	  break;
	case oLDB :
	  uparm1 = st->spb + arg16;
	  PUSH(st, GETBSTACK(st, uparm1));
	  st->pc += opsize;
	  break;
	case oLDM :
 /* FIX ME --> Need to handle the unaligned case */
	  POP(st, uparm1);
	  uparm2 = st->spb + arg16;
	  while (uparm1 > 0)
	    {
	      if (uparm1 >= BPERI)
		{
		  PUSH(st, GETSTACK(st, uparm2));
		  uparm2 += BPERI;
		  uparm1 -= BPERI;
		} /* end if */
	      else
		{
		  PUSH(st, GETBSTACK(st, uparm2));
		  uparm2++;
		  uparm1--;
		} /* end else */
            } /* end while */
	  st->pc += opsize;
	  break;

	  /* Load & store: arg16 = unsigned base offset (One stack argument) */

	case oST :
	  uparm1 = st->spb + arg16;
	  POP(st, uparm2);
	  PUTSTACK(st, uparm2, uparm1 + BPERI);
	  POP(st, uparm2);
	  PUTSTACK(st, uparm2, uparm1);
	  st->pc += opsize;
	  break;
	case oSTH   :
	  uparm1  = st->spb + arg16;
	  POP(st, uparm2);
	  PUTSTACK(st, uparm2, uparm1);
	  st->pc += opsize;
	  break;
	case oSTB  :
	  uparm1  = st->spb + arg16;
	  POP(st, uparm2);
	  PUTBSTACK(st, uparm2, uparm1);
	  st->pc += opsize;
	  break;
	case oSTM :
 /* FIX ME --> Need to handle the unaligned case */
	  POP(st, uparm1);                /* Size */
          uparm3 = uparm1;            /* Save for stack discard */
	  uparm2  = st->spb + arg16;
	  sparm1 = ROUNDBTOI(uparm1) - 1;
	  while (uparm1 > 0)
	    {
	      if (uparm1 >= BPERI)
		{
		  PUTSTACK(st, TOS(st, sparm1), uparm2);
		  uparm2 += BPERI;
		  uparm1 -= BPERI;
		  sparm1--;
		} /* end if */
	      else
		{
		  PUTBSTACK(st, TOS(st, sparm1), uparm2);
		  uparm2++;
		  uparm1--;
		} /* end else */
	    } /* end while */

	  /* Discard the stored data */

	  DISCARD(st, ROUNDBTOI(uparm3));
	  st->pc += opsize;
	  break;
	case oLDX  :
	  uparm1 = st->spb + arg16 + TOS(st, 0);
	  TOS(st, 0) = GETSTACK(st, uparm1);
	  PUSH(st, GETSTACK(st, uparm1 + BPERI));
	  st->pc += opsize;
	  break;
	case oLDXH  :
	  uparm1 = st->spb + arg16 + TOS(st, 0);
	  TOS(st, 0) = GETSTACK(st, uparm1);
	  st->pc += opsize;
	  break;
	case oLDXB :
	  uparm1 = st->spb + arg16 + TOS(st, 0);
	  TOS(st, 0) = GETBSTACK(st, uparm1);
	  st->pc += opsize;
	  break;
	case oLDXM  :
 /* FIX ME --> Need to handle the unaligned case */
	  POP(st, uparm1);
	  POP(st, uparm2);
	  uparm2 += st->spb + arg16;
	  while (uparm1 > 0)
	    {
	      if (uparm1 >= BPERI)
		{
		  PUSH(st, GETSTACK(st, uparm2));
		  uparm2 += BPERI;
		  uparm1 -= BPERI;
		} /* end if */
	      else
		{
		  PUSH(st, GETBSTACK(st, uparm2));
		  uparm2++;
		  uparm1--;
		} /* end else */
            } /* end while */
	  st->pc += opsize;
	  break;

	  /* Store: arg16 = unsigned base offset (Two stack arguments) */

	case oSTXH  :
	  POP(st, uparm1);
	  POP(st, uparm2);
	  uparm2 += st->spb + arg16;
	  PUTSTACK(st, uparm1,uparm2);
	  st->pc += opsize;
	  break;
	case oSTXB :
	  POP(st, uparm1);
	  POP(st, uparm2);
	  uparm2 += st->spb + arg16;
	  PUTBSTACK(st, uparm1, uparm2);
	  st->pc += opsize;
	  break;
	case oSTXM :
/* FIX ME --> Need to handle the unaligned case */
	  POP(st, uparm1);                /* Size */
          uparm3 = uparm1;            /* Save for stack discard */
	  sparm1 = ROUNDBTOI(uparm1); /* Size in 16-bit words */
	  uparm2 = TOS(st, sparm1);       /* index */
	  sparm1--;
	  uparm2 += st->spb + arg16;
	  while (uparm1 > 0)
	    {
	      if (uparm1 >= BPERI)
		{
		  PUTSTACK(st, TOS(st, sparm1), uparm2);
		  uparm2 += BPERI;
		  uparm1 -= BPERI;
		  sparm1--;
		} /* end if */
	      else
		{
		  PUTBSTACK(st, TOS(st, sparm1), uparm2);
		  uparm2++;
		  uparm1--;
		} /* end else */
	    } /* end while */

	  /* Discard the stored data + the index */

	  DISCARD(st, (ROUNDBTOI(uparm3) + 1));
	  st->pc += opsize;
	  break;

	case oLA  :
	  uparm1 = st->spb + arg16;
	  PUSH(st, uparm1);
	  st->pc += opsize;
	  break;
	case oLAX :
	  TOS(st, 0) = st->spb + arg16 + TOS(st, 0);
	  st->pc += opsize;
	  break;

	  /* Data stack:  arg16 = 16 bit signed data (no stack arguments) */

	case oPUSH  :
	  PUSH(st, arg16);
	  st->pc += opsize;
	  break;
	case oINDS  :
	  st->sp += signExtend16(arg16);
	  st->pc += opsize;
	  break;

	  /* System Functions:
	   * For LIB:        arg16 = sub-function code
	   */

	case oLIB  :
	  st->pc += opsize;
	  return pexec_libcall(st, arg8, arg16);

	  /* Program control:  arg16 = unsigned label (no stack arguments) */

	case oLAC :
	  uparm1 = arg16 + st->rop;
	  PUSH(st, uparm1);
	  st->pc += opsize;
	  break;

	case oLABEL :
	  return eILLEGALOPCODE;

/**---------------------------------------------------------------------
 OPCODES WITH BYTE ARGUMENT (arg8) AND 16-BIT ARGUMENT (arg16)
 ---------------------------------------------------------------------**/
	  /* Load:  arg8 = level; arg16 = signed frame offset (no stack arguments) */
	case oLDS :
	  uparm1 = pexec_getbaseaddress(st, arg8) + signExtend16(arg16);
	  PUSH(st, GETSTACK(st, uparm1));
	  PUSH(st, GETSTACK(st, uparm1 + BPERI));
	  st->pc += opsize;
	  break;
	case oLDSH :
	  uparm1 = pexec_getbaseaddress(st, arg8) + signExtend16(arg16);
	  PUSH(st, GETSTACK(st, uparm1));
	  st->pc += opsize;
	  break;
	case oLDSB :
	  uparm1 = pexec_getbaseaddress(st, arg8) + signExtend16(arg16);
	  PUSH(st, GETBSTACK(st, uparm1));
	  st->pc += opsize;
	  break;
	case oLDSM :
 /* FIX ME --> Need to handle the unaligned case */
	  POP(st, uparm1);
	  uparm2 = pexec_getbaseaddress(st, arg8) + signExtend16(arg16);
	  while (uparm1 > 0)
	    {
	      if (uparm1 >= BPERI)
		{
		  PUSH(st, GETSTACK(st, uparm2));
		  uparm2 += BPERI;
		  uparm1 -= BPERI;
		} /* end if */
	      else
		{
		  PUSH(st, GETBSTACK(st, uparm2));
		  uparm2++;
		  uparm1--;
		} /* end else */
            } /* end while */
	  st->pc += opsize;
	  break;

	  /* Load & store: arg8 = level; arg16 = signed frame offset (One stack argument) */

	case oSTSH   :
	  uparm1  = pexec_getbaseaddress(st, arg8) + signExtend16(arg16);
	  POP(st, uparm2);
	  PUTSTACK(st, uparm2, uparm1);
	  st->pc += opsize;
	  break;
	case oSTSB  :
	  uparm1  = pexec_getbaseaddress(st, arg8) + signExtend16(arg16);
	  POP(st, uparm2);
	  PUTBSTACK(st, uparm2, uparm1);
	  st->pc += opsize;
	  break;
	case oSTSM :
 /* FIX ME --> Need to handle the unaligned case */
	  POP(st, uparm1);                /* Size */
          uparm3 = uparm1;            /* Save for stack discard */
	  uparm2  = pexec_getbaseaddress(st, arg8) + signExtend16(arg16);
	  sparm1 = ROUNDBTOI(uparm1) - 1;
	  while (uparm1 > 0)
	    {
	      if (uparm1 >= BPERI)
		{
		  PUTSTACK(st, TOS(st, sparm1), uparm2);
		  uparm2 += BPERI;
		  uparm1 -= BPERI;
		  sparm1--;
		} /* end if */
	      else
		{
		  PUTBSTACK(st, TOS(st, sparm1), uparm2);
		  uparm2++;
		  uparm1--;
		} /* end else */
	    } /* end while */

	  /* Discard the stored data */

	  DISCARD(st, ROUNDBTOI(uparm3));
	  st->pc += opsize;
	  break;
	case oLDSX  :
	  uparm1 = pexec_getbaseaddress(st, arg8) + signExtend16(arg16) + TOS(st, 0);
	  TOS(st, 0) = GETSTACK(st, uparm1);
	  PUSH(st, GETSTACK(st, uparm1 + BPERI));
	  st->pc += opsize;
	  break;
	case oLDSXH  :
	  uparm1 = pexec_getbaseaddress(st, arg8) + signExtend16(arg16) + TOS(st, 0);
	  TOS(st, 0) = GETSTACK(st, uparm1);
	  st->pc += opsize;
	  break;
	case oLDSXB :
	  uparm1 = pexec_getbaseaddress(st, arg8) + signExtend16(arg16) + TOS(st, 0);
	  TOS(st, 0) = GETBSTACK(st, uparm1);
	  st->pc += opsize;
	  break;
	case oLDSXM  :
 /* FIX ME --> Need to handle the unaligned case */
	  POP(st, uparm1);
	  POP(st, uparm2);
	  uparm2 += pexec_getbaseaddress(st, arg8) + signExtend16(arg16);
	  while (uparm1 > 0)
	    {
	      if (uparm1 >= BPERI)
		{
		  PUSH(st, GETSTACK(st, uparm2));
		  uparm2 += BPERI;
		  uparm1 -= BPERI;
		} /* end if */
	      else
		{
		  PUSH(st, GETBSTACK(st, uparm2));
		  uparm2++;
		  uparm1--;
		} /* end else */
            } /* end while */
	  st->pc += opsize;
	  break;

	  /* Store: arg8 = level; arg16 = signed frame offset (Two stack arguments) */

	case oSTSXH  :
	  POP(st, uparm1);
	  POP(st, uparm2);
	  uparm2 += pexec_getbaseaddress(st, arg8) + signExtend16(arg16);
	  PUTSTACK(st, uparm1,uparm2);
	  st->pc += opsize;
	  break;
	case oSTSXB :
	  POP(st, uparm1);
	  POP(st, uparm2);
	  uparm2 += pexec_getbaseaddress(st, arg8) + signExtend16(arg16);
	  PUTBSTACK(st, uparm1, uparm2);
	  st->pc += opsize;
	  break;
	case oSTSXM :
/* FIX ME --> Need to handle the unaligned case */
	  POP(st, uparm1);                /* Size */
          uparm3 = uparm1;            /* Save for stack discard */
	  sparm1 = ROUNDBTOI(uparm1); /* Size in 16-bit words */
	  uparm2 = TOS(st, sparm1);       /* index */
	  sparm1--;
	  uparm2 += pexec_getbaseaddress(st, arg8) + signExtend16(arg16);
	  while (uparm1 > 0)
	    {
	      if (uparm1 >= BPERI)
		{
		  PUTSTACK(st, TOS(st, sparm1), uparm2);
		  uparm2 += BPERI;
		  uparm1 -= BPERI;
		  sparm1--;
		} /* end if */
	      else
		{
		  PUTBSTACK(st, TOS(st, sparm1), uparm2);
		  uparm2++;
		  uparm1--;
		} /* end else */
	    } /* end while */

	  /* Discard the stored data + the index */

	  DISCARD(st, (ROUNDBTOI(uparm3) + 1));
	  st->pc += opsize;
	  break;

	case oLAS  :
	  uparm1 = pexec_getbaseaddress(st, arg8) + signExtend16(arg16);
	  PUSH(st, uparm1);
	  st->pc += opsize;
	  break;
	case oLASX :
	  TOS(st, 0) = pexec_getbaseaddress(st, arg8) + signExtend16(arg16) + TOS(st, 0);
	  st->pc += opsize;
	  break;

	  /* Program Control:  arg8 = level; arg16 = unsigned label (No
	   * stack arguments)
	   */

	case oPCAL  :
	  PUSH(st, pexec_getbaseaddress(st, arg8));
	  PUSH(st, st->fp);
	  uparm1 = st->sp;
	  PUSH(st, st->pc + opsize);
	  st->fp = uparm1;
	  st->pc = (addr_t)arg16;
	  break;

	  /* System Functions:
	   * For SYSIO:   arg8 = file number; arg16 = sub-function code
	   */

	case oSYSIO  :
	  st->pc += opsize;
	  return pexec_sysio(st, arg8, arg16);

	  /* Psuedo-operations:  (No stack arguments)
	   * For LINE:    arg8 = file number; arg16 = line number
	   */

	case oLINE   :
	default :
	  return eILLEGALOPCODE;

	} /* end switch */
    } /* end else */

  return eNOERROR;
}

/****************************************************************************
 * Name: pexec_reset
 ****************************************************************************/

void pexec_reset(struct pexec_s *st)
{
  int dndx;

  /* Setup the bottom of the "normal" pascal stack */

  st->rop   = st->strsize;
  st->spb   = st->strsize + st->rosize;

  /* Initialize the emulated P-Machine registers */

  st->csp   = 0;
  st->sp    = st->spb + 2*BPERI;
  st->fp    = st->spb + BPERI;
  st->pc    = st->entry;

  /* Initialize the P-Machine stack */

  dndx                 = BTOISTACK(st->spb);
  st->dstack.i[dndx]   =  0;
  st->dstack.i[dndx+1] =  0;
  st->dstack.i[dndx+2] = -1;
}

/****************************************************************************
 * Name: pexec_release
 ****************************************************************************/

void pexec_release(struct pexec_s *st)
{
  if (st)
    {
      if (st->dstack.i)
        {
          free(st->dstack.i);
        }

      if (st->ispace)
        {
          free(st->ispace);
        }

      free(st);
    }
}
