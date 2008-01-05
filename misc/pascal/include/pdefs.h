/***********************************************************************
 * pdefs.h
 * General definitions for the Pascal Compiler/Optimizer
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
 ***********************************************************************/

#ifndef __PDEFS_H
#define __PDEFS_H

/***********************************************************************
 * Included Files
 ***********************************************************************/

#include <stdio.h> /* for FILE */
#include <config.h>

/***********************************************************************
 * Definitions
 ***********************************************************************/

/* Size Parameters -- some of these can be overridden from the
 * command line.
 */

#define MAX_SYM           (4096)
#define MAX_STRINGS       (65536)
#define MAX_INCL           3
#define MAX_FILES          8            /* max number of opened files */
#define FNAME_SIZE         40           /* max size file name */
#define LINE_SIZE          256          /* max size of input line buffer */
#define FNAME_SIZE         40           /* max size of file name */
#define MAX_INCPATHES       8           /* max number of include pathes */

/* Data Storage Sizes */

#ifdef CONFIG_INSN16
# define sINT_SIZE          2
# define MAXINT             32767
# define MININT            -32768
# define BITS_IN_INTEGER    16
# define MAXUINT            0xffff
# define MINUINT            0
#endif

#ifdef CONFIG_INSN32
# define sINT_SIZE          4
# define MAXINT             2147483647
# define MININT            -2147483648
# define BITS_IN_INTEGER    32
# define MAXUINT            0xffffffff
# define MINUINT            0
#endif

#define sCHAR_SIZE         1
#define sBOOLEAN_SIZE      sINT_SIZE
#define sREAL_SIZE         8
#define sPTR_SIZE          sINT_SIZE
#define sRETURN_SIZE      (3*sPTR_SIZE)

#define sSTRING_HDR_SIZE   2
#define sSTRING_SIZE       256                    /* size(2) + string(255) */
#define sSTRING_MAX_SIZE  (sSTRING_SIZE - 2)      /* string storage size(254) */
#define sRSTRING_SIZE     (sPTR_SIZE + sINT_SIZE) /* ptr + size */
#define sCSTRING_SIZE     (sizeof(void*))         /* absolute C pointer */

#define MAXCHAR            255
#define MINCHAR            0

/* Bit values for the 'flags' field of the symType_t, symProc_t, and
 * symVar_t (see below)
 */

#define STYPE_VARSIZE      0x01 /* Type has variable size */
#define SPROC_EXTERNAL     0x01 /* Proc/func. is defined externally */
#define SVAR_EXTERNAL      0x01 /* Variable is defined externally */

/***********************************************************************
 * Public Enumeration Types
 ***********************************************************************/

/* This enumeration identies what kind of binary object we are creating
 * with the compilation.  At present, we may be generating either a 
 * program binary or a unit binary.
 */

enum fileKind_e
{
  eIsProgram = 0,
  eIsUnit
};
typedef enum fileKind_e fileKind_t;

/* This enumeration determines what part of a file that we are
 * processing now.
 */

enum fileSection_e
{
  eIsOtherSection = 0,      /* Unspecified part of the file */
  eIsProgramSection,        /* Any part of a program file */
  eIsInterfaceSection,      /* INTERFACE section of a unit file */
  eIsImplementationSection, /* IMPLEMENTATION section of a unit file */
  eIsInitializationSection, /* INITIALIZATION section of a unit file */
};
typedef enum fileSection_e fileSection_t;

/***********************************************************************
 * Public Structure Types
 ***********************************************************************/

/* Reserved word table entry */

struct R
{
  char   *rname;         /* pointer to name in string stack */
  ubyte   rtype;         /* reserved word type */
  ubyte   subtype;       /* reserved word extended type */
};
typedef struct R RTYPE;

/* Symbol table entry */

struct symType_s         /* for sKind = sTYPE */
{
  ubyte     type;        /* specific type */
  ubyte     rtype;       /* reference to type */
  ubyte     subType;     /* constant type for subrange types */
  ubyte     flags;       /* flags to customize a type (see above) */
  uint32    asize;       /* size of allocated instances of this type */
  uint32    rsize;       /* size of reference to an instances of this type */
  sint32    minValue;    /* minimum value taken subrange */
  sint32    maxValue;    /* maximum value taken by subrange or scalar */
  struct S *parent;      /* pointer to parent type */
};
typedef struct symType_s symType_t;

struct symConst_s        /* for sKind == constant type */
{
  union
  {
    float64 f;           /* real value */
    sint32  i;           /* integer value */
  } val;
  struct S *parent;      /* pointer to parent type */
};
typedef struct symConst_s symConst_t;

struct symStringConst_s  /* for sKind == sSTRING_CONST */
{
  uint32 offset;         /* RO data section offset of string */
  uint32 size;           /* length of string in bytes */
};
typedef struct symStringConst_s symStringConst_t;

struct symVarString_s    /* for sKind == sSTRING */
{
  uint16 label;          /* label at string declaration */
  uint16 size;           /* valid length of string in bytes */
  uint16 alloc;          /* max length of string in bytes */
};
typedef struct symVarString_s symVarString_t;

struct symLabel_s        /* for sKind == sLABEL */
{
  uint16  label;         /* label number */
  boolean unDefined;     /* set false when defined */
};
typedef struct symLabel_s symLabel_t;

struct symVar_s          /* for sKind == type identifier */
{
  sint32    offset;      /* Data stack offset */
  uint32    size;        /* Size of variable */
  ubyte     flags;       /* flags to customize a variable (see above) */
  uint32    symIndex;    /* POFF symbol table index (if undefined) */
  struct S *parent;      /* pointer to parent type */
};
typedef struct symVar_s symVar_t;

struct symProc_s         /* for sKind == sPROC or sFUNC */
{
  uint16    label;       /* entry point label */
  uint16    nParms;      /* number of parameters that follow */
  ubyte     flags;       /* flags to customize a proc/func (see above) */
  uint32    symIndex;    /* POFF symbol table index (if undefined) */
  struct S *parent;      /* pointer to parent type (sFUNC only) */
};
typedef struct symProc_s symProc_t;

struct symRecord_s       /* for sKind == sRECORD_OBJECT */
{
  uint32    size;        /* size of this field */
  uint32    offset;      /* offset into the RECORD */
  struct S *record;      /* pointer to parent sRECORD type */
  struct S *parent;      /* pointer to parent field type */
};
typedef struct symRecord_s symRecord_t;

struct S
{
  char     *sName;       /* pointer to name in string stack */
  ubyte     sKind;       /* kind of symbol */
  ubyte     sLevel;      /* static nesting level */
  union
  {
    symType_t        t;          /* for type definitions */
    symConst_t       c;          /* for constants */
    symStringConst_t s;          /* for strings of constant size*/
    symVarString_t   vs;         /* for strings of variable size*/
    uint16           fileNumber; /* for files */
    symLabel_t       l;          /* for labels */
    symVar_t         v;          /* for variables */
    symProc_t        p;          /* for functions & procedures */
    symRecord_t      r;          /* for files of RECORDS */
  } sParm;
};
typedef struct S STYPE;

/* WITH structure */

struct W
{
  ubyte     level;       /* static nesting level */
  boolean   pointer;     /* TRUE if offset is to pointer to RECORD */
  boolean   varParm;     /* TRUE if VAR param (+pointer) */
  sint32    offset;      /* Data stack offset */
  uint16    index;       /* RECORD offset (if pointer) */
  STYPE    *parent;      /* pointer to parent RECORD type */
};
typedef struct W WTYPE;

/* File table record */

struct F
{
  sint16  defined;
  sint16  flevel;
  sint16  ftype;
  sint32  faddr;
  sint16  fsize;
};
typedef struct F FTYPE;

#ifdef CONFIG_INSN16
typedef struct P
{
  ubyte   op;
  ubyte   arg1;
  uint16  arg2;
} OPTYPE;
#endif

#ifdef CONFIG_INSN32
typedef struct P
{
  ubyte   op;
  uint32  arg;
} OPTYPE;
#endif

/* This structure captures the parsing state of the compiler for a particular
 * file.  Since multiple, nested files can be processed, this represents
 * only level in the "stack" of nested files.
 */

struct fileState_s
{
  /* These fields are managed by the higher level parsing logic
   *
   * stream    - Stream pointer the input stream associated with this
   *             file.
   * kind      - Kind of file we are processing.  If include > 0,
   *             this should be eIsUnit.
   * section   - This is the part of the program that we are parsing
   *             now.
   * dstack    - Level zero dstack offset at the time the unit was
   *             included.  This is used to convert absolute program
   *             stack offsets into relative unit stack offsets.
   * include   - Is a unique number that identifies the file.  In
   *             POFF ouput file, this would be the index to the
   *             entry in the .files section.
   */

  FILE          *stream;
  fileKind_t     kind;
  fileSection_t  section;
  sint32         dstack;
  sint16         include;

  /* These fields are managed by the tokenizer.  These are all
   * initialized by primeTokenizer().
   *
   * buffer[]  - Holds the current input line
   * line      - Is the line number in this file for the current line
   * cp        - Is the current pointer into buffer[]
   */

  uint32         line;
  unsigned char *cp;
  unsigned char  buffer[LINE_SIZE + 1];
};
typedef struct fileState_s fileState_t;

#endif /* __PDEFS_H */
