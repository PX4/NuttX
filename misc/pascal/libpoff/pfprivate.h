/***************************************************************************
 * pfprivate.h
 * Contains command, internal, private definitions used by
 * the POFF library.  These were not intended for exportation.
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
 ***************************************************************************/

#ifndef __PFPRIVATE_H
#define __PFPRIVATE_H

/***************************************************************************
 * Compilation Switches
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include "keywords.h"
#include "poff.h"
#include "paslib.h"    /* Endian-ness support */

/***************************************************************************
 * Definitions
 ***************************************************************************/

#define INITIAL_STRING_TABLE_SIZE     4096
#define STRING_TABLE_INCREMENT        1024

#define INITIAL_SYMBOL_TABLE_SIZE     256*sizeof(poffSymbol_t)
#define SYMBOL_TABLE_INCREMENT        64*sizeof(poffSymbol_t)

#define INITIAL_FILENAME_TABLE_SIZE   64*sizeof(poffFileTab_t)
#define FILENAME_TABLE_INCREMENT      64*sizeof(poffFileTab_t)

#define INITIAL_RELOC_TABLE_SIZE      128*sizeof(poffRelocation_t)
#define RELOC_TABLE_INCREMENT         64*sizeof(poffRelocation_t)

#define INITIAL_LINENUMBER_TABLE_SIZE 2048*sizeof(poffLineNumber_t)
#define LINENUMBER_TABLE_INCREMENT    512*sizeof(poffLineNumber_t)

#define INITIAL_DEBUGFUNC_TABLE_SIZE  128*sizeof(poffDebugFuncInfo_t)
#define DEBUGFUNC_TABLE_INCREMENT     64*sizeof(poffDebugFuncInfo_t)

#define INITIAL_PROG_SECTION_SIZE     8096
#define PROG_SECTION_INCREMENT        2048

#define INITIAL_RODATA_SECTION_SIZE   4096
#define RODATA_SECTION_INCREMENT      1024

#define HAVE_PROGRAM_SECTION (poffInfo->progSection.sh_size > 0)
#define HAVE_RODATA_SECTION  (poffInfo->roDataSection.sh_size > 0)
#define HAVE_SYMBOL_TABLE    (poffInfo->symbolTableSection.sh_size > 0)
#define HAVE_STRING_TABLE    (poffInfo->stringTableSection.sh_size > 0)
#define HAVE_RELOC_SECTION   (poffInfo->relocSection.sh_size > 0)
#define HAVE_FILE_TABLE      (poffInfo->fileNameTableSection.sh_size > 0)
#define HAVE_LINE_NUMBER     (poffInfo->lineNumberSection.sh_size > 0)
#define HAVE_DEBUG_SECTION   (poffInfo->debugFuncSection.sh_size > 0)

#ifndef CONFIG_POFF_SWAPNEEDED
# define poffSwapFileHeader(p)
# define poffSwapSectionHeader(p)
#endif

/***************************************************************************
 * Public Types
 ***************************************************************************/

struct poffInfo_s
{
  /* POFF file header */

  poffFileHeader_t    fileHeader;

  /* Section headers: */

  poffSectionHeader_t progSection;
  poffSectionHeader_t roDataSection;
  poffSectionHeader_t symbolTableSection;
  poffSectionHeader_t stringTableSection;
  poffSectionHeader_t relocSection;
  poffSectionHeader_t fileNameTableSection;
  poffSectionHeader_t lineNumberSection;
  poffSectionHeader_t debugFuncSection;

  /* In-memory section data */

  ubyte              *progSectionData;
  ubyte              *roDataSectionData;
  ubyte              *symbolTable;
  char               *stringTable;
  ubyte              *relocTable;
  poffFileTab_t      *fileNameTable;
  ubyte              *lineNumberTable;
  ubyte              *debugFuncTable;

  /* Current allocation sizes.  Used only on writing to determine if
   * in-memory has been allocated and how much memory has been allocated
   * in case the buffer needs to be re-allocated.
   */

  uint32              progSectionAlloc;
  uint32              roDataSectionAlloc;
  uint32              symbolTableAlloc;
  uint32              stringTableAlloc;
  uint32              relocAlloc;
  uint32              fileNameTableAlloc;
  uint32              lineNumberTableAlloc;
  uint32              debugFuncTableAlloc;

  /* Current buffer indices.  These are used on reading data sequentially
   * from the in-memory section data.
   */

  uint32              progSectionIndex;
  uint32              symbolIndex;
  uint32              relocIndex;
  uint32              fileNameIndex;
  uint32              lineNumberIndex;
  uint32              debugFuncIndex;
};
typedef struct poffInfo_s poffInfo_t;

struct poffProgInfo_s
{
  uint32              progSectionSize;
  uint32              progSectionAlloc;
  ubyte              *progSectionData;
};
typedef struct poffProgInfo_s poffProgInfo_t;

struct poffSymInfo_s
{
  uint32              symbolTableSize;
  uint32              symbolTableAlloc;
  ubyte              *symbolTable;
};
typedef struct poffSymInfo_s poffSymInfo_t;

/***************************************************************************
 * Public Variables
 ***************************************************************************/

/***************************************************************************
 * Public Function Prototypes
 ***************************************************************************/

#ifdef CONFIG_POFF_SWAPNEEDED
extern void poffSwapFileHeader(poffFileHeader_t *pFileHeader);
extern void poffSwapSectionHeader(poffSectionHeader_t *pSectionHeader);
extern void poffSwapSymbolTableData(poffInfo_t *poffInfo);
extern void poffSwapRelocationData(poffInfo_t *poffInfo);
extern void poffSwapFileTableData(poffInfo_t *poffInfo);
extern void poffSwapLineNumberData(poffInfo_t *poffInfo);
extern void poffSwapDebugData(poffInfo_t *poffInfo);
#endif

#endif /* __PFPRIVATE_H */
