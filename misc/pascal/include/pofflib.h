/***************************************************************************
 * pofflib.h
 * Interfaces to the POFF library
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

#ifndef __POFFLIB_H
#define __POFFLIB_H

/***************************************************************************
 * Compilation Switches
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include "keywords.h"
#include "poff.h"

/***************************************************************************
 * Definitions
 ***************************************************************************/

/***************************************************************************
 * Public Types
 ***************************************************************************/

/* The internal form of the POFF data structures are hidden from the caller
 * in these "handles"
 */

typedef void *poffHandle_t;
typedef void *poffProgHandle_t;
typedef void *poffSymHandle_t;

/* This is a externally visible form of a symbol table entry that is
 * not entangled in the POFF internal string table logic.
 */

struct poffLibSymbol_s
{
  /* type is the type of symbol described by this entry.
   * See the STT_ definitions in poff.h.
   */

  ubyte type;

  /* For data section symbols, the following provides the required
   * data space alignment for the symbol memory representation.  For
   * procedures and functions, this value is ignored. See the STT_
   * definitions in poff.h
   */

  ubyte align;

  /* These flags describe the characteristics of the symbol.  See the
   * STF_ definitions above.
   */

  ubyte flags;

  /* name is a reference to the symbol name in the string table
   * section data.
   */
  
  const char *name;

  /* value is the value associated with symbol.  For defined data
   * section symbols, this is the offset into the initialized data
   * section data; for defined procedures and functions, this the
   * offset into program section data.  For undefined symbols, this
   * valid can be used as as addend.
   */

  uint32 value;

  /* For data section symbols, this is the size of the initialized
   * data region associated with the symbol.
   */

  uint32 size;
};
typedef struct poffLibSymbol_s poffLibSymbol_t;

/* The externally visible form of a line number structure.  Line numbers
 * are associated with executable program data sections.
 */

struct poffLibLineNumber_s
{
  /* This is the source file line number */

  uint32 lineno;

  /* This is the full filename of the file containing the line number. */

  const char *filename;

  /* This is an offset to the beginning code in the program data section
   * associated with this line number.
   */

   uint32 offset;
};
typedef struct poffLibLineNumber_s poffLibLineNumber_t;

/* The externally visible form of a debug function info structure.
 */

struct poffLibDebugFuncInfo_s
{
  /* For use outside of libpoff so that the allocated debug
   * information can be retained in a list.
   */

  struct poffLibDebugFuncInfo_s *next;

  /* This is the address or label of the function/procedure entry
   * point.
   */

  uint32 value;

  /* This is the size of the value returned by the function in
   * bytes (zero for procedures).
   */

  uint32 retsize;

  /* This is the number of parameters accepted by the function/
   * procedure.
   */

  uint32 nparms;

  /* This is the beginning of a table of input parameter sizes
   * the actually allocate size will be nparms entries.
   */

  uint32 argsize[1];
};
typedef struct poffLibDebugFuncInfo_s poffLibDebugFuncInfo_t;

#define SIZEOFDEBUFINFO(n) (sizeof(poffLibDebugFuncInfo_t) + ((n)-1)*sizeof(uint32))

/***************************************************************************
 * Public Variables
 ***************************************************************************/

/***************************************************************************
 * Public Function Prototypes
 ***************************************************************************/

/* Functions to create/destroy a handle to POFF file data */

extern poffHandle_t poffCreateHandle(void);
extern void         poffDestroyHandle(poffHandle_t handle);
extern void         poffResetAccess(poffHandle_t handle);

/* Functions to manage writing a POFF file */

extern void         poffSetFileType(poffHandle_t handle, ubyte fh_type,
				    uint16 nfiles, const char *name);
extern void         poffSetArchitecture(poffHandle_t handle, ubyte fh_arch);
extern void         poffSetEntryPoint(poffHandle_t handle, uint32 entryPoint);
extern sint32       poffFindString(poffHandle_t handle, const char *string);
extern uint32       poffAddString(poffHandle_t handle, const char *string);
extern uint32       poffAddFileName(poffHandle_t handle, const char *name);
extern void         poffAddProgByte(poffHandle_t handle, ubyte progByte);
#if 0 /* not used */
extern uint32       poffAddRoDataByte(poffHandle_t handle, ubyte dataByte);
#endif
extern uint32       poffAddRoDataString(poffHandle_t handle,
					const char *string);
extern uint32       poffAddSymbol(poffHandle_t handle,
				  poffLibSymbol_t *symbol);
extern uint32       poffAddLineNumber(poffHandle_t handle,
				      uint16 lineNumber, uint16 fileNumber,
				      uint32 progSectionDataOffset);
extern uint32       poffAddDebugFuncInfo(poffHandle_t handle,
					 poffLibDebugFuncInfo_t *pContainer);
extern uint32       poffAddRelocation(poffHandle_t handle,
				      ubyte relocType, uint32 symIndex,
				      uint32 sectionDataOffset);
extern void         poffWriteFile(poffHandle_t handle, FILE *poffFile);

/* Functions to manage reading a POFF file */

extern uint16       poffReadFile(poffHandle_t handle, FILE *poffFile);
extern ubyte        poffGetFileType(poffHandle_t handle);
extern ubyte        poffGetArchitecture(poffHandle_t handle);
extern uint32       poffGetEntryPoint(poffHandle_t handle);
extern const char  *poffGetFileHdrName(poffHandle_t handle);
extern uint32       poffGetRoDataSize(poffHandle_t handle);
extern sint32       poffGetFileName(poffHandle_t handle, const char **fname);
extern int          poffGetProgByte(poffHandle_t handle);
extern sint32       poffGetSymbol(poffHandle_t handle,
				  poffLibSymbol_t *symbol);
extern const char  *poffGetString(poffHandle_t handle, uint32 index);
extern sint32       poffGetLineNumber(poffHandle_t handle,
				      poffLibLineNumber_t *lineno);
extern sint32       poffGetRawLineNumber(poffHandle_t handle,
					 poffLineNumber_t *lineno);
extern sint32       poffGetRawRelocation(poffHandle_t handle,
					 poffRelocation_t *reloc);
extern poffLibDebugFuncInfo_t *poffGetDebugFuncInfo(poffHandle_t handle);
extern poffLibDebugFuncInfo_t *poffCreateDebugInfoContainer(uint32 nparms);
extern void         poffReleaseDebugFuncContainer(poffLibDebugFuncInfo_t *pDebugFuncInfo);
extern void         poffDiscardDebugFuncInfo(poffHandle_t handle);
extern sint32       poffProgTell(poffHandle_t handle);
extern int          poffProgSeek(poffHandle_t handle, uint32 offset);
extern uint32       poffGetProgSize(poffHandle_t handle);
extern void         poffReleaseProgData(poffHandle_t handle);

/* Functions used to manage modifications to a POFF file using a
 * temporary container for the new program data.
 */

extern poffProgHandle_t poffCreateProgHandle(void);
extern void         poffDestroyProgHandle(poffProgHandle_t handle);
extern void         poffResetProgHandle(poffProgHandle_t handle);
extern uint16       poffAddTmpProgByte(poffProgHandle_t handle,
				       ubyte progByte);
extern uint16       poffWriteTmpProgBytes(ubyte *buffer, uint32 nbyte,
					  poffProgHandle_t handle);
extern void         poffReplaceProgData(poffHandle_t handle,
					poffProgHandle_t progHandle);

/* Functions used to manage modifications to a POFF file using a
 * temporary container for the new symbol data.
 */

extern poffSymHandle_t poffCreateSymHandle(void);
extern void         poffDestroySymHandle(poffSymHandle_t handle);
extern void         poffResetSymHandle(poffSymHandle_t handle);
extern uint32       poffAddTmpSymbol(poffHandle_t handle, poffSymHandle_t symHandle,
				     poffLibSymbol_t *symbol);
extern void         poffReplaceSymbolTable(poffHandle_t handle,
					   poffSymHandle_t symHandle);

/* Functions used to extract/insert whole data sections from/into a POFF
 * file container
 */

extern uint32       poffExtractProgramData(poffHandle_t handle,
					   ubyte **progData);
extern void         poffInsertProgramData(poffHandle_t handle,
					  ubyte *progData, uint32 progSize);
extern uint32       poffExtractRoData(poffHandle_t handle,
				      ubyte **roData);
extern void         poffAppendRoData(poffHandle_t handle,
				     ubyte *roData, uint32 roDataSize);

/* Functions to manage printing of the POFF file content */

extern void         poffDumpFileHeader(poffHandle_t handle, FILE *outFile);
extern void         poffDumpSectionHeaders(poffHandle_t handle, FILE *outFile);
extern void         poffDumpSymbolTable(poffHandle_t handle, FILE *outFile);
extern void         poffDumpRelocTable(poffHandle_t handle, FILE *outFile);

/* Helper functions to manage resolution of labels in POFF files.  These
 * just store and retrieve information by label number.
 */

extern void         poffAddToDefinedLabelTable(uint32 label, uint32 pc);
extern void         poffAddToUndefinedLabelTable(uint32 label,
						 uint32 symIndex);
extern int          poffGetSymIndexForUndefinedLabel(uint32 label);
extern int          poffGetPcForDefinedLabel(uint32 label);
extern void         poffReleaseLabelReferences(void);

/* Helper functions for line numbers */

extern void         poffReadLineNumberTable(poffHandle_t handle);
extern poffLibLineNumber_t *poffFindLineNumber(uint32 offset);
extern void         poffReleaseLineNumberTable(void);

/* Helper functions for debug information */

extern void         poffReadDebugFuncInfoTable(poffHandle_t handle);
extern poffLibDebugFuncInfo_t *poffFindDebugFuncInfo(uint32 offset);
extern void         poffReplaceDebugFuncInfo(poffHandle_t handle);
extern void         poffReleaseDebugFuncInfoTable(void);

#endif /* __POFFLIB_H */
