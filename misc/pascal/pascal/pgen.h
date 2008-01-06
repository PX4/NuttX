/***************************************************************************
 * pgen.h
 * External Declarations associated with pgen.c
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

#ifndef __PGEN_H
#define __PGEN_H

/***************************************************************************
 * Compilation Switches
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include "podefs.h"

/***************************************************************************
 * Definitions
 ***************************************************************************/

/***************************************************************************
 * Global Types
 ***************************************************************************/

/***************************************************************************
 * Global Variable Prototypes
 ***************************************************************************/

/***************************************************************************
 * Global Function Prototypes
 ***************************************************************************/

extern sint32 pas_GetCurrentStackLevel(void);
extern void   pas_InvalidateCurrentStackLevel(void);
extern void   pas_SetCurrentStackLevel(sint32 dwLsp);
extern uint32 pas_GetNStackLevelChanges(void);

extern void   pas_GenerateSimple(enum pcode_e eOpCode);
extern void   pas_GenerateDataOperation(enum pcode_e eOpCode, sint32 dwData);
extern void   pas_GenerateDataSize(sint32 dwDataSize);
extern void   pas_GenerateFpOperation(ubyte fpOpcode);
extern void   pas_GenerateIoOperation(uint16 ioOpcode, uint16 fileNumber);
extern void   pas_BuiltInFunctionCall(uint16 libOpcode);
extern void   pas_GenerateLevelReference(enum pcode_e eOpCode, uint16 wLevel,
					 sint32 dwOffset);
extern void   pas_GenerateStackReference(enum pcode_e eOpCode, STYPE *pVarPtr);
extern void   pas_GenerateProcedureCall(STYPE *pProcPtr);
extern void   pas_GenerateLineNumber(uint16 wIncludeNumber,
				     uint32 dwLineNumber);
extern void   pas_GenerateStackExport(STYPE *pVarPtr);
extern void   pas_GenerateStackImport(STYPE *pVarPtr);
extern void   pas_GenerateProcedureCall(STYPE *pProcPtr);
extern void   pas_GenerateDebugInfo(STYPE *pProcPtr, uint32 dwReturnSize);
extern void   pas_GenerateProcExport(STYPE *pProcPtr);
extern void   pas_GenerateProcImport(STYPE *pProcPtr);
extern void   pas_GeneratePoffOutput(void);

#endif /* __PGEN_H */

