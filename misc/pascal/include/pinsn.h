/***************************************************************************
 * pinsn.h
 * External Declarations associated libinsn.a
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

#ifndef __PINSN_H
#define __PINSN_H

/***************************************************************************
 * Included Files
 ***************************************************************************/

/***************************************************************************
 * Global Function Prototypes
 ***************************************************************************/

/* Opcode generators */

extern void   insn_GenerateSimple(enum pcode_e opcode);
extern void   insn_GenerateDataOperation(enum pcode_e opcode, sint32 data);
extern void   insn_GenerateDataSize(uint32 dwDataSize);
extern void   insn_GenerateFpOperation(ubyte fpOpcode);
extern void   insn_GenerateIoOperation(uint16 ioOpcode, uint16 fileNumber);
extern void   insn_BuiltInFunctionCall(uint16 libOpcode);
extern void   insn_GenerateLevelReference(enum pcode_e opcode, uint16 level,
					  sint32 offset);
extern void   insn_GenerateProcedureCall(uint16 level, sint32 offset);
extern void   insn_GenerateLineNumber(uint16 includeNumber, uint32 lineNumber);
extern void   insn_SetStackLevel(uint32 level);

/* Opcode relocation */

extern int    insn_Relocate(OPTYPE *op, uint32 pcOffset, uint32 roOffset);
extern void   insn_FixupProcedureCall(ubyte *progData, uint32 symValue);

/* POFF-wrapped INSNS access helpers */

extern uint32 insn_GetOpCode(poffHandle_t handle, OPTYPE *ptr);
extern void   insn_ResetOpCodeRead(poffHandle_t handle);
extern void   insn_AddOpCode(poffHandle_t handle, OPTYPE *ptr);
extern void   insn_ResetOpCodeWrite(poffHandle_t handle);
extern void   insn_AddTmpOpCode(poffProgHandle_t progHandle, OPTYPE *ptr);
extern void   insn_ResetTmpOpCodeWrite(poffProgHandle_t progHandle);

/* INSN-specific disassembler */

extern void   insn_DisassemblePCode(FILE* lfile, OPTYPE *pop);

#endif /* __PINSN_H */
