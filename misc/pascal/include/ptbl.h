/***************************************************************************
 * ptbl.h
 * External Declarations associated with ptbl.c
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

#ifndef __PTBL_H
#define __PTBL_H

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include "config.h"

/***************************************************************************
 * Global Variables
 ***************************************************************************/

extern STYPE *parentInteger;
extern STYPE *parentString;

/***************************************************************************
 * Global Function Prototypes
 ***************************************************************************/

extern const RTYPE *findReservedWord (char *name);
extern STYPE *findSymbol       (char *inName);
extern STYPE *addTypeDefine    (char *name, ubyte type, uint16 size,
				STYPE *parent);
extern STYPE *addConstant      (char *name, ubyte type, sint32 *value,
				STYPE *parent);
extern STYPE *addStringConst   (char *name, uint32 offset, uint32 size);
extern STYPE *addFile          (char *name, uint16 fileNumber);
extern STYPE *addLabel         (char *name, uint16 label);
extern STYPE *addProcedure     (char *name, ubyte type, uint16 label,
				uint16 nParms, STYPE *parent);
extern STYPE *addVariable      (char *name, ubyte type, uint16 offset,
				uint16 size, STYPE *parent);
extern STYPE *addField         (char *name, STYPE *record);
extern void   primeSymbolTable (unsigned long symbolTableSize);
extern void   verifyLabels     (sint32 symIndex);

#if CONFIG_DEBUG
extern void   dumpTables       (void);
#endif

#endif /* __PTBL_H */
