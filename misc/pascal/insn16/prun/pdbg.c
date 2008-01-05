/**********************************************************************
 * pdbg.c
 * P-Code Debugger
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
 **********************************************************************/

/**********************************************************************
 * Included Files
 **********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <ctype.h>

#include "keywords.h"
#include "pdefs.h"
#include "podefs.h"
#include "pinsn16.h"
#include "pxdefs.h"
#include "pedefs.h"

#include "paslib.h"
#include "pinsn.h"
#include "pexec.h"
#include "pdbg.h"

/**********************************************************************
 * Definitions
 **********************************************************************/

#define TRACE_ARRAY_SIZE       16
#define MAX_BREAK_POINTS        8
#define DISPLAY_STACK_SIZE     16
#define DISPLAY_INST_SIZE      16

/**********************************************************************
 * Private Type Definitions
 **********************************************************************/

enum commandEnum
{
  eCMD_NONE = 0,
  eCMD_RESET,
  eCMD_RUN,
  eCMD_STEP,
  eCMD_NEXT,
  eCMD_GO,
  eCMD_BS,
  eCMD_BC,
  eCMD_DP,
  eCMD_DT,
  eCMD_DS,
  eCMD_DI,
  eCMD_DB,
  eCMD_HELP,
  eCMD_QUIT
};

typedef struct
{
  addrType   PC;
  addrType   SP;
  uStackType TOS;
} traceType;

/**********************************************************************
 * Private Constant Data
 **********************************************************************/

/**********************************************************************
 * Private Data
 **********************************************************************/

static enum commandEnum lastCmd         = eCMD_NONE;
static uint32           lastValue;

/**********************************************************************
 * Private Function Prototypes
 **********************************************************************/

static void     pdbg_showcommands(void);
static void     pdbg_execcommand(struct pexec_s *st, enum commandEnum cmd, uint32 value);
static sint32   pdbg_readdecimal(char *ptr);
static sint32   pdbg_readhex(char *ptr, sint32 defaultValue);
static void     pdbg_programstatus(struct pexec_s *st);
static addrType pdbg_printpcode(struct pexec_s *st, addrType PC, sint16 nItems);
static addrType pdbg_printstack(struct pexec_s *st, addrType SP, sint16 nItems);
static void     pdbg_printregisters(struct pexec_s *st);
static void     pdbg_printtracearray(struct pexec_s *st);
static void     pdbg_addbreakpoint(addrType PC);
static void     pdbg_deletebreakpoint(sint16 bpNumber);
static void     pdbg_printbreakpoints(struct pexec_s *st);
static void     pdbg_checkbreakpoint(struct pexec_s *st);
static void     pdbg_initdebugger(void);
static void     pdbg_debugpcode(struct pexec_s *st);

/**********************************************************************
 * Global Variables
 **********************************************************************/

/**********************************************************************
 * Private Variables
 **********************************************************************/

/* Debugging variables */

static traceType traceArray[TRACE_ARRAY_SIZE];
			/* Holds execution histor */
static uint16   traceIndex;
			/* This is the index into the circular traceArray */
static uint16   numTracePoints;
			/* This is the number of valid enties in traceArray */
static addrType breakPoint[MAX_BREAK_POINTS];
			/* Contains address associated with all active */
                        /* break points. */
static addrType untilPoint;
                        /* The 'untilPoint' is a temporary breakpoint */
static uint16   numBreakPoints;
			/* Number of items in breakPoints[] */
static addrType displayLoc;
			/* P-code display location display */
static boolean  stopExecution;
			/* TRUE means to stop program execution */

/* ? */
static int K;

/* I/O variables */

static char inLine[LINE_SIZE+1];
			/* Command line buffer */

/**********************************************************************
 * Public Functions
 **********************************************************************/

void dbg_run(struct pexec_s *st)
{
  addrType PC;
  int i;

  pdbg_showcommands();
  pdbg_initdebugger();
  pdbg_programstatus(st);

  while (TRUE)
    {
      printf("CMD: ");
      (void) fgets(inLine, LINE_SIZE, stdin);
      switch (toupper(inLine[0]))
	{
	case 'R' :
	  switch (toupper(inLine[1])) {
	  case 'E' :  /* Reset */
	    pdbg_execcommand(st, eCMD_RESET, 0);
	    break;
	  case 'U' :  /* Run */
	    pdbg_execcommand(st, eCMD_RUN, 0);
	    break;
	  default :
	    printf("Unrecognized Command\n");
	    pdbg_execcommand(st, eCMD_HELP, 0);
	    break;
	  } /* end switch */
	  break;
	case 'S' :  /* Single Step (into) */
	  pdbg_execcommand(st, eCMD_STEP, 0);
	  break;
	case 'N' :  /* Single Step (over) */
	  pdbg_execcommand(st, eCMD_NEXT, 0);
	  break;
	case 'G' :  /* Go */
	  pdbg_execcommand(st, eCMD_GO, 0);
	  break;
	case 'B' :
	  switch (toupper(inLine[1])) {
	  case 'S' :  /* Set Breakpoint */
	    PC = pdbg_readhex(&inLine[2], st->pc);
	    pdbg_execcommand(st, eCMD_BS, PC);
	    break;
	  case 'C' :  /* Clear Breakpoint */
	    i =  pdbg_readdecimal(&inLine[2]);
	    pdbg_execcommand(st, eCMD_BC, i);
	    break;
	  default :
	    printf("Unrecognized Command\n");
	    pdbg_execcommand(st, eCMD_HELP, 0);
	    break;
	  } /* end switch */
	  break;
	case 'D' :
	  switch (toupper(inLine[1])) {
	  case 'P' :  /* Display Program Status */
	    pdbg_execcommand(st, eCMD_DP, 0);
	    break;
	  case 'T' :  /* Display Program Trace */
	    pdbg_execcommand(st, eCMD_DT, 0);
	    break;
	  case 'S' :  /* Display Stack */
	    PC = pdbg_readhex(&inLine[2], st->sp);
	    pdbg_execcommand(st, eCMD_DS, PC);
	    break;
	  case 'I' :  /* Display Instructions */
	    PC = pdbg_readhex(&inLine[2], st->pc);
	    pdbg_execcommand(st, eCMD_DI, PC);
	    break;
	  case 'B' :  /* Display Breakpoints */
	    pdbg_execcommand(st, eCMD_DB, PC);
	    break;
	  default :
	    printf("Unrecognized Command\n");
	    pdbg_execcommand(st, eCMD_HELP, 0);
	    break;
	  } /* end switch */
	  break;
	case 'Q' :  /* Quit */
	  pdbg_execcommand(st, eCMD_QUIT, PC);
	  break;
	case 'H' :  /* Help */
	case '?' :
	  pdbg_execcommand(st, eCMD_HELP, 0);
	  break;
	case '\0' : /* Repeat last command */
	case '\n' : /* Repeat last command */
	  pdbg_execcommand(st, lastCmd, lastValue);
	  break;
	default :
	  printf("Unrecognized Command\n");
	  pdbg_execcommand(st, eCMD_HELP, 0);
	  break;
	} /* end switch */
    } /* end while */

} /* end pdbg_debugpcodeProgram */

/**********************************************************************
 * Private Functions
 **********************************************************************/
/* Show command characters */

static void pdbg_showcommands(void)
{
   printf("Commands:\n");
   printf("  RE[set]   - Reset\n");
   printf("  RU[n]     - Run\n");
   printf("  S[tep]    - Single Step (Into)\n");
   printf("  N[ext]    - Single Step (Over)\n");
   printf("  G[o]      - Go\n");
   printf("  BS xxxx   - Set Breakpoint\n");
   printf("  BC n      - Clear Breakpoint\n");
   printf("  DP        - Display Program Status\n");
   printf("  DT        - Display Program Trace\n");
   printf("  DS [xxxx] - Display Stack\n");
   printf("  DI [xxxx] - Display Instructions\n");
   printf("  DB        - Display Breakpoints\n");
   printf("  H or ?    - Shows this list\n");
   printf("  Q[uit]    - Quit\n");

} /* end pdbg_showcommands */

/***********************************************************************/
static void pdbg_execcommand(struct pexec_s *st, enum commandEnum cmd, uint32 value)
{
  /* Save the command to resuse if the user enters nothing */

  lastCmd = cmd;
  lastValue = value;

  switch (cmd)
    {
    case eCMD_NONE:   /* Do nothing */
      break;
    case eCMD_RESET:  /* Reset */
      pexec_reset(st);
      pdbg_initdebugger();
      pdbg_programstatus(st);
      lastCmd = eCMD_NONE;
      break;
    case eCMD_RUN:    /* Run */
      pexec_reset(st);
      pdbg_initdebugger();
      pdbg_debugpcode(st);
      pdbg_programstatus(st);
      break;
    case eCMD_STEP:   /* Single Step (into)*/
      stopExecution = TRUE;
      pdbg_debugpcode(st);
      pdbg_programstatus(st);
      break;
    case eCMD_NEXT:   /* Single Step (over) */
      if (st->ispace[st->pc] == oPCAL)
	{
	  stopExecution = FALSE;
	  untilPoint = st->pc + 4;
	}
      else
	{
	  stopExecution = TRUE;
	}
      pdbg_debugpcode(st);
      untilPoint = 0;
      pdbg_programstatus(st);
      break;
    case eCMD_GO:     /* Go */
      stopExecution = FALSE;
      pdbg_debugpcode(st);
      pdbg_programstatus(st);
      break;
    case eCMD_BS:     /* Set Breakpoint */
      if (numBreakPoints >= MAX_BREAK_POINTS)
	{
	  printf("Too many breakpoints\n");
	  lastCmd = eCMD_NONE;
	}
      else if (value >= st->maxpc)
	{
	  printf("Invalid address for breakpoint\n");
	  lastCmd = eCMD_NONE;
	}
      else
	{
	  pdbg_addbreakpoint(value);
	  pdbg_printbreakpoints(st);
	} /* end else */
      break;
    case eCMD_BC:     /* Clear Breakpoint */
      if ((value >= 1) && (value <= numBreakPoints))
	{
	  pdbg_deletebreakpoint(value);
	}
      else
	{
	  printf("Invalid breakpoint number\n");
	  lastCmd = eCMD_NONE;
	}
      pdbg_printbreakpoints(st);
      break;
    case eCMD_DP:     /* Display Program Status */
      pdbg_programstatus(st);
      break;
    case eCMD_DT:     /* Display Program Trace */
      pdbg_printtracearray(st);
      break;
    case eCMD_DS:     /* Display Stack */
      if (value > st->sp)
	{
	  printf("Invalid stack address\n");
	  lastCmd = eCMD_NONE;
	}
      else
	{
	  lastValue = pdbg_printstack(st, value, DISPLAY_STACK_SIZE);
	} /* end else */
      break;
    case eCMD_DI:     /* Display Instructions */
      if (value >= st->maxpc)
	{
	  printf("Invalid instruction address\n");
	  lastCmd = eCMD_NONE;
	}
      else
	{
	  lastValue = pdbg_printpcode(st, value, DISPLAY_INST_SIZE);
	} /* end else */
      break;
    case eCMD_DB:     /* Display Breakpoints */
      pdbg_printbreakpoints(st);
      break;
    case eCMD_QUIT:   /* Quit */
      printf("Goodbye\n");
      exit(0);
      break;
    case eCMD_HELP:   /* Help */
    default:          /* Internal error */
      pdbg_showcommands();
      lastCmd = eCMD_NONE;
      break;
    } /* end switch */

} /* end pdbg_execcommand */

/***********************************************************************/
/* Read a decimal value from the  input string */

static sint32 pdbg_readdecimal(char *ptr)
{
   sint32 decimal = 0;

   while (!isspace(*ptr)) ptr++;
   while (isspace(*ptr))  ptr++;
   for (; ((*ptr >= '0') && (*ptr <= '9')); ptr++)
      decimal = 10*decimal + (sint32)*ptr - (sint32)'0';
 
   return decimal;

} /* end pdbg_readdecimal */
/***********************************************************************/
/* Read a hexadecimal value from the  input string */

static sint32 pdbg_readhex(char *ptr, sint32 defaultValue)
{
   char    c;
   sint32  hex = 0;
   boolean found = FALSE;

   while (!isspace(*ptr)) ptr++;
   while (isspace(*ptr))  ptr++;
   while (TRUE) {

      c = toupper(*ptr);
      if ((c >= '0') && (c <= '9')) {
	 hex = ((hex << 4) | ((sint32)c - (sint32)'0'));
	 found = TRUE;
      } /* end if */
      else if ((c >= 'A') && (c <= 'F')) {
	 hex = ((hex << 4) | ((sint32)c - (sint32)'A' + 10));
	 found = TRUE;
      } /* end else if */
      else {
         if (found)
	    return hex;
	 else
            return defaultValue;
      } /* end else */
      ptr++;

   } /* end while */

} /* end pdbg_readhex */

/***********************************************************************/
/* Print the disassembled P-Code at PC */

static void pdbg_programstatus(struct pexec_s *st)
{
   (void)pdbg_printpcode(st, st->pc, 1);
   (void)pdbg_printstack(st, st->sp, 2);
   pdbg_printregisters(st);

} /* end pdbg_programstatus */

/***********************************************************************/
/* Print the disassembled P-Code at PC */

static addrType pdbg_printpcode(struct pexec_s *st, addrType PC, sint16 nItems)
{
  OPTYPE op;
  addrType pCodeSize;
  ubyte *address;

  for (; ((PC < st->maxpc) && (nItems > 0)); nItems--)
    {
      address = &st->ispace[PC];

      op.op     = *address++;
      op.arg1   = 0;
      op.arg2   = 0;
      pCodeSize = 1;
      printf("PC:%04x  %02x", PC, op.op);

      if ((op.op & o8) != 0)
	{
	  op.arg1 = *address++;
	  printf("%02x", op.arg1);
	  pCodeSize++;
	} /* end if */
      else 
	printf("..");

      if ((op.op & o16) != 0)
	{
	  op.arg2  = ((*address++) << 8);
	  op.arg2 |= *address++;
	  printf("%04x", op.arg2);
	  pCodeSize += 2;
	} /* end if */
      else
	printf("....");

      /* The disassemble it to stdout */

      printf("  ");
      insn_DisassemblePCode(stdout, &op);

      /* Get the address of the next P-Code */

      PC += pCodeSize;

    } /* end for */

  return PC;

} /* end pdbg_printpcode */

/***********************************************************************/
/* Print the stack value at SP */

static addrType pdbg_printstack(struct pexec_s *st, addrType SP, sint16 nItems)
{
  sint32 iSP;

  if ((st->sp < st->stacksize) && (SP <= st->sp))
    {
      iSP = BTOISTACK(SP);
      printf("SP:%04x  %04x\n", SP, st->dstack.i[iSP]);

      for (iSP--, SP -= BPERI, nItems--;
	   ((iSP >= 0) && (nItems > 0));
	   iSP--, SP -= BPERI, nItems--)
	printf("   %04x  %04x\n", SP, st->dstack.i[iSP] & 0xffff);
    } /* end if */
  else
    {
      printf("SP:%04x  BAD\n", SP);
    } /* end else */

  return SP;
} /* end pdbg_printstack */

/***********************************************************************/
/* Print the base register */

static void pdbg_printregisters(struct pexec_s *st)
{
   if (st->fp <= st->sp)
      printf("FP:%04x ", st->fp);

   printf("CSP:%04x\n", st->csp);

} /* end pdbg_printregisters */

/***********************************************************************/
/* Print the traceArray */

static void pdbg_printtracearray(struct pexec_s *st)
{
   int nPrinted;
   int index;

   index = traceIndex + TRACE_ARRAY_SIZE - numTracePoints;
   if (index >= TRACE_ARRAY_SIZE)
     index -= TRACE_ARRAY_SIZE;

   for (nPrinted = 0; nPrinted < numTracePoints; nPrinted++) {

      printf("SP:%04x  %04x  ",
         traceArray[ index ].SP, traceArray[ index ].TOS); 

      /* Print the instruction executed at this traced address */
      (void)pdbg_printpcode(st, traceArray[ index ].PC, 1);

      /* Index to the next trace entry */
      if (++index >= TRACE_ARRAY_SIZE)
	 index = 0;

   } /* end for */

} /* end pdbg_printtracearray */

/***********************************************************************/
/* Add a breakpoint to the breakpoint array */

static void pdbg_addbreakpoint(addrType PC)
{
  int i;

  /* Is there room for another breakpoint? */

  if (numBreakPoints < MAX_BREAK_POINTS)
    {
      /* Yes..Check if the breakpoint already exists */

      for (i = 0; i < numBreakPoints; i++)
	{
	  if (breakPoint[i] == PC)
	    {
	      /* It is already set.  Return without doing anything */

	      return;
	    }
	}

      /* The breakpoint is not already set -- set it */

      breakPoint[numBreakPoints++] = PC;
    } /* end if */

} /* end pdbg_addbreakpoint */

/***********************************************************************/
/* Remove a breakpoint from the breakpoint array */

static void pdbg_deletebreakpoint(sint16 bpNumber)
{
   if ((bpNumber >= 1) && (bpNumber <= numBreakPoints)) {

      for (; (bpNumber < numBreakPoints); bpNumber++)
         breakPoint[bpNumber-1] = breakPoint[bpNumber];
 
      numBreakPoints--;

   } /* end if */

} /* end pdbg_deletebreakpoint */

/***********************************************************************/
/* Print the breakpoint array */

static void pdbg_printbreakpoints(struct pexec_s *st)
{
   int i;
   printf("BP:#  Address  P-Code\n");
   for (i = 0; i < numBreakPoints; i++)
     {
       printf("BP:%d  ", (i+1));
       (void)pdbg_printpcode(st, breakPoint[i], 1);
     } /* end for */

} /* end pdbg_printbreakpoints */

/***********************************************************************/
/* Check if a breakpoint is set at the current value of program counter.
 * If so, print the instruction and stop execution. */

static void pdbg_checkbreakpoint(struct pexec_s *st)
{
  uint16 bpIndex;

  /* Check for a user breakpoint */

  for (bpIndex = 0;
       ((bpIndex < numBreakPoints) && (!stopExecution));
       bpIndex++)
    {
      if (breakPoint[bpIndex] == st->pc)
	{
	  printf("Breakpoint #%d -- Execution Stopped\n", (bpIndex+1));
	  stopExecution = TRUE;
	  return;
	} /* end if */
    } /* end for */

} /* end pdbg_checkbreakpoint */

/***********************************************************************/
/* Initialize Debugger variables */

static void pdbg_initdebugger(void)
{
   stopExecution    = FALSE;
   displayLoc       = 0;
   K                = 0;
   traceIndex       = 0;
   numTracePoints   = 0;
}

/***********************************************************************/
/* This function executes the P-Code program until a stopping condition
 * is encountered. */

static void pdbg_debugpcode(struct pexec_s *st)
{
   uint16 errno;

   do {
      /* Trace the next instruction execution */

      traceArray[traceIndex].PC  = st->pc;
      traceArray[traceIndex].SP  = st->sp;
      if (st->sp < st->stacksize)
	 traceArray[traceIndex].TOS = st->dstack.i[BTOISTACK(st->sp)];
      else
	 traceArray[traceIndex].TOS = 0;

      if (++traceIndex >= TRACE_ARRAY_SIZE)
	 traceIndex = 0;
      if (numTracePoints < TRACE_ARRAY_SIZE)
         numTracePoints++;

      /* Execute the instruction */

      errno = pexec(st);

      /* Check for exceptional stopping conditions */

      if (errno != eNOERROR)
	{
	  if (errno == eEXIT)
	    printf("Normal Termination\n");
	  else
	    printf("Runtime error 0x%02x -- Execution Stopped\n", errno);
	  stopExecution = TRUE;
	} /* end if */

      /* Check for normal stopping conditions */

      if (!stopExecution)
	{
	  /* Check for attempt to execute code outside of legal range */

	  if (st->pc >= st->maxpc)
	    stopExecution = TRUE;

	  /* Check for a temporary breakpoint */

	  else if ((untilPoint > 0) && (untilPoint == st->pc))
	    stopExecution = TRUE;

	  /* Check if there is a breakpoint at the next instruction */

	  else if (numBreakPoints > 0)
	    pdbg_checkbreakpoint(st);
	}

   } while (!stopExecution);

} /* end pdbg_debugpcode */

/***********************************************************************/
