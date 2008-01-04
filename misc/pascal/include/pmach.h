/**********************************************************************
 * File:        pmach.h
 * Description: Definitions associated with the simulated P-Machine
 * Author:      Gregory Nutt
 * Modified:
 **********************************************************************/

#ifndef __PMACH_H
#define __PMACH_H

/**********************************************************************
 * Definitions
 **********************************************************************/

#define MIN_PROGRAM_COUNTER   0

/**********************************************************************
 * Global Type Definitions
 **********************************************************************/

typedef uint16 uStackType; /* Stack values are 16-bits in length */
typedef sint16 sStackType;
typedef uint16 addrType;   /* Addresses are 16-bits in length */
typedef uint16 levelType;  /* Limits to MAXUINT16 levels */
typedef uint16 labelType;  /* Limits to MAXUINT16 labels */

#define BPERI         2
#define ITOBSTACK(i)  ((i) << 1)
#define BTOISTACK(i)  ((i) >> 1)
#define ROUNDBTOI(i)  (((i) + 1) >> 1)
#define STACKALIGN(i) (((i) + 1) & ~1)

union stack_u
{
  uStackType *i;
  ubyte      *b;
};
typedef union stack_u stackType;

/**********************************************************************
 * Global Variables
 **********************************************************************/

/* This is the emulated P-Machine stack (D-Space) */

extern stackType stack;

/* This is the emulated P-Machine instruction space */

extern ubyte *iSpace;

/* These are the emulated P-Machine registers:
 *
 * baseReg: Base Register of the current stack frame.  Holds the address
 *     of the base of the stack frame of the current block.
 * topOfStringStack: The current top of the stack used to manage string
 *     storage
 * topOfStack: The Pascal stack pointer
 * programCounter: Holds the current p-code location
 */

extern addrType baseReg;
extern addrType topOfStringStack;
extern addrType topOfStack;
extern addrType programCounter;

/* Configuration variables
 *
 * readOnlyData: Stack address of read-only data
 * bottomOfStack: Initial Value of the stack pointer
 * sizeOfStack: Total allocated size of the Pascal stack
 * maxProgramCounter: Address of last valid P-Code
 * entryPoint: This is the address where execution begins.
 */

extern addrType readOnlyData;
extern addrType bottomOfStack;
extern addrType sizeOfStack;
extern addrType maxProgramCounter;
extern uint32   entryPoint;

#endif /* __PMACH_H */
