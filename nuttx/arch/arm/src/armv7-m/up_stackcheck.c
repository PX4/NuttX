#include <nuttx/config.h>

#include <stdint.h>

#include "up_arch.h"
#include "nvic.h"

void    __cyg_profile_func_enter(void *func, void *caller) __attribute__((naked, no_instrument_function));
void    __cyg_profile_func_exit(void *func, void *caller)  __attribute__((naked, no_instrument_function));
void    __stack_overflow_trap(void) __attribute__((naked, no_instrument_function));

void
__stack_overflow_trap(void)
{
  /* if we get here, the stack has overflowed */

  /* force hard fault */
  uint32_t regval;

  /* Set up for the system reset, retaining the priority group from the
   * the AIRCR register.
   */

  regval  = getreg32(NVIC_INTCTRL);
  regval |= NVIC_INTCTRL_NMIPENDSET;
  putreg32(regval, NVIC_INTCTRL);

    /* trap */
    //asm ( "b .");
}

void
__cyg_profile_func_enter(void *func, void *caller)
{
    asm volatile (
            "   mrs r2, ipsr        \n" /* Check whether we are in interrupt mode */
            "   cmp r2, #0          \n" /* since we don't switch r10 on interrupt entry, we */
            "   bne 2f              \n" /* can't detect overflow of the interrupt stack. */
            "                       \n"
            "   sub r2, sp, #68     \n" /* compute stack pointer as though we just stacked a full frame */
            "   mrs r1, control     \n" /* Test CONTROL.FPCA to see whether we also need room for the FP */
            "   tst r1, #4          \n" /* context. */
            "   beq 1f              \n"
            "   sub r2, r2, #136    \n" /* subtract FP context frame size */
            "1:                     \n"
            "   cmp r2, r10         \n" /* compare stack with limit */
            "   bgt 2f              \n" /* stack is above limit and thus OK */
            "   b __stack_overflow_trap\n"
            "2:                     \n"
            "   bx lr               \n"
    );
}

void
__cyg_profile_func_exit(void *func, void *caller)
{
    asm volatile("bx lr");
}
