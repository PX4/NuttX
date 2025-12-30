/****************************************************************************
 * Name: security_monitor.c
 *
 * Description:
 *   Minimal security monitor for STM32H7 (no MPU yet).
 *
 *   - Configure PC9 as a tamper GPIO input
 *   - Route PC9 to EXTI9
 *   - On rising edge on PC9, trigger a global failsafe hook
 *
 *   This file is intended to live in:
 *     nuttx/arch/arm/src/stm32h7/security_monitor.c
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>          /* getreg32/putreg32/modifyreg32 */
#include <nuttx/syslog/syslog.h>
#include <syslog.h>

#include "arm_internal.h"        /* ARMv7-M internal helpers (if needed) */
#include "mpu.h"                 /* ARMv7-M MPU helpers */

#include "stm32_gpio.h"          /* stm32_configgpio, GPIO_* macros */
#include "stm32_exti.h"          /* stm32_gpiosetevent */

#include "hardware/stm32_gpio.h"
#include "hardware/stm32_exti.h"
#include "hardware/stm32_syscfg.h"
#include "hardware/stm32_rcc.h"

#include "security_monitor.h"

/* -------------------------------------------------------------------------
 *  Configuration
 * ------------------------------------------------------------------------- */

/* Tamper GPIO: PC9 → EXTI9 */
#define SEC_GPIO_PIN            9
#define SEC_EXTI_BIT            (1u << SEC_GPIO_PIN)

/* Map EXTI9 to port C in SYSCFG_EXTICR3 */
#define SYSCFG_EXTICR3_EXTI9_PC (2u << SYSCFG_EXTICR3_EXTI9_SHIFT)

#if defined(STM32_EXTI_CPUPR1)
#  define SEC_EXTI_PR   STM32_EXTI_CPUPR1
#else
#  error "No suitable EXTI pending register macro found for this STM32H7 variant"
#endif

/* Interrupt mask register for core domain 1 */
#define SEC_EXTI_IMR   STM32_EXTI_CPUIMR1

/* Enable dev-only self test to intentionally trigger MemFault reset.
 * Define CONFIG_SECURITY_MONITOR_SELFTEST in your NuttX config to enable.
 */
#ifdef CONFIG_SECURITY_MONITOR_SELFTEST
#  define SEC_SELFTEST_ENABLED  1
#else
#  define SEC_SELFTEST_ENABLED  0
#endif

/* -------------------------------------------------------------------------
 *  Helper functions
 * ------------------------------------------------------------------------- */

/****************************************************************************
 * Name: sec_gpio_lock
 *
 * Description:
 *   Lock GPIO pins using the LCKR register sequence.
 *   This prevents accidental reconfiguration of the pins.
 *
 *   LCKR sequence:
 *   1. Write LCKR with LCK[15:0] set for pins to lock, LCKK=1
 *   2. Write LCKR with LCK[15:0] set for pins to lock, LCKK=0
 *   3. Write LCKR with LCK[15:0] set for pins to lock, LCKK=1
 *   4. Read LCKR to confirm lock
 *
 ****************************************************************************/

static bool sec_gpio_lock_pins(uintptr_t gpio_base, uint32_t pin_mask)
{
  const uintptr_t lckr = gpio_base + STM32_GPIO_LCKR_OFFSET;
  uint32_t v;

  /* Step 1: Write LCKK=1 */
  putreg32(pin_mask | GPIO_LCKK, lckr);

  /* Step 2: Write LCKK=0 */
  putreg32(pin_mask, lckr);

  /* Step 3: Write LCKK=1 */
  putreg32(pin_mask | GPIO_LCKK, lckr);

  /* Step 4: Read LCKR */
  (void)getreg32(lckr);

  /* Step 5: Read LCKR to verify lock */
  v = getreg32(lckr);

  /* Confirm: LCKK set + pins locked */
  return ((v & GPIO_LCKK) != 0) && ((v & (pin_mask & 0xFFFFu)) == (pin_mask & 0xFFFFu));
}

/* -------------------------------------------------------------------------
 *  ISR
 * ------------------------------------------------------------------------- */

/****************************************************************************
 * Name: security_isr
 *
 * Description:
 *   EXTI interrupt handler for the tamper GPIO (PC9 / EXTI9).
 *   Called by NuttX GPIO EXTI system when PC9 edge is detected.
 *
 ****************************************************************************/

static int security_isr(int irq, void *context, void *arg)
{
  /* Reset the system immediately */
  up_systemreset();

  return OK;
}

/* -------------------------------------------------------------------------
 *  Public API
 * ------------------------------------------------------------------------- */

/****************************************************************************
 * Name: security_monitor_init
 *
 * Description:
 *   Initialize the security monitor:
 *     - Configure PC9 as input (with pull-down)
 *     - Route PC9 to EXTI9
 *     - Configure EXTI9 to trigger on rising edge
 *     - Attach ISR and enable the EXTI5–9 IRQ
 *
 *   This should be called once during board bring-up, e.g. from up_initialize()
 *   or a board-specific init function after clocks and GPIO are ready.
 *
 ****************************************************************************/

void security_monitor_init(void)
{
  uint32_t pinset;
  int ret;

  /* Build GPIO pinset for PC9 as input with pull-down */
  pinset = (GPIO_INPUT | GPIO_PORTC | GPIO_PIN9 | GPIO_PULLDOWN);

  /* Configure PC9 GPIO */
  stm32_configgpio(pinset);

  /* Register EXTI interrupt using NuttX GPIO system
   * This properly handles the shared EXTI5-9 IRQ
   */
  ret = stm32_gpiosetevent(pinset, 
                           true,   /* rising edge */
                           false,  /* no falling edge */
                           false,  /* no event */
                           security_isr, 
                           NULL);

  /* Lock GPIO pin to prevent accidental reconfiguration */
  sec_gpio_lock_pins(STM32_GPIOC_BASE, (1u << SEC_GPIO_PIN));

  irq_attach(STM32_IRQ_MEMFAULT, security_isr, NULL);
  up_enable_irq(STM32_IRQ_MEMFAULT);

}
