/****************************************************************************
 * boards/arm/rtl8721dx/pke8721daf/src/rtl8721dx_gpio.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <syslog.h>

#include <nuttx/ioexpander/gpio.h>

#include "ameba_gpio.h"
#include "rtl8721dx_pke8721daf.h"

#ifdef CONFIG_AMEBA_GPIO

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* One entry per GPIO pin exposed to NuttX.  The pins below are examples used
 * by the `gpio` (examples/gpio) test -- adjust them to match your board's
 * wiring.  They are registered in order as /dev/gpio0, /dev/gpio1, ...
 */

struct rtl8721dx_gpio_s
{
  uint8_t pin;                  /* AMEBA_PA()/AMEBA_PB() pin encoding */
  enum gpio_pintype_e pintype;  /* Input, output or interrupt */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rtl8721dx_gpio_s g_gpio_pins[] =
{
  { AMEBA_PB(18), GPIO_OUTPUT_PIN    },  /* /dev/gpio0: output    */
  { AMEBA_PB(19), GPIO_INPUT_PIN     },  /* /dev/gpio1: input     */
  { AMEBA_PB(20), GPIO_INTERRUPT_PIN },  /* /dev/gpio2: interrupt */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8721dx_gpio_initialize
 *
 * Description:
 *   Register the board's GPIO pins with the NuttX GPIO upper half.
 *
 ****************************************************************************/

int rtl8721dx_gpio_initialize(void)
{
  int ret;
  size_t i;

  for (i = 0; i < nitems(g_gpio_pins); i++)
    {
      ret = ameba_gpio_register(i, g_gpio_pins[i].pin,
                                g_gpio_pins[i].pintype);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: ameba_gpio_register(/dev/gpio%zu) failed: %d\n",
                 i, ret);
          return ret;
        }
    }

  return OK;
}

#endif /* CONFIG_AMEBA_GPIO */
