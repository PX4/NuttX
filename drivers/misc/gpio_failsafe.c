/****************************************************************************
 * drivers/misc/gpio_failsafe.c
 *
 * Implements a simple hardware failsafe trigger using a GPIO interrupt.
 * When the configured GPIO input detects a rising edge, this driver
 * stops the system watchdog timer using WDIOC_STOP.
 *
 * This driver relies on board-specific logic to configure and enable
 * the GPIO interrupt.  The board must provide the functions
 * board_gpio_failsafe_init() and board_gpio_failsafe_enable().
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

#include <fcntl.h>
#include <unistd.h>
#include <debug.h>
#include <sys/ioctl.h>

#include <nuttx/wqueue.h>
#include <nuttx/timers/watchdog.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>

#ifdef CONFIG_DRVR_GPIO_FAILSAFE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_WATCHDOG_DEVPATH
#  define CONFIG_WATCHDOG_DEVPATH "/dev/watchdog0"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct work_s g_failsafe_work;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void gpio_failsafe_work(void *arg)
{
  int fd;

  finfo("Failsafe triggered\n");

  fd = open(CONFIG_WATCHDOG_DEVPATH, O_RDONLY);
  if (fd >= 0)
    {
      ioctl(fd, WDIOC_STOP, 0);
      close(fd);
    }
}

static int gpio_failsafe_isr(int irq, void *context, void *arg)
{
  /* Schedule the work on the high priority worker thread.  */

  return work_queue(HPWORK, &g_failsafe_work, gpio_failsafe_work, NULL, 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int gpio_failsafe_initialize(void)
{
  int ret;

  /* Configure the GPIO and attach the interrupt via board logic.  */

  ret = board_gpio_failsafe_init(gpio_failsafe_isr, NULL);
  if (ret < 0)
    {
      ferr("ERROR: board_gpio_failsafe_init failed: %d\n", ret);
      return ret;
    }

  /* Enable the interrupt */

  ret = board_gpio_failsafe_enable(true);
  if (ret < 0)
    {
      ferr("ERROR: board_gpio_failsafe_enable failed: %d\n", ret);
    }

  return ret;
}

#endif /* CONFIG_DRVR_GPIO_FAILSAFE */

