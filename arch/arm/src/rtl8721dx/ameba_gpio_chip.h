/****************************************************************************
 * arch/arm/src/rtl8721dx/ameba_gpio_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8721DX_AMEBA_GPIO_CHIP_H
#define __ARCH_ARM_SRC_RTL8721DX_AMEBA_GPIO_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip GPIO parameters for the shared driver
 * (arch/arm/src/common/ameba/ameba_gpio.c).  The driver logic, pin encoding
 * and fwlib API are identical across every Ameba ARM chip, but the port
 * count, the per-port NVIC vectors and the RCC gate bits are not.  Each chip
 * supplies its own <ameba_gpio_chip.h> on the include path (arch/.../chip);
 * the common driver sizes its tables and wires its vectors from the macros
 * below, so nothing IC-specific is left in common.
 *
 * RTL8721Dx (KM4) breaks out two GPIO ports, A and B, each a 32-pin bank
 * with its own interrupt vector.
 */

/* Number of GPIO ports (banks) this chip exposes. */

#define AMEBA_GPIO_NPORTS      2

/* NVIC vector for each port, as an initialiser indexed by port number
 * (0 = A, 1 = B).  Its width must match AMEBA_GPIO_NPORTS.
 */

#define AMEBA_GPIO_PORT_IRQS   { RTL8721DX_IRQ_GPIOA, RTL8721DX_IRQ_GPIOB }

/* APBPeriph_GPIO / APBPeriph_GPIO_CLOCK (sysreg_lsys.h): the peripheral and
 * clock bits RCC_PeriphClockCmd() gates for the GPIO block.  On RTL8721Dx
 * both are ((1 << 30) | (1 << 4)); this value differs on amebasmart, which
 * is why it lives here rather than in the shared driver.
 */

#define AMEBA_APBPERIPH_GPIO   (((uint32_t)1 << 30) | ((uint32_t)1 << 4))

#endif /* __ARCH_ARM_SRC_RTL8721DX_AMEBA_GPIO_CHIP_H */
