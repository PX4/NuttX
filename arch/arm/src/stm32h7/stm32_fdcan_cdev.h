/****************************************************************************
 * arch/arm/src/stm32h7/stm32_fdcan_cdev.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_FDCAN_CDEV_H
#define __ARCH_ARM_SRC_STM32H7_STM32_FDCAN_CDEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/stm32_fdcan.h"

#if defined(CONFIG_CAN) && (defined(CONFIG_STM32H7_CAN1) || \
    defined(CONFIG_STM32H7_CAN2) || defined(CONFIG_STM32H7_CAN3))

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: stm32_caninitialize
 *
 * Description:
 *   Initialize the selected CAN port as a character device
 *
 * Input Parameters:
 *   port - CAN port number (1, 2, or 3)
 *
 * Returned Value:
 *   Valid CAN device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct can_dev_s;
struct can_dev_s *stm32_caninitialize(int port);

/****************************************************************************
 * Name: stm32_fdcan_loopback
 *
 * Description:
 *   Enable or disable internal loopback mode for testing.
 *   In loopback mode, transmitted messages are internally routed to RX
 *   without going to the physical bus.
 *
 * Input Parameters:
 *   port   - The FDCAN port number (1 or 2)
 *   enable - true to enable loopback, false to disable
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int stm32_fdcan_loopback(int port, bool enable);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_CAN && (CONFIG_STM32H7_CAN1 || CONFIG_STM32H7_CAN2 || CONFIG_STM32H7_CAN3) */
#endif /* __ARCH_ARM_SRC_STM32H7_STM32_FDCAN_CDEV_H */
