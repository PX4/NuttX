/****************************************************************************
 * arch/arm/src/imxrt/imxrt_enet.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_ENET_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_ENET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

#include "hardware/imxrt_enet.h"

#ifdef CONFIG_IMXRT_ENET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Definitions for use with imxrt_phy_boardinitialize */

#define EMAC_INTF 0

/****************************************************************************
 * Public Functions Prototypes
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
 * Function: imxrt_netinitialize
 *
 * Description:
 *   Initialize the Ethernet controller and driver
 *
 * Input Parameters:
 *   intf - In the case where there are multiple EMACs, this value identifies
 *          which EMAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_LATEINIT
int imxrt_netinitialize(int intf);
#endif

/****************************************************************************
 * Function: imxrt_phy_boardinitialize
 *
 * Description:
 *   Some boards require specialized initialization of the PHY before it can
 *   be used.  This may include such things as configuring GPIOs, resetting
 *   the PHY, etc.  If CONFIG_IMXRT_ENET_PHYINIT is defined in the
 *   configuration then the board specific logic must provide
 *   imxrt_phyinitialize();  The i.MX RT Ethernet driver will call this
 *   function one time before it first uses the PHY.
 *
 * Input Parameters:
 *   intf - Always zero for now.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_ENET_PHYINIT
int imxrt_phy_boardinitialize(int intf);
#endif

/****************************************************************************
 * Function: imxrt_eth_has_phy
 *
 * Description:
 *   Report whether a PHY is attached to the ENET MII/RMII interface.  The
 *   driver provides a weak default that returns true.  A board with a
 *   direct MAC-to-MAC link overrides it and returns false; the driver then
 *   skips MDIO and configures the MAC for 100 Mbps full duplex.
 *
 * Input Parameters:
 *   intf - Always zero for now.
 *
 * Returned Value:
 *   true if a PHY is attached; false otherwise.
 *
 ****************************************************************************/

bool imxrt_eth_has_phy(int intf);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_IMXRT_ENET */
#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_ENET_H */
