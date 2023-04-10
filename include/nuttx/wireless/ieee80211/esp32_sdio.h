//
// Created by zhengweiqian on 9/4/2023.
//

#ifndef PX4_AUTOPILOT_ESP32_SDIO_H
#define PX4_AUTOPILOT_ESP32_SDIO_H
/****************************************************************************
 * include/nuttx/wireless/ieee80211/bcmf_sdio.h
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

#include <nuttx/mmcsd.h>

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bcmf_sdio_initialize
 *
 * Description:
 *   Initialize Broadcom FullMAC driver.
 *
 * Input Parameters:
 *   minor - zero based minor device number which is unique
 *           for each wlan device.
 *   dev   - SDIO device used to communicate with the wlan chip
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int esp32_sdio_initialize(int minor, FAR struct sdio_dev_s *dev);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif //PX4_AUTOPILOT_ESP32_SDIO_H
