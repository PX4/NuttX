//
// Created by zhengweiqian on 9/4/2023.
//


#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>

#include <net/ethernet.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/wdog.h>
#include <nuttx/sdio.h>
#include <nuttx/net/arp.h>
#include <nuttx/wireless/ieee80211/ieee80211.h>
#include <nuttx/wireless/ieee80211/esp32_sdio.h>

int esp32_sdio_initialize(int minor, FAR struct sdio_dev_s *dev)
{

    return 0;
}