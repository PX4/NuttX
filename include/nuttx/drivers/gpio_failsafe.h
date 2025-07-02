#ifndef __INCLUDE_NUTTX_DRIVERS_GPIO_FAILSAFE_H
#define __INCLUDE_NUTTX_DRIVERS_GPIO_FAILSAFE_H

#include <nuttx/config.h>
#include <nuttx/irq.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_DRVR_GPIO_FAILSAFE
int gpio_failsafe_initialize(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_DRIVERS_GPIO_FAILSAFE_H */
