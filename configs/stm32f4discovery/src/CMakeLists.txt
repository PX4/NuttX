############################################################################
# configs/stm32f4discovery/src/Makefile
#
#   Copyright (C) 2011-2013, 2015-2016, 2018 Gregory Nutt. All rights
#     reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

set(CSRCS)

list(APPEND CSRCS stm32_boot.c stm32_bringup.c stm32_spi.c)

if("${CONFIG_ARCH_LEDS}" STREQUAL "y")
	list(APPEND CSRCS stm32_autoleds.c)
else()
	list(APPEND CSRCS stm32_userleds.c)
endif()

if("${CONFIG_SCHED_CRITMONITOR}" STREQUAL "y")
	list(APPEND CSRCS stm32_critmon.c)
endif()

if("${CONFIG_AUDIO_CS43L22}" STREQUAL "y")
	list(APPEND CSRCS stm32_cs43l22.c)
endif()

if("${CONFIG_ARCH_BUTTONS}" STREQUAL "y")
	list(APPEND CSRCS  stm32_buttons.c)
endif()

if("${CONFIG_CAN}" STREQUAL "y")
	list(APPEND CSRCS stm32_can.c)
endif()

if("${CONFIG_STM32_OTGFS}" STREQUAL "y")
	list(APPEND CSRCS stm32_usb.c)
endif()

if("${CONFIG_SENSORS_BH1750FVI}" STREQUAL "y")
	list(APPEND CSRCS stm32_bh1750fvi.c)
endif()

if("${CONFIG_SENSORS_BMP180}" STREQUAL "y")
	list(APPEND CSRCS stm32_bmp180.c)
endif()

if("${CONFIG_SENSORS_MLX90614}" STREQUAL "y")
	list(APPEND CSRCS stm32_mlx90614.c)
endif()

if("${CONFIG_LCD_ST7567}" STREQUAL "y")
	list(APPEND CSRCS stm32_st7567.c)
endif()

if("${CONFIG_INPUT_NUNCHUCK}" STREQUAL "y")
	list(APPEND CSRCS stm32_nunchuck.c)
endif()

if("${CONFIG_SENSORS_MAX31855}" STREQUAL "y")
	list(APPEND CSRCS stm32_max31855.c)
endif()

if("${CONFIG_STM32F4DISCO_LIS3DSH}" STREQUAL "y")
	list(APPEND CSRCS stm32_lis3dsh.c)
endif()

if("${CONFIG_SENSORS_MAX6675}" STREQUAL "y")
	list(APPEND CSRCS stm32_max6675.c)
endif()

if("${CONFIG_LCD_MAX7219}" STREQUAL "y")
	list(APPEND CSRCS stm32_max7219.c)
endif()

if("${CONFIG_LCD_ST7032}" STREQUAL "y")
	list(APPEND CSRCS stm32_st7032.c)
endif()

if("${CONFIG_PCA9635PW}" STREQUAL "y")
	list(APPEND CSRCS stm32_pca9635.c)
endif()

if("${CONFIG_STM32_SDIO}" STREQUAL "y")
	list(APPEND CSRCS stm32_sdio.c)
endif()

if("${CONFIG_STM32_ETHMAC}" STREQUAL "y")
	list(APPEND CSRCS stm32_ethernet.c)
endif()

if("${CONFIG_LEDS_MAX7219}" STREQUAL "y")
	list(APPEND CSRCS stm32_max7219_leds.c)
endif()

if("${CONFIG_RGBLED}" STREQUAL "y")
	list(APPEND CSRCS stm32_rgbled.c)
endif()

if("${CONFIG_PWM}" STREQUAL "y")
	list(APPEND CSRCS stm32_pwm.c)
endif()

if("${CONFIG_SENSORS_QENCODER}" STREQUAL "y")
	list(APPEND CSRCS stm32_qencoder.c)
endif()

if("${CONFIG_SENSORS_XEN1210}" STREQUAL "y")
	list(APPEND CSRCS stm32_xen1210.c)
endif()

if("${CONFIG_LIB_BOARDCTL}" STREQUAL "y")
	list(APPEND CSRCS stm32_appinit.c)
endif()

if("${CONFIG_ARCH_CUSTOM_PMINIT}" STREQUAL "y")
	list(APPEND CSRCS stm32_pm.c)
endif()

if("${CONFIG_PM_BUTTONS}" STREQUAL "y")
	list(APPEND CSRCS stm32_pmbuttons.c)
endif()

if("${CONFIG_SENSORS_ZEROCROSS}" STREQUAL "y")
	list(APPEND CSRCS stm32_zerocross.c)
endif()

if("${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CSRCS stm32_idle.c)
endif()

if("${CONFIG_STM32_FSMC}" STREQUAL "y")
	list(APPEND CSRCS stm32_extmem.c)

if("${CONFIG_LCD_SSD1289}" STREQUAL "y")
	list(APPEND CSRCS stm32_ssd1289.c)
endif()
endif()

if("${CONFIG_LCD_SSD1351}" STREQUAL "y")
	list(APPEND CSRCS stm32_ssd1351.c)
endif()

if("${CONFIG_LCD_UG2864AMBAG01}" STREQUAL "y")
	list(APPEND CSRCS stm32_ug2864ambag01.c)
endif()

if("${CONFIG_LCD_UG2864HSWEG01}" STREQUAL "y")
	list(APPEND CSRCS stm32_ug2864hsweg01.c)
endif()

if("${CONFIG_EXAMPLES_OSTEST}" STREQUAL "y")
	list(APPEND CSRCS stm32_ostest.c)
endif()

if("${CONFIG_TIMER}" STREQUAL "y")
	list(APPEND CSRCS stm32_timer.c)
endif()

if("${CONFIG_STM32_HCIUART}" STREQUAL "y")
if("${CONFIG_BLUETOOTH_UART}" STREQUAL "y")
	list(APPEND CSRCS stm32_hciuart.c)
endif()
endif()

if("${CONFIG_STM32_ROMFS}" STREQUAL "y")
	list(APPEND CSRCS stm32_romfs_initialize.c)
endif()

if("${CONFIG_BOARDCTL_UNIQUEID}" STREQUAL "y")
	list(APPEND CSRCS stm32_uid.c)
endif()

if("${CONFIG_USBMSC}" STREQUAL "y")
	list(APPEND CSRCS stm32_usbmsc.c)
endif()

if(NOT "${CONFIG_STM32_ETHMAC}" STREQUAL "y")
if("${CONFIG_NETDEVICES}" STREQUAL "y")
	list(APPEND CSRCS stm32_netinit.c)
endif()
endif()

nuttx_add_library(board ${CSRCS})
