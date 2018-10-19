############################################################################
# configs/viewtool-stm32f107/src/Makefile
#
#   Copyright (C) 2013, 2017-2018 Gregory Nutt. All rights reserved.
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

list(APPEND CSRCS stm32_boot.c stm32_bringup.c stm32_leds.c stm32_spi.c)

if("${CONFIG_CAN}" STREQUAL "y")
	list(APPEND CSRCS stm32_can.c)
endif()

if("${CONFIG_MMCSD}" STREQUAL "y")
	list(APPEND CSRCS stm32_mmcsd.c)
endif()

if("${CONFIG_STM32_OTGFS}" STREQUAL "y") # F107
list(APPEND CSRCS stm32_usbdev.c)
else()
if("${CONFIG_STM32_USB}" STREQUAL "y") # F103
list(APPEND CSRCS stm32_usbdev.c)
endif()
endif()

if("${CONFIG_SENSORS_MPL115A}" STREQUAL "y")
	list(APPEND CSRCS stm32_mpl115a.c)
endif()

if("${CONFIG_INPUT_ADS7843E}" STREQUAL "y") # F103
list(APPEND CSRCS stm32_ads7843e.c)
endif()

if("${CONFIG_LCD_SSD1289}" STREQUAL "y") # F103
list(APPEND CSRCS stm32_ssd1289.c)
endif()

if("${CONFIG_LIB_BOARDCTL}" STREQUAL "y")
	list(APPEND CSRCS stm32_appinit.c)
endif()

if("${CONFIG_USBMSC}" STREQUAL "y")
	list(APPEND CSRCS stm32_usbmsc.c)
endif()

if("${CONFIG_ARCH_BUTTONS}" STREQUAL "y")
	list(APPEND CSRCS stm32_buttons.c)
endif()

if("${CONFIG_VIEWTOOL_HIGHPRI}" STREQUAL "y")
	list(APPEND CSRCS stm32_highpri.c)
endif()

if("${CONFIG_VIEWTOOL_FT80X_SPI1}" STREQUAL "y")
	list(APPEND CSRCS stm32_ft80x.c)
elseif("${CONFIG_VIEWTOOL_FT80X_SPI2}" STREQUAL "y")
	list(APPEND CSRCS stm32_ft80x.c)
endif()

if("${CONFIG_VIEWTOOL_MAX3421E_SPI1}" STREQUAL "y")
	list(APPEND CSRCS stm32_max3421e.c)
elseif("${CONFIG_VIEWTOOL_MAX3421E_SPI2}" STREQUAL "y")
	list(APPEND CSRCS stm32_max3421e.c)
endif()

nuttx_add_library(board ${CSRCS})
