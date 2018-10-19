############################################################################
# configs/mikroe-stm32f4/src/Makefile
#
#   Copyright (C) 2011-2013, 2016 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
#   Updates: 04/15/2013 - Ken Pettit
#               - Modifications for port to Mikroe for STM32F4
#               - Added support for LCD, Serial Flash, SD Card
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

list(APPEND CSRCS stm32_boot.c stm32_spi.c)

if("${CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG}" STREQUAL "y")
	list(APPEND CSRCS stm32_clockconfig.c)
endif()

if("${CONFIG_STM32_OTGFS}" STREQUAL "y")
	list(APPEND CSRCS stm32_usb.c)
endif()

if("${CONFIG_PWM}" STREQUAL "y")
	list(APPEND CSRCS stm32_pwm.c)
endif()

if("${CONFIG_SENSORS_QENCODER}" STREQUAL "y")
	list(APPEND CSRCS stm32_qencoder.c)
endif()

if("${CONFIG_LIB_BOARDCTL}" STREQUAL "y")
	list(APPEND CSRCS stm32_appinit.c)
endif()

if("${CONFIG_ARCH_CUSTOM_PMINIT}" STREQUAL "y")
	list(APPEND CSRCS stm32_pm.c)
endif()

if("${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CSRCS stm32_idle.c)
endif()

if("${CONFIG_STM32_FSMC}" STREQUAL "y")
	list(APPEND CSRCS stm32_extmem.c)
endif()

if("${CONFIG_INPUT}" STREQUAL "y")
	list(APPEND CSRCS stm32_touchscreen.c)
endif()

if("${CONFIG_LCD_MIO283QT2}" STREQUAL "y")
	list(APPEND CSRCS stm32_mio283qt2.c)
endif()

if("${CONFIG_LCD_MIO283QT9A}" STREQUAL "y")
	list(APPEND CSRCS stm32_mio283qt9a.c)
endif()

if("${CONFIG_AUDIO_VS1053}" STREQUAL "y")
	list(APPEND CSRCS stm32_vs1053.c)
endif()

nuttx_add_library(board ${CSRCS})
