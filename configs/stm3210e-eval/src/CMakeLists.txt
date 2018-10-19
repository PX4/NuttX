############################################################################
# configs/stm3210e-eval/src/Makefile
#
#   Copyright (C) 2009-2010, 2012 Gregory Nutt. All rights reserved.
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

list(APPEND CSRCS stm32_boot.c stm32_bringup.c stm32_leds.c stm32_spi.c stm32_usbdev.c)

if("${CONFIG_STM32_FSMC}" STREQUAL "y")
	list(APPEND CSRCS stm32_lcd.c stm32_extcontext.c stm32_extmem.c stm32_selectnor.c)
	list(APPEND CSRCS stm32_deselectnor.c stm32_selectsram.c stm32_deselectsram.c)
	list(APPEND CSRCS stm32_selectlcd.c stm32_deselectlcd.c)
endif()

if("${CONFIG_LIB_BOARDCTL}" STREQUAL "y")
	list(APPEND CSRCS stm32_appinit.c)
endif()

if("${CONFIG_ADC}" STREQUAL "y")
	list(APPEND CSRCS stm32_adc.c)
endif()

if("${CONFIG_USBMSC}" STREQUAL "y")
	list(APPEND CSRCS stm32_usbmsc.c)
endif()

if("${CONFIG_USBDEV_COMPOSITE}" STREQUAL "y")
	list(APPEND CSRCS stm32_composite.c)
endif()

if("${CONFIG_LM75_I2C}" STREQUAL "y")
	list(APPEND CSRCS stm32_lm75.c)
endif()

if("${CONFIG_CAN}" STREQUAL "y")
	list(APPEND CSRCS stm32_can.c)
endif()

if("${CONFIG_ARCH_CUSTOM_PMINIT}" STREQUAL "y")
	list(APPEND CSRCS stm32_pm.c)
endif()

if("${CONFIG_ARCH_BUTTONS}" STREQUAL "y")
	list(APPEND CSRCS stm32_buttons.c)

if("${CONFIG_PM_BUTTONS}" STREQUAL "y")
	list(APPEND CSRCS stm32_pmbuttons.c)
endif()
endif()

if("${CONFIG_DJOYSTICK}" STREQUAL "y")
	list(APPEND CSRCS stm32_djoystick.c)
endif()

if("${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CSRCS stm32_idle.c)
endif()

nuttx_add_library(board ${CSRCS})
