############################################################################
# configs/sama5d4-ek/src/Makefile
#
#   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

list(APPEND CSRCS sam_boot.c)

if("${CONFIG_SAMA5_SPI0}" STREQUAL "y")
	list(APPEND CSRCS sam_spi.c)
else()
if("${CONFIG_SAMA5_SPI1}" STREQUAL "y")
	list(APPEND CSRCS sam_spi.c)
endif()
endif()

if("${CONFIG_SAMA5_DDRCS}" STREQUAL "y")
	list(APPEND CSRCS sam_sdram.c)
endif()

if("${CONFIG_SAMA5_EBICS3_NAND}" STREQUAL "y")
	list(APPEND CSRCS sam_nandflash.c)
endif()

if("${CONFIG_SAMA5D4EK_DRAM_MAIN}" STREQUAL "y")
	list(APPEND CSRCS dram_main.c)
endif()

if("${CONFIG_SAMA5D4EK_AT25_MAIN}" STREQUAL "y")
	list(APPEND CSRCS at25_main.c)
endif()

if("${CONFIG_MTD_AT25}" STREQUAL "y")
if("${CONFIG_SAMA5_SPI0}" STREQUAL "y")
	list(APPEND CSRCS sam_at25.c)
endif()
endif()

if("${CONFIG_FS_AUTOMOUNTER}" STREQUAL "y")
	list(APPEND CSRCS sam_automount.c)
endif()

if("${CONFIG_AUDIO_WM8904}" STREQUAL "y")
if("${CONFIG_SAMA5_TWI0}" STREQUAL "y")
if("${CONFIG_SAMA5_SSC0}" STREQUAL "y")
	list(APPEND CSRCS sam_wm8904.c)
endif()
endif()
endif()

if("${CONFIG_SAMA5_TWI0}" STREQUAL "y")
	list(APPEND CSRCS sam_pmic.c)
endif()

if("${CONFIG_AUDIO_NULL}" STREQUAL "y")
	list(APPEND CSRCS sam_audio_null.c)
endif()

if("${CONFIG_SAMA5_HSMCI0}" STREQUAL "y")
	list(APPEND CSRCS sam_hsmci.c)
else()
if("${CONFIG_SAMA5_HSMCI1}" STREQUAL "y")
	list(APPEND CSRCS sam_hsmci.c)
endif()
endif()

if("${CONFIG_SAMA5_UHPHS}" STREQUAL "y")
	list(APPEND CSRCS sam_usb.c)
else()
if("${CONFIG_SAMA5_UDPHS}" STREQUAL "y")
	list(APPEND CSRCS sam_usb.c)
endif()
endif()

if("${CONFIG_SAMA5_EMACB}" STREQUAL "y")
	list(APPEND CSRCS sam_ethernet.c)
endif()

if("${CONFIG_INPUT_MXT}" STREQUAL "y")
	list(APPEND CSRCS sam_maxtouch.c)
endif()

if("${CONFIG_LIB_BOARDCTL}" STREQUAL "y")
	list(APPEND CSRCS sam_appinit.c sam_bringup.c)
else()
if("${CONFIG_BOARD_INITIALIZE}" STREQUAL "y")
	list(APPEND CSRCS sam_bringup.c)
endif()
endif()

if("${CONFIG_ADC}" STREQUAL "y")
	list(APPEND CSRCS sam_adc.c)
endif()

if("${CONFIG_PWM}" STREQUAL "y")
	list(APPEND CSRCS sam_pwm.c)
endif()

if("${CONFIG_USBMSC}" STREQUAL "y")
	list(APPEND CSRCS sam_usbmsc.c)
endif()

if("${CONFIG_ARCH_FPU}" STREQUAL "y")
	list(APPEND CSRCS sam_ostest.c)
endif()

if("${CONFIG_ARCH_LEDS}" STREQUAL "y")
	list(APPEND CSRCS sam_autoleds.c)
else()
	list(APPEND CSRCS sam_userleds.c)
endif()

if("${CONFIG_ARCH_BUTTONS}" STREQUAL "y")
	list(APPEND CSRCS sam_buttons.c)
endif()

nuttx_add_library(board ${CSRCS})
