############################################################################
# configs/sama5d3x-ek/src/Makefile
#
#   Copyright (C) 2013-2014, 2016 Gregory Nutt. All rights reserved.
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

if("${CONFIG_SAMA5_EBICS0_NOR}" STREQUAL "y")
	list(APPEND CSRCS sam_norflash.c)
endif()

if("${CONFIG_SAMA5_EBICS3_NAND}" STREQUAL "y")
	list(APPEND CSRCS sam_nandflash.c)
endif()

if("${CONFIG_SAMA5D3xEK_NOR_MAIN}" STREQUAL "y")
	list(APPEND CSRCS nor_main.c)
endif()

if("${CONFIG_MTD_AT25}" STREQUAL "y")
if("${CONFIG_SAMA5_SPI0}" STREQUAL "y")
	list(APPEND CSRCS sam_at25.c)
endif()
endif()

if("${CONFIG_MTD_AT24XX}" STREQUAL "y")
if("${CONFIG_SAMA5_TWI0}" STREQUAL "y")
	list(APPEND CSRCS sam_at24.c)
endif()
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

if("${CONFIG_SAMA5_EMACA}" STREQUAL "y")
	list(APPEND CSRCS sam_ethernet.c)
else()
if("${CONFIG_SAMA5_GMAC}" STREQUAL "y")
	list(APPEND CSRCS sam_ethernet.c)
endif()
endif()

if("${CONFIG_AUDIO_WM8904}" STREQUAL "y")
if("${CONFIG_SAMA5_TWI0}" STREQUAL "y")
if("${CONFIG_SAMA5_SSC0}" STREQUAL "y")
	list(APPEND CSRCS sam_wm8904.c)
endif()
endif()
endif()

if("${CONFIG_LIB_BOARDCTL}" STREQUAL "y")
	list(APPEND CSRCS sam_appinit.c)
endif()

if("${CONFIG_ADC}" STREQUAL "y")
	list(APPEND CSRCS sam_adc.c)
endif()

if("${CONFIG_PWM}" STREQUAL "y")
	list(APPEND CSRCS sam_pwm.c)
endif()

if("${CONFIG_CAN}" STREQUAL "y")
	list(APPEND CSRCS sam_can.c)
endif()

if("${CONFIG_AUDIO_I2SCHAR}" STREQUAL "y")
if("${CONFIG_SAMA5_SSC0}" STREQUAL "y")
	list(APPEND CSRCS sam_i2schar.c)
else()
if("${CONFIG_SAMA5_SSC1}" STREQUAL "y")
	list(APPEND CSRCS sam_i2schar.c)
endif()
endif()
endif()

if("${CONFIG_USBMSC}" STREQUAL "y")
	list(APPEND CSRCS sam_usbmsc.c)
endif()

if("${CONFIG_ARCH_FPU}" STREQUAL "y")
	list(APPEND CSRCS sam_ostest.c)
endif()

if("${CONFIG_SAMA5_TSD}" STREQUAL "y")
	list(APPEND CSRCS sam_touchscreen.c)
endif()

if("${CONFIG_VIDEO_OV2640}" STREQUAL "y")
	list(APPEND CSRCS sam_ov2640.c)
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
