############################################################################
# configs/same70-xplained/src/Makefile
#
#   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
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

if("${CONFIG_SAMV7_SDRAMC}" STREQUAL "y")
	list(APPEND CSRCS sam_sdram.c)
endif()

if("${CONFIG_LIB_BOARDCTL}" STREQUAL "y")
	list(APPEND CSRCS sam_appinit.c sam_bringup.c)
elseif("${CONFIG_BOARD_INITIALIZE}" STREQUAL "y")
	list(APPEND CSRCS sam_bringup.c)
endif()

if("${CONFIG_ARCH_LEDS}" STREQUAL "y")
	list(APPEND CSRCS sam_autoleds.c)
else()
	list(APPEND CSRCS sam_userleds.c)
endif()

if("${CONFIG_ARCH_BUTTONS}" STREQUAL "y")
	list(APPEND CSRCS sam_buttons.c)
endif()

if("${CONFIG_SAMV7_HSMCI0}" STREQUAL "y")
	list(APPEND CSRCS sam_hsmci.c)
endif()

if("${CONFIG_SAMV7_EMAC}" STREQUAL "y")
	list(APPEND CSRCS sam_ethernet.c)
endif()

if("${CONFIG_SAMV7_USBDEVHS}" STREQUAL "y")
	list(APPEND CSRCS sam_usbdev.c)
endif()

if("${CONFIG_SAMV7_SPI}" STREQUAL "y")
	list(APPEND CSRCS sam_spi.c)
endif()

if("${CONFIG_USBMSC}" STREQUAL "y")
	list(APPEND CSRCS sam_usbmsc.c)
endif()

if("${CONFIG_SAMV7_MCAN}" STREQUAL "y")
	list(APPEND CSRCS sam_mcan.c)
endif()

if("${CONFIG_MTD_CONFIG}" STREQUAL "y")
if("${CONFIG_SAMV7_TWIHS0}" STREQUAL "y")
if("${CONFIG_MTD_AT24XX}" STREQUAL "y")
	list(APPEND CSRCS sam_at24config.c)
endif()
endif()
endif()

if(("${CONFIG_SAMV7_DAC0}" STREQUAL "y") OR ("${CONFIG_SAMV7_DAC1}" STREQUAL "y"))
	list(APPEND CSRCS sam_dac.c)
endif()

if("${CONFIG_IEEE802154_MRF24J40}" STREQUAL "y")
	list(APPEND CSRCS sam_mrf24j40.c)
endif()

if("${CONFIG_IEEE802154_XBEE}" STREQUAL "y")
	list(APPEND CSRCS sam_xbee.c)
endif()

nuttx_add_library(board ${CSRCS})
