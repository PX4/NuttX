############################################################################
# configs/lc823450-xgevk/src/Makefile
#
#  Copyright 2017 Sony Video & Sound Products Inc.
#  Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
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

list(APPEND CSRCS lc823450_boot.c lc823450_mux.c)

if("${CONFIG_LIB_BOARDCTL}" STREQUAL "y")
	list(APPEND CSRCS lc823450_appinit.c lc823450_bringup.c)
elseif("${CONFIG_BOARD_INITIALIZE}" STREQUAL "y")
	list(APPEND CSRCS lc823450_bringup.c)
endif()

if("${CONFIG_LC823450_SDIF}" STREQUAL "y")
	list(APPEND CSRCS lc823450_sdif.c)
endif()

if("${CONFIG_ADC}" STREQUAL "y")
	list(APPEND CSRCS lc823450_adc.c)
endif()

if("${CONFIG_USBMSC}" STREQUAL "y")
	list(APPEND CSRCS lc823450_usbmsc.c)
endif()

if("${CONFIG_SPI}" STREQUAL "y")
	list(APPEND CSRCS lc823450_spi.c)
endif()

if("${CONFIG_LCD_ST7565}" STREQUAL "y")
	list(APPEND CSRCS lc823450_st7565.c)
endif()

if("${CONFIG_BMA250}" STREQUAL "y")
	list(APPEND CSRCS lc823450_bma250.c)
endif()

if("${CONFIG_NETDEVICES}" STREQUAL "y")
	list(APPEND CSRCS lc823450_netinit.c)
endif()

if("${CONFIG_AUDIO_WM8776}" STREQUAL "y")
	list(APPEND CSRCS lc823450_wm8776.c)
endif()

if("${CONFIG_ARCH_LEDS}" STREQUAL "y")
	list(APPEND CSRCS lc823450_autoleds.c)
endif()

if("${CONFIG_LC823450_XGEVK_BTPOWER}" STREQUAL "y")
	list(APPEND CSRCS lc823450_bt.c)
endif()

nuttx_add_library(board ${CSRCS})
