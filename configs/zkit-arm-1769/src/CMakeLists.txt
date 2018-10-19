############################################################################
# configs/zkit-arm-1769/src/Makefile
#
#   Copyright (C) 2013 Zilogic Systems. All rights reserved.
#   Author: BabuSubashChandar <code@zilogic.com>
#
# Based on configs/lpcxpresso-lpc1768/src/Makefile
#
#   Copyright (C) 201, 2016 Gregory Nutt. All rights reserved.
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

list(APPEND CSRCS lpc17_boot.c lpc17_leds.c lpc17_ssp.c lpc17_buttons.c lpc17_lcd.c)

if("${CONFIG_LIB_BOARDCTL}" STREQUAL "y")
	list(APPEND CSRCS lpc17_appinit.c)
endif()

if("${CONFIG_USBMSC}" STREQUAL "y")
	list(APPEND CSRCS lpc17_usbmsc.c)
endif()

if("${CONFIG_ADC}" STREQUAL "y")
	list(APPEND CSRCS lpc17_adc.c)
endif()

if("${CONFIG_DAC}" STREQUAL "y")
	list(APPEND CSRCS lpc17_dac.c)
endif()

if("${CONFIG_CAN}" STREQUAL "y")
	list(APPEND CSRCS lpc17_can.c)
endif()

if("${CONFIG_LPC17_SPI}" STREQUAL "y")
	list(APPEND CSRCS lpc17_spi.c)
endif()

nuttx_add_library(board ${CSRCS})
