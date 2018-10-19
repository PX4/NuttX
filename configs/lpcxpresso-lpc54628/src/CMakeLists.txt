############################################################################
# configs/lpcxpresso-lpc54628/src/Makefile
#
#   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

list(APPEND CSRCS lpc54_boot.c lpc54_bringup.c lpc54_userleds.c lpc54_lcd.c)

if("${CONFIG_ARCH_LEDS}" STREQUAL "y")
	list(APPEND CSRCS lpc54_autoleds.c)
endif()

if("${CONFIG_ARCH_BUTTONS}" STREQUAL "y")
	list(APPEND CSRCS lpc54_buttons.c)
endif()

if("${CONFIG_LIB_BOARDCTL}" STREQUAL "y")
	list(APPEND CSRCS lpc54_appinit.c)
endif()

if("${CONFIG_LPC54_HAVE_I2C_MASTER}" STREQUAL "y")
	list(APPEND CSRCS lpc54_i2c.c)
endif()

if("${CONFIG_SYSTEM_I2CTOOL}" STREQUAL "y")
	list(APPEND CSRCS lpc54_i2ctool.c)
endif()

if("${CONFIG_INPUT_FT5X06}" STREQUAL "y")
	list(APPEND CSRCS lpc54_ft5x06.c)
endif()

if("${CONFIG_LPC54_EMC}" STREQUAL "y")
if("${CONFIG_LPC54_EMC_DYNAMIC}" STREQUAL "y")
	list(APPEND CSRCS lpc54_sdram.c)
endif()
endif()

nuttx_add_library(board ${CSRCS})
