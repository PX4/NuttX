############################################################################
# configs/freedom-k64f/src/Makefile
#
#   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

list(APPEND CSRCS k64_boot.c k64_spi.c)

if("${CONFIG_ARCH_LEDS}" STREQUAL "y")
	list(APPEND CSRCS k64_autoleds.c)
else()
	list(APPEND CSRCS k64_userleds.c)
endif()

if("${CONFIG_ARCH_BUTTONS}" STREQUAL "y")
	list(APPEND CSRCS k64_buttons.c)
endif()

if("${CONFIG_LIB_BOARDCTL}" STREQUAL "y")
	list(APPEND CSRCS k64_appinit.c k64_bringup.c)
elseif("${CONFIG_BOARD_INITIALIZE}" STREQUAL "y")
	list(APPEND CSRCS k64_bringup.c)
endif()

if("${CONFIG_KINETIS_SDHC}" STREQUAL "y")
	list(APPEND CSRCS k64_sdhc.c)
if("${CONFIG_FS_AUTOMOUNTER}" STREQUAL "y")
	list(APPEND CSRCS k64_automount.c)
endif()
endif()

if("${CONFIG_USBDEV}" STREQUAL "y")
	list(APPEND CSRCS k64_usbdev.c)
endif()

if("${CONFIG_USBMSC}" STREQUAL "y")
	list(APPEND CSRCS k64_usbmsc.c)
endif()

if("${CONFIG_PWM}" STREQUAL "y")
	list(APPEND CSRCS k64_pwm.c)
endif()

if("${CONFIG_BOARDCTL_UNIQUEID}" STREQUAL "y")
	list(APPEND CSRCS k64_uid.c)
endif()

nuttx_add_library(board ${CSRCS})
