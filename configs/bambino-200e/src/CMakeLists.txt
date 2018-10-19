############################################################################
# configs/bambino-200e/src/Makefile
#
#   Copyright (C) 2012 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#           Alan Carvalho de Assis acassis@gmail.com [nuttx] <nuttx@googlegroups.com>
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

list(APPEND CSRCS lpc43_boot.c lpc43_ssp.c)

if("${CONFIG_SENSORS_MAX31855}" STREQUAL "y")
	list(APPEND CSRCS lpc43_max31855.c)
endif()

if("${CONFIG_LIB_BOARDCTL}" STREQUAL "y")
	list(APPEND CSRCS lpc43_appinit.c)
endif()

if("${CONFIG_ARCH_FPU}" STREQUAL "y")
	list(APPEND CSRCS lpc43_ostest.c)
endif()

if("${CONFIG_ARCH_LEDS}" STREQUAL "y")
	list(APPEND CSRCS lpc43_autoleds.c)
else()
	list(APPEND CSRCS lpc43_userleds.c)
endif()

if("${CONFIG_ARCH_BUTTONS}" STREQUAL "y")
	list(APPEND CSRCS lpc43_buttons.c)
endif()

if("${CONFIG_TIMER}" STREQUAL "y")
	list(APPEND CSRCS lpc43_timer.c)
endif()

if("${CONFIG_USBMSC}" STREQUAL "y")
	list(APPEND CSRCS lpc43_usbmsc.c)
endif()

if("${CONFIG_SPIFI_LIBRARY}" STREQUAL "y")
CFLAGS += -DEUROBIRD
SPIFI_LIB = spifi_lib
list(APPEND CSRCS $(SPIFI_LIB)/spifi_rom_api.c)
#list(APPEND CSRCS $(SPIFI_LIB)/amic.c)
#list(APPEND CSRCS $(SPIFI_LIB)/atmel.c)
#list(APPEND CSRCS $(SPIFI_LIB)/chi.c)
#list(APPEND CSRCS $(SPIFI_LIB)/eon.c)
#list(APPEND CSRCS $(SPIFI_LIB)/esmt.c)
#list(APPEND CSRCS $(SPIFI_LIB)/esmt.c)
#list(APPEND CSRCS $(SPIFI_LIB)/giga.c)
#list(APPEND CSRCS $(SPIFI_LIB)/macronix.c)
#list(APPEND CSRCS $(SPIFI_LIB)/numonyx.c)
	list(APPEND CSRCS $(SPIFI_LIB)/spansion.c)
#list(APPEND CSRCS $(SPIFI_LIB)/sst.c)
	list(APPEND CSRCS $(SPIFI_LIB)/winbond.c)
endif()

nuttx_add_library(board ${CSRCS})
