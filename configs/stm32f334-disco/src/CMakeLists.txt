############################################################################
# configs/stm32f334-disco/src/Makefile
#
#   Copyright (C) 2017 Gregory Nutt. All rights reserved.
#   Author: Mateusz Szafoni <raiden00@railab.me>
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

list(APPEND CSRCS stm32_boot.c)

if("${CONFIG_ARCH_LEDS}" STREQUAL "y")
	list(APPEND CSRCS stm32_autoleds.c)
else()
	list(APPEND CSRCS stm32_userleds.c)
endif()

if("${CONFIG_ARCH_BUTTONS}" STREQUAL "y")
	list(APPEND CSRCS stm32_buttons.c)
endif()

if("${CONFIG_LIB_BOARDCTL}" STREQUAL "y")
	list(APPEND CSRCS stm32_appinit.c)
endif()

if("${CONFIG_CAN}" STREQUAL "y")
	list(APPEND CSRCS stm32_can.c)
endif()

if("${CONFIG_PWM}" STREQUAL "y")
	list(APPEND CSRCS stm32_pwm.c)
endif()

if("${CONFIG_SPI}" STREQUAL "y")
	list(APPEND CSRCS stm32_spi.c)
endif()

if("${CONFIG_TIMER}" STREQUAL "y")
	list(APPEND CSRCS stm32_timer.c)
endif()

if("${CONFIG_DRIVERS_POWERLED}" STREQUAL "y")
	list(APPEND CSRCS stm32_powerled.c)
endif()

if("${CONFIG_DRIVERS_SMPS}" STREQUAL "y")
	list(APPEND CSRCS stm32_smps.c)
endif()

nuttx_add_library(board ${CSRCS})
