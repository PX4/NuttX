############################################################################
# arch/arm/src/lpc17xx/CMakeLists.txt
#
#   Copyright (C) 2010-2011, 2013-2015, 2018 Gregory Nutt. All rights reserved.
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

# The start-up, "head", file

HEAD_ASRC =

# Common ARM and Cortex-M3 files

CMN_UASRCS =
CMN_UCSRCS =

set(CMN_ASRCS)
list(APPEND CMN_ASRCS up_saveusercontext.S up_fullcontextrestore.S up_switchcontext.S)
list(APPEND CMN_ASRCS up_testset.S up_fetchadd.S vfork.S)

set(CMN_CSRCS)
list(APPEND CMN_ASRCS up_assert.c up_blocktask.c up_copyfullstate.c up_createstack.c)
list(APPEND CMN_CSRCS up_mdelay.c up_udelay.c up_exit.c up_initialize.c up_memfault.c)
list(APPEND CMN_CSRCS up_initialstate.c up_interruptcontext.c up_modifyreg8.c)
list(APPEND CMN_CSRCS up_modifyreg16.c up_modifyreg32.c up_releasepending.c)
list(APPEND CMN_CSRCS up_releasestack.c up_reprioritizertr.c up_schedulesigaction.c)
list(APPEND CMN_CSRCS up_sigdeliver.c up_stackframe.c up_trigger_irq.c)
list(APPEND CMN_CSRCS up_unblocktask.c up_usestack.c up_doirq.c up_hardfault.c)
list(APPEND CMN_CSRCS up_svcall.c up_checkstack.c up_vfork.c)

if("${CONFIG_ARMV7M_STACKCHECK}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_stackcheck.c)
endif()

if("${CONFIG_ARMV7M_LAZYFPU}" STREQUAL "y")
	list(APPEND CMN_ASRCS up_lazyexception.S)
else()
	list(APPEND CMN_ASRCS up_exception.S)
endif()
list(APPEND CMN_CSRCS up_vectors.c)

if("${CONFIG_ARCH_RAMVECTORS}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_ramvec_initialize.c up_ramvec_attach.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_mpu.c up_task_start.c up_pthread_start.c)
if(NOT "${CONFIG_DISABLE_SIGNALS}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_signal_dispatch.c)
list(APPEND CMN_UASRCS up_signal_handler.S)
endif()
endif()

# Use of common/up_etherstub.c is deprecated.  The preferred mechanism is to
# use CONFIG_NETDEV_LATEINIT=y to suppress the call to up_netinitialize() in
# up_initialize().  Then this stub would not be needed.

if("${CONFIG_NET}" STREQUAL "y")
if(NOT "${CONFIG_LPC17_ETHERNET}" STREQUAL "y")
	list(APPEND CMN_CSRCS up_etherstub.c)
endif()
endif()

if("${CONFIG_ARCH_FPU}" STREQUAL "y")
	list(APPEND CMN_ASRCS up_fpu.S)
list(APPEND CMN_CSRCS up_copyarmstate.c)
endif()

# Required LPC17xx files


set(CHIP_CSRCS)
list(APPEND CHIP_CSRCS lpc17_allocateheap.c lpc17_clockconfig.c lpc17_clrpend.c)
list(APPEND CHIP_CSRCS lpc17_gpio.c lpc17_i2c.c lpc17_irq.c lpc17_lowputc.c)
list(APPEND CHIP_CSRCS lpc17_serial.c lpc17_spi.c lpc17_ssp.c lpc17_start.c)

# Configuration-dependent LPC17xx files

if(NOT "${CONFIG_ARCH_IDLE_CUSTOM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc17_idle.c)
endif()

if(NOT "${CONFIG_SCHED_TICKLESS}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc17_timerisr.c)
endif()

if("${CONFIG_BUILD_PROTECTED}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc17_userspace.c lpc17_mpuinit.c)
endif()

if("${CONFIG_LPC17_EMC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc17_emc.c)
endif()

if("${CONFIG_LPC17_GPIOIRQ}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc17_gpioint.c)
endif()

if("${CONFIG_DEBUG_GPIO_INFO}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc17_gpiodbg.c)
endif()

if("${CONFIG_LPC17_LCD}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc17_lcd.c)
endif()

if("${CONFIG_USBDEV}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc17_usbdev.c)
endif()

if("${CONFIG_USBHOST}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc17_usbhost.c)
endif()

if("${CONFIG_LPC17_GPDMA}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc17_gpdma.c)
endif()

if("${CONFIG_LPC17_SDCARD}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc17_sdcard.c)
endif()

if("${CONFIG_NET}" STREQUAL "y")
if("${CONFIG_LPC17_ETHERNET}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc17_ethernet.c)
endif()
endif()

if("${CONFIG_CAN}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc17_can.c)
endif()

if("${CONFIG_LPC17_ADC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc17_adc.c)
endif()

if("${CONFIG_LPC17_DAC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc17_dac.c)
endif()

if("${CONFIG_LPC17_RTC}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc176x_rtc.c)
endif()

if("${CONFIG_LPC17_PWM1}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc17_pwm.c)
endif()

if("${CONFIG_LPC17_MCPWM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc17_mcpwm.c)
endif()

if("${CONFIG_LPC17_TMR0}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc17_timer.c)
endif()

if("${CONFIG_MTD_PROGMEM}" STREQUAL "y")
	list(APPEND CHIP_CSRCS lpc17_progmem.c)
endif()
