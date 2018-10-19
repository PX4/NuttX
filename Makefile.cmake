############################################################################
# Makefile
#
#   Copyright (C) 2012, 2018 Gregory Nutt. All rights reserved.
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

# Parsing
# --------------------------------------------------------------------
# assume 1st argument passed is the main target, the
# rest are arguments to pass to the makefile generated
# by cmake in the subdirectory
FIRST_ARG := $(firstword $(MAKECMDGOALS))
ARGS := $(wordlist 2,$(words $(MAKECMDGOALS)),$(MAKECMDGOALS))
j ?= 4

NINJA_BIN := ninja
ifndef NO_NINJA_BUILD
	NINJA_BUILD := $(shell $(NINJA_BIN) --version 2>/dev/null)

	ifndef NINJA_BUILD
		NINJA_BIN := ninja-build
		NINJA_BUILD := $(shell $(NINJA_BIN) --version 2>/dev/null)
	endif
endif

ifdef NINJA_BUILD
	NUTTX_CMAKE_GENERATOR := Ninja
	NUTTX_MAKE := $(NINJA_BIN)

	ifdef VERBOSE
		NUTTX_MAKE_ARGS := -v
	else
		NUTTX_MAKE_ARGS :=
	endif
else
	ifdef SYSTEMROOT
		# Windows
		NUTTX_CMAKE_GENERATOR := "MSYS\ Makefiles"
	else
		NUTTX_CMAKE_GENERATOR := "Unix\ Makefiles"
	endif
	NUTTX_MAKE = $(MAKE)
	NUTTX_MAKE_ARGS = -j$(j) --no-print-directory
endif

SRC_DIR := $(shell dirname "$(realpath $(lastword $(MAKEFILE_LIST)))")

# Functions
# --------------------------------------------------------------------
# describe how to build a cmake config
define cmake-build
+@$(eval NUTTX_CONFIG = $(1))
+@$(eval BUILD_DIR = "$(SRC_DIR)"/build/$(NUTTX_CONFIG)$(BUILD_DIR_SUFFIX))
+@if [ $(NUTTX_CMAKE_GENERATOR) = "Ninja" ] && [ -e $(BUILD_DIR)/Makefile ]; then rm -rf $(BUILD_DIR); fi
+@if [ ! -e $(BUILD_DIR)/CMakeCache.txt ]; then mkdir -p $(BUILD_DIR) && cd $(BUILD_DIR) && cmake "$(SRC_DIR)" -G"$(NUTTX_CMAKE_GENERATOR)" $(CMAKE_ARGS) -DCONFIG=$(NUTTX_CONFIG) || (rm -rf $(BUILD_DIR)); fi
+@$(NUTTX_MAKE) -C $(BUILD_DIR) $(NUTTX_MAKE_ARGS) $(ARGS)
endef

# Get a list of all config targets boards/*/*.cmake
ALL_CONFIG_TARGETS := $(shell find configs -maxdepth 3 -mindepth 3 -name 'defconfig' -print | sed -e 's/configs\///' | sed -e 's/\/defconfig//' | sort)

# ADD CONFIGS HERE
# --------------------------------------------------------------------
#  Do not put any spaces between function arguments.

# All targets.
$(ALL_CONFIG_TARGETS):
	$(call cmake-build,$@)

.PHONY: $(ALL_CONFIG_TARGETS)

# Check
# --------------------------------------------------------------------

check: \
	freedom-k28f/nsh \
	freedom-k64f/netnsh \
	freedom-k64f/nsh \
	freedom-k66f/netnsh \
	freedom-k66f/nsh \
	stm32f4discovery/audio \
	stm32f4discovery/canard \
	stm32f4discovery/hciuart \
	stm32f4discovery/ipv6 \
	stm32f4discovery/max31855 \
	stm32f4discovery/module \
	stm32f4discovery/netnsh \
	stm32f4discovery/nsh \
	stm32f4discovery/pm \
	stm32f4discovery/pseudoterm \
	stm32f4discovery/usbmsc \
	stm32f4discovery/usbnsh \
	stm32f4discovery/xen1210 \
	stm3210e-eval/composite \
	stm3210e-eval/nsh \
	stm3210e-eval/usbmsc \
	stm3210e-eval/usbserial \
	stm3220g-eval/dhcpd \
	stm3220g-eval/nsh \
	stm3220g-eval/nsh2 \
	stm3220g-eval/telnetd \
	stm3240g-eval/dhcpd \
	stm3240g-eval/discover \
	stm3240g-eval/fb \
	stm3240g-eval/nsh \
	stm3240g-eval/nsh2 \
	stm3240g-eval/telnetd \
	stm32butterfly2/nsh \
	stm32butterfly2/nshnet \
	stm32butterfly2/nshusbhost \
	stm32f103-minimum/apds9960 \
	stm32f103-minimum/audio_tone \
	stm32f103-minimum/buttons \
	stm32f103-minimum/mcp2515 \
	stm32f103-minimum/nsh \
	stm32f103-minimum/rfid-rc522 \
	stm32f103-minimum/rgbled \
	stm32f103-minimum/usbnsh \
	stm32f103-minimum/userled \
	stm32f103-minimum/veml6070 \
	stm32f334-disco/nsh \
	stm32f3discovery/nsh \
	stm32f3discovery/usbnsh \
	stm32f411e-disco/nsh \
	stm32f429i-disco/extflash \
	stm32f429i-disco/highpri \
	stm32f429i-disco/nsh \
	stm32f429i-disco/usbmsc \
	stm32f429i-disco/usbnsh \
	stm32f746g-disco/fb \
	stm32f746g-disco/nsh \
	stm32f746-ws/nsh \
	stm32f769i-disco/nsh \
	stm32f769i-disco/netnsh \
	stm32l476-mdk/nsh \
	stm32_tiny/nsh \
	stm32_tiny/usbnsh \
	stm32vldiscovery/nsh \
	stm32f746g-disco/netnsh \
	sizes

sizes:
	@-find build -name nuttx -type f | sort | xargs size 2> /dev/null || :

.PHONY: check sizes

# Cleanup
# --------------------------------------------------------------------

clean:
	@rm -rf "$(SRC_DIR)"/build

distclean:
	@git clean -ff -x -d -e ".project" -e ".cproject" -e ".idea" -e ".settings" -e ".vscode"

.PHONY: clean distclean


# --------------------------------------------------------------------

# All other targets are handled by PX4_MAKE. Add a rule here to avoid printing an error.
%:
	$(if $(filter $(FIRST_ARG),$@), \
		$(error "$@ cannot be the first argument. Use '$(MAKE) help|list_config_targets' to get a list of all possible [configuration] targets."),@#)

#help:
#	@echo
#	@echo "Type 'make ' and hit the tab key twice to see a list of the available"
#	@echo "build configurations."
#	@echo

empty :=
space := $(empty) $(empty)

# Print a list of non-config targets (based on http://stackoverflow.com/a/26339924/1487069)
help:
	@echo "Usage: $(MAKE) <target>"
	@echo "Where <target> is one of:"
	@$(MAKE) -pRrq -f $(lastword $(MAKEFILE_LIST)) : 2>/dev/null | \
		awk -v RS= -F: '/^# File/,/^# Finished Make data base/ {if ($$1 !~ "^[#.]") {print $$1}}' | sort | \
		egrep -v -e '^[^[:alnum:]]' -e '^($(ALL_CONFIG_TARGETS))$$' -e '^(Makefile)'
	@echo
	@echo "Or, $(MAKE) <config_target> [<make_target(s)>]"
	@echo "Use '$(MAKE) list_config_targets' for a list of configuration targets."

# Print a list of all config targets.
list_config_targets:
	@for targ in $(ALL_CONFIG_TARGETS); do echo $$targ; done

