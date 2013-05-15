############################################################################
# Makefile
#
#   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

# Things you might want to override from the make command line

CC ?= gcc
LD ?= ld
OPT ?= -g
INSTALLDIR ?= /usr/local/bin

CFLAGS = $(OPT)
CFLAGS += -I.
CFLAGS += -Wall -Wstrict-prototypes -Wshadow

# Control build verbosity

ifeq ($(V),1)
  export Q :=
else
  export Q := @
endif


# Files

OCSRCS1 = osmocon.c tpu_debug.c
OCSRCS2 = msgb.c serial.c panic.c talloc.c timer.c select.c rbtree.c sercomm.c
OCSRCS  = $(OCSRCS1) $(OCSRCS2)

OLSRCS1 = osmoload.c
OLSRCS2 = msgb.c panic.c talloc.c timer.c select.c rbtree.c crc16.c
OLSRCS  = $(OLSRCS1) $(OLSRCS2)

CMNSRCS = msgb.c serial.c panic.c talloc.c timer.c select.c rbtree.c sercomm.c crc16.c

SRCS = $(OCSRCS1) $(OLSRCS1) $(CMNSRCS)

OCOBJS = $(OCSRCS:.c=.o)
OLOBJS = $(OLSRCS:.c=.o)

OBJS = $(SRCS:.c=.o)

ifdef EXEET
.PHONY: all osmocon osmoload clean install
else
.PHONY: all clean install
endif

all: osmocon osmoload

$(OBJS): %.o: %.c
	$(Q) $(CC) -c $(CFLAGS) $< -o $@

osmocon$(EXEEXT): $(OCOBJS)
	$(Q) $(CC) $(OCOBJS) -o osmocon$(EXEEXT)

osmoload$(EXEEXT): $(OLOBJS)
	$(Q) $(CC) $(OLOBJS) -o osmoload$(EXEEXT)

ifdef EXEET
osmocon: osmocon$(EXEEXT)

osmoload$: osmoload$(EXEEXT)
endif

install: osmocon$(EXEEXT) osmoload$(EXEEXT)
	$(Q) install -m 755 osmocon$(EXEEXT) $(INSTALLDIR)/osmocon$(EXEEXT)
	$(Q) install -m 755 osmoload$(EXEEXT) $(INSTALLDIR)/osmoload$(EXEEXT)

clean:
	$(Q) rm -f osmocon$(EXEEXT) osmoload$(EXEEXT) *.o
