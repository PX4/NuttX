CC	= gcc
CFLAGS	= -g -Wall -IZ80 -DLSB_FIRST=1 -DDEBUG=1 -DJUMPZ80

LD	= gcc
LDFLAGS	= 

SRCS	= main.c Debug.c Z80.c
OBJS	= $(SRCS:.c=.o)

BIN	= z80sim

VPATH	= Z80

all: $(BIN)
default: $(BIN)

$(OBJS): %.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@


Z80: Z80-081707.zip
	unzip Z80-081707.zip

z80sim: Z80 $(OBJS)
	$(LD) $(LDFLAGS) $(OBJS) -o $@

clean:
	@rm -f $(BIN) *.o *.rel *.asm *.lst *.sym *.adb *~

distclean: clean
	rm -rf Z80

