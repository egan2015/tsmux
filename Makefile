VERSION := 0.0.1
OS ?= $(shell uname -s)
CWARNFLAGS := -Wall
CFLAGS := $(CWARNFLAGS) -O2 -I./build/include 
LDFLAGS:= -L./build/lib 
LDLIBS := -lpthread \
		  ./build/lib/libdvbpsi.a

#prefix := /usr

tsmux_BIN := tsmux
tsmux_SRC := main.c \
			 block.c \
			 pes.c \
			 csa.c	\
			 tsmux.c

tsmux_OBJ := main.o \
			 block.o \
			 pes.o \
			 csa.o \
			 tsmux.o


.PHONY: all clean
all: $(tsmux_BIN)


clean:
	$(RM) $(tsmux_OBJ)
	$(RM) $(tsmux_BIN)
$(tsmux_BIN) : $(tsmux_OBJ)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)


