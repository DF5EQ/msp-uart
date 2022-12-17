# name of the program to build
PROGNAME := msp-uart
DEVICE   := msp430fr5969                  # according to msp430-gcc mmcu option

CSRC := $(shell find -name *.c)           # list of all .c files
HSRC := $(shell find -name *.h)           # list of all .h files
SSRC := $(shell find -name *.s)           # list of all .s files (sources)
OBJS := $(CSRC:.c=.o) $(SSRC:.s=.o)       # list of all .o files
ASMS := $(CSRC:.c=.S)                     # list of all .S files (generated)
DEPS := $(CSRC:.c=.d)                     # list of all .d files
BAKS := $(shell find -name *.*~*)         # list of all backup files
CDIR := $(sort $(dir $(CSRC)))            # directories of all .c files
HDIR := $(sort $(dir $(HSRC)))            # directories of all .h files
SDIR := $(sort $(dir $(SSRC)))            # directories of all .s files
IDIR := $(sort $(CDIR) $(HDIR))           # directories of all include files
INCS := $(foreach dir, $(IDIR), -I$(dir)) # flags for all include directories
LIBS :=                                   # library directories

# command and flags for dependencies
DP      = msp430-gcc -MM
DPFLAGS = -MG -MT $(basename $<).o -MT $(basename $<).d
# command and flags for C Compiler
CC      = msp430-gcc -c
CFLAGS  = -mmcu=$(DEVICE)
# command and flags for assembler
AS      = msp430-as
ASFLAGS = -mmcu=$(DEVICE)
# command and flags for linker
LD      = msp430-gcc
LDFLAGS = -mmcu=$(DEVICE) -Wl,-Map,$(basename $@).map
# command and flags for hex-file creator
HX      = msp430-objcopy
HXFLAGS = -O ihex
# command and flags for listing
LS      = msp430-objdump
LSFLAGS = -D -s
# command and flags for download
DL      = mspdebug 
DLFLAGS = tilib

# search path for targets and prerequisites
VPATH = $(IDIR)

# set only the needed suffixes
.SUFFIXES:
.SUFFIXES: .c .s .d .o .hex .elf

# implicite rule for making dependencies file from C file
%.d: %.c
	@echo "makefile: "$@
	@$(DP) $(DPFLAGS) $(INCS) -o $@ $< 

# implicite rule for making object file from C file
# as second output an assembler file is generated
%.o: %.c
	@echo "makefile: "$@
	@$(CC) $(CFLAGS) $(INCS) -o $@ $<
	@$(CC) $(CFLAGS) $(INCS) -S -o $(basename $@).S $<
 
# implicite rule for making object file from S file
%.o: %.s
	@echo "makefile: "$@
	@$(AS) $(ASFLAGS) -o $@ $<

# implicite rule for making elf file from object files
# as second output a listing file is generated
%.elf: $(OBJS)
	@echo "makefile: "$@
	@$(LD) $(LDFLAGS) $(OBJS) -o $@
	@$(LS) $(LSFLAGS) $@ > $(basename $@).lst

# implicite rule for making hex file from elf file
%.hex: %.elf
	@echo "makefile: "$@
	@$(HX) $(HXFLAGS) $< $@

# rule for making the target program
.PHONY: all
all: $(PROGNAME).hex
	@echo "makefile: all done"
	
# target and rule for running the program
.PHONY: run
run: $(PROGNAME).hex
	@echo "makefile: download"
	@$(DL) $(DLFLAGS) "prog $<"

# target and rule for cleaning up the directories
.PHONY: clean
clean:
	@echo "makefile: remove .d files"
	@rm -f $(DEPS)
	@echo "makefile: remove .o files"
	@rm -f $(OBJS)
	@echo "makefile: remove .S files (generated)"
	@rm -f $(ASMS)
	@echo "makefile: remove program"
	@rm -f $(PROGNAME).*
	@echo "makefile: remove backup files"
	@rm -f $(BAKS)

# prevent intermediate files from automatic deleting
.SECONDARY: $(PROGNAME).elf
.SECONDARY: $(OBJS)

# append all dependencies files
include $(DEPS)


