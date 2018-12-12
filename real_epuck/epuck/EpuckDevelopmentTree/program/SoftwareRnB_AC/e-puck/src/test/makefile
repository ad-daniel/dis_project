# EXE MAKEFILE
#------------------------------
ROOT = ../..
EXE_TARGET = ircomTest
EPUCKLIBROOT = ../../../../../library
PIC30ELF = /usr/local/pic30-elf/pic30-elf

LIBS = -lIrcom -lBtcom -lE-puck -lm
INCS =
OPTS = -Wall -O3


# don't modify what is below ...
#------------------------------

ASM_SRCS 	= $(wildcard *.s)
C_SRCS 	= $(wildcard *.c)
SRCS 		= $(C_SRCS) $(ASM_SRCS)
HDRS 		= $(wildcard *.h) $(wildcard *.inc)
OBJS 		= $(addsuffix .o,$(basename $(SRCS)))

CC	  		= pic30-elf-gcc
AS	  		= pic30-elf-as
BIN2HEX  	= pic30-elf-bin2hex
AR			= pic30-elf-ar
RANLIB		= pic30-elf-ranlib
STRIP		= pic30-elf-strip

CC_FLAGS 	= -mcpu=30f6014a -I$(PIC30ELF)/include -I$(EPUCKLIBROOT)/std_microchip/support/h -I$(EPUCKLIBROOT)/std_microchip/include
AS_FLAGS 	= -I$(PIC30ELF)/support/inc -I$(EPUCKLIBROOT)/std_microchip/support/inc -p=30f6014a
LD_FLAGS 	= -Wl,--heap=512,-L$(EPUCKLIBROOT)/std_microchip/lib,-T$(EPUCKLIBROOT)/std_microchip/support/gld/p30f6014a.gld,-mpic30_elf32


all : $(EXE_TARGET).hex

$(EXE_TARGET).hex : $(EXE_TARGET).elf
	$(BIN2HEX) $(EXE_TARGET).elf
	if [ ! -d $(ROOT)/bin ]; then mkdir $(ROOT)/bin; fi
	cp $@ $(ROOT)/bin/
	make clean

$(EXE_TARGET).elf : $(OBJS) 
#	$(STRIP) $(OBJS) 
	$(CC) -o $@ $? $(LD_FLAGS) -L$(ROOT)/lib $(LIBS)

%.o : %.c %.h
	$(CC) $(CC_FLAGS) $(OPTS) $(INCS) -I$(ROOT)/include/ -c $< -o $(<:.c=.o)

%.o : %.c
	$(CC) $(CC_FLAGS) $(OPTS) $(INCS) -I$(ROOT)/include/ -c $< -o $(<:.c=.o)

%.o : %.s
	$(AS) -o $(<:.s=.o) $(AS_FLAGS) $(INCS) -I$(ROOT)/include/ $<


.PHONY : clean
clean : 
	rm -f *~ *.o *.a *.bak *.hex *.elf $(LIB_TARGET) core gmon.out  




