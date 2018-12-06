# Makefile
# ==============================================================================
# 07.02.2002 cmc :  Created based on sample from Kevin Nickels.
# 07.15.2004 cmc :  Updated and applied to sn_sim project.
# 08.05.2004 cmc :  Added tags to removed files in distclean, added "doc" target
# 12.16.2004 cmc :  Remove output files with distclean.
# ------------------------------------------------------------------------------
#  TODO: Figure out why LD won't link C++ object files with standard args.
# ==============================================================================

EPUCKLIBROOT = ../..

# ==============================================================================
# Local variables
# ------------------------------------------------------------------------------
#  first, find all the pertinent filenames
ASMSRCS = $(wildcard *.s)
CSRCS = $(wildcard *.c)
SRCS = $(CSRCS) $(ASMSRCS)
HDRS = $(wildcard *.h) $(wildcard *.inc)
OBJS = $(addsuffix .o,$(basename $(SRCS)))

CTAGS    = ctags
CC       = pic30-elf-gcc
LD 	 = pic30-elf-ld
ASM	 = pic30-elf-as
CFLAGS   = -g -Wall -mcpu=30F6014A 
CPPFLAGS = -I$(EPUCKLIBROOT) -I$(EPUCKLIBROOT)/std_microchip/support/h -I$(EPUCKLIBROOT)/std_microchip/include -I$(EPUCKLIBROOT)/uart -I$(EPUCKLIBROOT)/motor_led -I$(EPUCKLIBROOT)/I2C -I$(EPUCKLIBROOT)/codec -I$(EPUCKLIBROOT)/a_d 


# ==============================================================================
# Dependencies & rules
# ------------------------------------------------------------------------------
all: $(OBJS)

%.o: %.s
	$(ASM) -g -I$(EPUCKLIBROOT)/std_microchip/support/inc -p=30F6014A $< -o $@
# ==============================================================================
# Clean up directory
# ------------------------------------------------------------------------------
.PHONY:	clean
clean:	
	- $(RM) $(OBJS) *~ core.* *.rpo

.PHONY: distclean
distclean: clean
	- $(RM) $(DEPS) tags *.a *.so $(OUTPUT)
# ==============================================================================


# ==============================================================================
# make tags files for vim
# ------------------------------------------------------------------------------
tags:	$(SRCS) $(HDRS)
	$(CTAGS) $(SRCS) $(HDRS)
# ==============================================================================

# ==============================================================================
# end of Makefile
