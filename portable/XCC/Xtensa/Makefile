### Makefile to build the FreeRTOS library ###

# Build target (options: sim, board)

TARGET      = sim
SMALL       =

# Tools

CC          = xt-clang
AS          = xt-clang
AR          = xt-ar
ISS         = xt-run
XT_CORE     = $(patsubst %-params,%,$(notdir $(shell xt-clang --show-config=core)))
CONFIGDIR   = $(shell xt-clang --show-config=config)

# For platform-specific commands

include $(CONFIGDIR)/misc/hostenv.mk

# Source code and build locations

# Build with MPU support
ifeq ($(MPU),1)
MPUFLAGS    = -DportUSING_MPU_WRAPPERS=1  -DportALIGN_SECTIONS
endif

SRCROOT     = $(subst /,$(S),$(CURDIR))
TSTROOT     = $(abspath $(SRCROOT)$(S)..$(S)..$(S)..$(S)demos$(S)cadence$(S)sim$(SMALL))
BLDROOT     = $(TSTROOT)$(S)build
BLDDIR      = $(BLDROOT)$(S)$(XT_CORE)

FR_SRCDIR   = $(abspath $(SRCROOT)$(S)..$(S)..$(S)..$(S))
FR_SRCDIR2  = $(FR_SRCDIR)$(S)portable$(S)MemMang
ifeq ($(MPU),1)
FR_SRCDIR3  = $(FR_SRCDIR)$(S)portable$(S)Common
endif
XT_SRCDIR   = $(SRCROOT)

vpath %.c $(FR_SRCDIR) $(FR_SRCDIR2) $(FR_SRCDIR3) $(XT_SRCDIR)
vpath %.S $(XT_SRCDIR)

# File lists
# NOTE: Only heap_4.c is included in the default FreeRTOS build.
# If you want to use another heap manager then edit this file
# and specify the one you want.

FR_C_FILES  = $(notdir $(wildcard $(FR_SRCDIR)/*.c)) \
	      $(notdir $(wildcard $(FR_SRCDIR3)/*.c)) \
	      heap_4.c
XT_C_FILES  = $(notdir $(wildcard $(XT_SRCDIR)/*.c))
XT_S_FILES  = $(notdir $(wildcard $(XT_SRCDIR)/*.S))

# List of all .o files that will go into the library

LIB_C_O     = $(patsubst %.c,%.o,$(filter-out asm-offsets.c,$(XT_C_FILES) $(FR_C_FILES)))
LIB_S_O     = $(patsubst %.S,%.o,$(XT_S_FILES))
LIB_O_LIST  = $(addprefix $(BLDDIR)/,$(LIB_C_O) $(LIB_S_O))

# Output files

OSLIB       = $(BLDDIR)$(S)libfreertos.a

# Build options

ifeq ($(TARGET),sim)
DFLAGS      = -DXT_SIMULATOR
endif
ifeq ($(TARGET),board)
DFLAGS      = -DXT_BOARD
endif

IFLAGS      = \
  -I$(FR_SRCDIR)$(S)include -I$(XT_SRCDIR) -I$(TSTROOT)$(S)common$(S)config_files -I$(BLDDIR)

CSTD        = -std=c99
CFLAGS      = -O2 -g
WFLAGS      = -Werror -Wall -Wextra
CCFLAGS     = $(CSTD) $(CFLAGS) $(WFLAGS) -mno-coproc -mlongcalls -ffunction-sections -mno-l32r-flix $(DFLAGS) $(MPUFLAGS)
ASFLAGS     = $(CCFLAGS)

# File-specific flags

FLAGS_xtensa_vectors_xea3 = -mtext-section-literals

FLAGS_mpu = -mtext-section-literals
FLAGS_portasm = -mtext-section-literals
FLAGS_xtensa_context = -mtext-section-literals
FLAGS_xtensa_coproc_handler = -mtext-section-literals

# Include dependency rules (generated using -MD)

-include $(wildcard $(BLDDIR)/*.d)

# Targets

all : $(OSLIB)

$(BLDDIR)/.mkdir :
	@$(MKPATH) $(BLDDIR)
	@echo "" > $@

$(LIB_O_LIST) : $(BLDDIR)/asm-offsets.h

$(OSLIB) : $(LIB_O_LIST)
	$(AR) -rs $@ $^

$(BLDDIR)/asm-offsets.h : asm-offsets.c $(BLDDIR)/.mkdir
	$(CC) $(CCFLAGS) $(IFLAGS) -MD -MF $(subst .h,.d,$@) -MT $@ -ffunction-sections -fdata-sections -Wl,--gc-sections -o $@.exe $<
	$(ISS) $@.exe > $@
	$(RM) $(subst /,$(S),$@.exe)

$(BLDDIR)/%.o : %.c $(BLDDIR)/.mkdir
	$(CC) $(CCFLAGS) $(IFLAGS) $(FLAGS_$*) -mtext-section-literals -MD -MF $(subst .o,.d,$@) -c -o $@ $<

$(BLDDIR)/%.o : %.S $(BLDDIR)/.mkdir
	$(CC) $(ASFLAGS) $(IFLAGS) $(FLAGS_$*) -MD -MF $(subst .o,.d,$@) -c -o $@ $<

clean :
	$(RM_R) $(BLDDIR)

clean_all :
	$(RM_R) $(BLDROOT)

.PHONY : all clean clean_all

