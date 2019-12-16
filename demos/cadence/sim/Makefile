### Makefile to build the FreeRTOS demo ###

# Override the compiler options if desired by specifying CFLAGS on the make
# command line. For example,
#
#    make all CFLAGS="-Os -g" to optimize for size.

# Build target (options: sim, ml605, kc705)

TARGET      = sim

# Tools

CC          = xt-xcc
AS          = xt-xcc
XT_CORE     = $(patsubst %-params,%,$(notdir $(shell xt-xcc --show-config=core)))
CONFIGDIR   = $(shell xt-xcc --show-config=config)

# For platform-specific commands

include $(CONFIGDIR)/misc/hostenv.mk

# Source code and build locations

CWDIR       = $(subst /,$(S),$(CURDIR))
DROOT       = $(CWDIR)
SRCDIR      = $(DROOT)$(S)common$(S)application_code$(S)cadence_code
CFGDIR      = $(DROOT)$(S)common$(S)config_files
OSROOT      = $(abspath $(DROOT)$(S)..$(S)..$(S)..$(S)portable$(S)XCC$(S)Xtensa)
BLDROOT     = $(DROOT)$(S)build
BLDDIR      = $(BLDROOT)$(S)$(XT_CORE)
OSINCDIR    = $(abspath $(DROOT)$(S)..$(S)..$(S)..$(S)include)

# Source files

TEST_FILES  = \
    example.c            \
    perf_test.c          \
    xt_alloca.c          \
    xt_coproc.c          \
    xt_clib.c            \
    xt_intr.c

SRC_FILES   = $(TEST_FILES)

OBJ_FILES   = $(addprefix $(BLDDIR)/,$(patsubst %.c,%.o,$(notdir $(SRC_FILES))))

# Search paths

vpath %.c $(SRCDIR)

# Output files

OSLIB       = $(BLDDIR)$(S)libfreertos.a

# Target-specific options

ifeq ($(TARGET),sim)
DFLAGS      = -DXT_SIMULATOR
LSP         = sim
endif
ifeq ($(TARGET),ml605)
DFLAGS      = -DXT_BOARD
LSP         = xtml605-rt
endif
ifeq ($(TARGET),kc705)
DFLAGS      = -DXT_BOARD
LSP         = xtkc705-rt
endif

# Build options. Override CFLAGS from the make command line if desired.
# NOTE: Default LDFLAGS are set up to discard unused sections so that 
# the final exe size is minimized.

INCS        = -I$(SRCDIR) -I$(CFGDIR) -I$(OSROOT) -I$(OSINCDIR) -I$(OSINCDIR)$(S)private -I$(BLDDIR)

CFLAGS     ?= -O2 -Os -g -Wall -Werror
CCFLAGS     = $(CFLAGS) -ffunction-sections -mlongcalls $(DFLAGS)
LDFLAGS    ?= -Wl,--gc-sections

# include dependency rules (generated using -MD)

-include $(wildcard $(BLDDIR)/*.d)

# Targets

all : mkdir example perf_test xt_coproc xt_alloca xt_clib xt_intr

# NOTE: The small test needs its own version of the FreeRTOS library.
# So don't club it with the other tests, it has to be built on its own.

small : mkdir small_test

os  : mkdir $(OSLIB)

mkdir : $(BLDDIR)/.mkdir

$(BLDDIR)/.mkdir :
	@$(MKPATH) $(BLDDIR)
	@echo "" > $@

$(EXE) : $(OBJ_FILES) $(OSLIB)
	$(CC) $(CCFLAGS) $(INCS) -o $@ $^ -mlsp=$(LSP) $(LDFLAGS)

$(BLDDIR)/%.o : %.c
	$(CC) $(CCFLAGS) $(INCS) -MD -MF $(subst .o,.d,$@) -c -o $@ $<

$(OSLIB) :
	$(MAKE) -f $(OSROOT)$(S)Makefile all TARGET=$(TARGET) CFLAGS="$(CFLAGS)" SRCROOT=$(OSROOT) BLDROOT=$(BLDROOT)

example   : $(BLDDIR)/example.exe

perf_test : $(BLDDIR)/perf_test.exe

xt_coproc : $(BLDDIR)/xt_coproc.exe

xt_alloca : $(BLDDIR)/xt_alloca.exe

xt_clib   : $(BLDDIR)/xt_clib.exe

xt_intr   : $(BLDDIR)/xt_intr.exe

small_test :
	$(MAKE) --no-print-directory $(BLDDIR)/small_test.exe CFLAGS="$(CFLAGS) -DSMALL_TEST -DXT_USE_THREAD_SAFE_CLIB=0"

$(BLDDIR)/%.exe : %.c $(OSLIB)
	$(CC) $(CCFLAGS) $(INCS) -o $@ $^ -mlsp=$(LSP) $(LDFLAGS) -DSTANDALONE=1

clean :
	$(RM_R) $(BLDDIR)

clean_all :
	$(RM_R) $(BLDROOT)

.PHONY : all mkdir clean clean_all example perf_test xt_coproc xt_alloca xt_clib xt_intr

help :
	@echo Available options:
	@echo     "make all              -- build all tests for simulator"
	@echo     "make all TARGET=ml605 -- build all tests for ML605 FPGA board"
	@echo     "make all TARGET=kc705 -- build all tests for KC705 FPGA board"
	@echo     "make clean            -- Clean build for current config"
	@echo     "make clean_all        -- Clean build for all configs"

