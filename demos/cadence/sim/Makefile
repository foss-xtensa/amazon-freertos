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
ISS         = xt-run
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
    timer_test1.c        \
    perf_test.c          \
    xt_alloca.c          \
    xt_coproc.c          \
    xt_clib.c            \
    xt_intr.c

ifeq ($(MPU),1)
TEST_FILES += mpu_basic.c
endif

EXE         = $(addprefix $(BLDDIR)/,$(patsubst %.c,%.exe,$(TEST_FILES)))
OBJS        = $(addprefix $(BLDDIR)/,$(patsubst %.c,%.o,$(TEST_FILES)))
SRC_FILES   = $(TEST_FILES)

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

# Build with MPU support
ifeq ($(MPU),1)
LSPPATH = $(BLDDIR)$(S)lsp$(S)$(LSP)
MPUFLAGS = -DportUSING_MPU_WRAPPERS=1 -DportALIGN_SECTIONS
else
LSPPATH = $(LSP)
endif

# Build options. Override CFLAGS from the make command line if desired.
# NOTE: Default LDFLAGS are set up to discard unused sections so that
# the final exe size is minimized.

INCS        = -I$(SRCDIR) -I$(CFGDIR) -I$(OSROOT) -I$(OSINCDIR) -I$(OSINCDIR)$(S)private -I$(BLDDIR)
DFLAGS     += -DSTANDALONE
CFLAGS     ?= -O2 -Os -g -Wall -Werror
CCFLAGS     = $(CFLAGS) -ffunction-sections -mlongcalls $(DFLAGS) $(MPUFLAGS)
LDFLAGS    ?= -Wl,--gc-sections $(CFLAGS)

mpu_basic_CCFLAGS = -O0

# use make DEBUG=1 to get MPU debug prints
ifeq ($(DEBUG),1)
CFLAGS     += -DDEBUG
endif

# include dependency rules (generated using -MD)

-include $(wildcard $(BLDDIR)/*.d)

# Targets

all : $(EXE)
check : $(addprefix check_,$(notdir $(EXE)))
check_% : $(BLDDIR)/%
	$(ISS) --exit_with_target_code $<

# NOTE: The small test needs its own version of the FreeRTOS library.
# So don't club it with the other tests, it has to be built on its own.

small : small_test

os : $(OSLIB)

ifeq ($(HOSTTYPE),win)
SECT_PRIV := PLACE SECTIONS(privileged_data privileged_functions freertos_system_calls private_region1 private_region2 private_region3 private_stack) WITH_SECTION(.data)
endif

ifeq ($(HOSTTYPE),unix)
SECT_PRIV := "PLACE SECTIONS(privileged_data privileged_functions freertos_system_calls private_region1 private_region2 private_region3 private_stack) WITH_SECTION(.data)"
endif

$(LSPPATH)/.mkdir :
	$(MKPATH) $(LSPPATH)
	$(CP) $(subst /,$(S),$(CONFIGDIR))$(S)xtensa-elf$(S)lib$(S)$(LSP)$(S)memmap.xmm $(LSPPATH)
	$(CP) $(subst /,$(S),$(CONFIGDIR))$(S)xtensa-elf$(S)lib$(S)$(LSP)$(S)specs $(LSPPATH)
	echo $(SECT_PRIV) >> $(LSPPATH)$(S)memmap.xmm
	xt-genldscripts -b $(LSPPATH)
	@echo "" > $@

$(BLDDIR)/.mkdir :
	@$(MKPATH) $(BLDDIR)
	@echo "" > $@

ifeq ($(MPU),1)
MPULSP = $(LSPPATH)/.mkdir
endif

$(EXE) : $(BLDDIR)/%.exe : $(BLDDIR)/%.o $(OSLIB) $(MPULSP)
	$(CC) -o $@ $< -mlsp=$(LSPPATH) $(LDFLAGS) $(OSLIB) -lxtutil

$(OBJS) : $(BLDDIR)/%.o : %.c $(BLDDIR)/.mkdir
	$(CC) $(CCFLAGS) $($(%*)_CFLAGS) $(INCS) -MD -MF $(subst .o,.d,$@) -c -o $@ $<

$(OSLIB) ::
	$(MAKE) -f $(OSROOT)$(S)Makefile all TARGET=$(TARGET) CFLAGS="$(CFLAGS)" SRCROOT=$(OSROOT) BLDROOT=$(BLDROOT) MPU=$(MPU)

mpu_basic : $(BLDDIR)/mpu_basic.exe

example   : $(BLDDIR)/example.exe

perf_test : $(BLDDIR)/perf_test.exe

xt_coproc : $(BLDDIR)/xt_coproc.exe

xt_alloca : $(BLDDIR)/xt_alloca.exe

xt_clib   : $(BLDDIR)/xt_clib.exe

xt_intr   : $(BLDDIR)/xt_intr.exe

# xt_clib uses asm-offsets.h, which is build as a side effect of $(OSLIB)
$(BLDDIR)/xt_clib.o : $(OSLIB)

small_test : $(BLDDIR)/.mkdir
	$(MAKE) --no-print-directory $(BLDDIR)/small_test.exe CFLAGS="$(CFLAGS) -DSMALL_TEST -DXT_USE_THREAD_SAFE_CLIB=0"

clean :
	$(RM_R) $(BLDDIR)

clean_all :
	$(RM_R) $(BLDROOT)

.PHONY : all clean clean_all mpu_basic example perf_test xt_coproc xt_alloca xt_clib xt_intr

help :
	@echo Available options:
	@echo     "make all              -- build all tests for simulator"
	@echo     "make all TARGET=ml605 -- build all tests for ML605 FPGA board"
	@echo     "make all TARGET=kc705 -- build all tests for KC705 FPGA board"
	@echo     "make check            -- build and run all tests on simulator"
	@echo     "make clean            -- Clean build for current config"
	@echo     "make clean_all        -- Clean build for all configs"

