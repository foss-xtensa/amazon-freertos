
How to build and run the tests
------------------------------

To build the tests, run "make all" in this directory. This will build
the following files under the build/ subdirectory -

    example.exe     -- Simple Xtensa demo program
    
    perf_test.exe   -- Performance benchmarking

    xt_coproc.exe   -- Xtensa coprocessor exerciser demo
    
    xt_alloca.exe   -- Xtensa alloca exception demo
    
    xt_clib.exe     -- Xtensa C lib demo (Newlib)

    xt_intr.exe     -- Xtensa interrupt test

To build a minimal-size test, run "make small". This will build the
following test in build/ -

    small_test.exe  -- Small size test

This test modifies the configuration to minimize the code and data
size. See the FreeRTOSConfig.h file for the changes from the default
config. The changes are all ifdefed with SMALL_TEST.

NOTE: The small test requires the FreeRTOS library to be built with
the modified config. So you should do a "make clean" before running
"make small" to make sure you get the smaller version. Similarly,
run "make clean" before running "make all".


The tests will build by default to be run on the Xtensa simulator
(xt-run). The available options are

    make TARGET=sim     -- build to run on simulator (default)

    make TARGET=ml605   -- build to run on the ML605 FPGA board

    make TARGET=kc705   -- build to run on the KC705 FPGA board

Remember to do a "make clean" when switching targets.

To build the tests for your own hardware, you must provide the correct
linker script (LSP) and your own board support package. See the makefile
and refer to Xtensa tools documentation for more details.

If you build from Xplorer, remember to enable -mlongcalls in the Assembler
options for certain targets (e.g. when you get the linker warning: dangerous
relocation).

-End-

