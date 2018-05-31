
Release Notes for FreeRTOS Xtensa port
--------------------------------------


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


Notes for version 1.7
---------------------

- portSTACK_TYPE changed to uint32_t from uint8_t. Stack size definitions
  adjusted accordingly.

- portYIELD_FROM_ISR changed to call _frxt_setup_switch conditionally.

- Tick handler calls xTaskIncrementTick inside critical region to prevent
  interrupt nesting issues.


Notes for version 1.6
---------------------

- This version supports FreeRTOS version 10.0.0.

- The directory structure and makefiles have been reorganized to conform
  to the structure of FreeRTOS version 10.0.0.


Notes for version 1.5
---------------------

- This version supports FreeRTOS version 9.0.0.

- This version supports thread safe operation with the Xtensa C Library
  (xclib) in addition to the Newlib C library (newlib).
  The xtensa_config.h file is set up to enable thread safe operation by
  default if xclib or newlib is detected. Use -D XT_USE_THREAD_SAFE_CLIB=0
  on the command line or in the project definition to disable this.

- Incorrect function prototypes for xt_ints_on/xt_ints_off have been fixed.


Notes for version 1.4.2
-----------------------

- This version implements a bugfix. The bug was that even when the
  config parameter "configUSE_TIME_SLICING" was set to zero, time
  slicing would still occur due to a problem in the Xtensa interrupt
  processing code.

- The "small_test" test has been added to illustrate how FreeRTOS 
  can be configured to reduce the memory footprint.


Notes for version 1.4
---------------------

- This version supports FreeRTOS version 8.2.0.

- This version adds a new test xt_intr.c for interrupts testing.

- This version separates the trace and benchmark facilities into a
  separate downloadable.
  

Notes for version 1.3
---------------------

- This version implements a separate interrupt stack. The interrupt
  dispatch code saves the current task's state on to the task's own
  stack and then switches to the interrupt stack for the interrupt
  processing. Nested interrupts will continue to use the interrupt
  stack. This helps to reduce the sizes of task stacks, since task
  stacks no longer have to account for space to accommodate possible
  nested interrupts. The size of the interrupt stack is controlled
  by a config parameter.


Notes for version 1.2
---------------------

- This version supports FreeRTOS version 8.1.2.

- This version supports both CALL0 and windowed Xtensa configurations.

- This version supports thread safe operation for the Newlib C library.

- This version introduces an API for installing low/medium priority
  interrupt handlers and exception handlers, which can be written in C.
  See xtensa_api.h for details.

- This version supports software prioritization of interrupts at the
  same level. Define XT_USE_SWPRI to use this feature. Higher-numbered
  interrupts are handled at a higher priority than lower-numbered ones.

- The FreeRTOS demo application requires a stack size of at least 4 KB.
  The standalone tests should be able to run with smaller stack sizes.
  The minimum stack size depends on the specific Xtensa configuration.
  Adjust the stack size of your application as required.

- The examples are all defined with a heap size of 256 KB. This could
  be a problem with Xtensa configurations that have only small amounts
  of memory. Adjust the heap size to suit your platform and application.
  There is an example of a small-footprint configuration in the directory
  Xtensa_XCC_small.

- NOTE: Code overlay support is not yet enabled for this port, even though
  the support code is present. This feature is not enabled nor tested.
  Do not define XT_USE_OVLY when building.

-End-

