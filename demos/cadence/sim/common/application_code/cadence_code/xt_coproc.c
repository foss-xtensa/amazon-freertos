/*******************************************************************************
// Copyright (c) 2003-2015 Cadence Design Systems, Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
// CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
// SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
--------------------------------------------------------------------------------
*/

/*
*********************************************************************************************************
*
*                                          CO-PROCESSOR TEST
*
* This is a test application for co-processor lazy context-switching in the Xtensa port of FreeRTOS.
* It is a simple test that uses only the floating point co-processor and single-precision floats.
*
* It will also run on an Xtensa core configuration without a floating-point co-processor.
* It does not rely on a C library so can run on practically anything.
* If compiled with C library support (THREAD_SAFE_NEWLIB), uses printf() instead of putstr().
*
* This test normally has 4 tasks of equal priority round-robin with a single-tick timeslice. 
* Since FreeRTOS does not support multiple tasks at the same priority, this version of the test
* simulates round-robin of the application tasks with a single-tick timeslice by means of a
* round robin simulation that raises each application task in turn above the others for one tick.
*
* Target  : All Xtensa configurable and Diamond preconfigured processors.
*********************************************************************************************************
*/
#include    <ctype.h>
#include    <string.h>
#include    <unistd.h>
#include    <assert.h>

#ifdef XT_BOARD
#include    <xtensa/xtbsp.h>
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <testcommon.h>

#if XT_USE_THREAD_SAFE_CLIB > 0u
#include    <stdio.h>
#endif

/*
*********************************************************************************************************
*                                         SHARED DATA AND MACROS
*********************************************************************************************************
*/

#if XT_USE_THREAD_SAFE_CLIB > 0u
/* Uncomment to turn on diagnostics to help debug. Needs thread-safe printf(). */
//#define DIAGNOSTICS
#endif

/* Stack size for tasks that do not use the C library or the coprocessors */
#define     TASK_STK_SIZE_MIN       (XT_STACK_MIN_SIZE)

/* Stack size for tasks that use the C library and/or the coprocessors */
#define     TASK_STK_SIZE_STD       (XT_STACK_EXTRA + 0x1000)

/* Uniform prefix for reporting PASS/FAIL test results. */
#define TEST_PFX    "Xtensa co-processor context-switch test (xt_coproc)"

#define NTASKS      4

/*
Parameters to crunch() are chosen to ensure different values in each task so that if the
co-processor context-switch is incorrect they will not converge on their expected values.
The first param n determines how long the computation runs and thus how many context switches
occur. The second param x determines the rate of convergence (slow enough, but converges
before the end). The third parameter z is an initial value that keeps the sequences from
different tasks in disjoint regions. Different params are chosen for configs without the
floating-point co-processor so that this test will run on any config in reasonable time.
*/
#if XCHAL_HAVE_FP
    #define TASK0_PARAMS    50000, 0.9997, 2.0
    #define TASK1_PARAMS    50000, 0.9996, 4.0
    #define TASK2_PARAMS    50000, 0.9995, 6.0
    #define TASK3_PARAMS    50000, 0.9994, 8.0
#else                       
    #define TASK0_PARAMS      500, 0.97,   2.0
    #define TASK1_PARAMS      500, 0.96,   4.0
    #define TASK2_PARAMS      500, 0.95,   6.0
    #define TASK3_PARAMS      500, 0.94,   8.0
#endif

/* Expected and Actual Results. */
static float    result[NTASKS];
static float    expect[NTASKS];


/*
*********************************************************************************************************
*                                             LOCAL FUNCTIONS
*********************************************************************************************************
*/

#if XT_USE_THREAD_SAFE_CLIB > 0u
#define putstr(s)  printf(s)
#else
/* Output a simple string to the console. */
static void putstr(const char *s)
{
    int c;

    while ((c = *s) != '\0') {
        if (c == '\n') {
            outbyte('\r');
            outbyte('\n');
        }
        else if (iscntrl(c) && c != '\r') {
            outbyte('^');
            outbyte('@' + c);
        }
        else outbyte(c);
        ++s;
    }
}
#endif /* XT_USE_THREAD_SAFE_CLIB */

/*
Shared function that does a convergent iterative floating point computation
a specified number of iterations and returns the result.
    n = number of iterations (integer >= 1, can be arbitrarily large).
    x = initial coefficient in range [0,1), close to 1 slows convergence.
    z = arbitrary initial value > 0.
*/
static float crunch(unsigned n, float x, float z)
{
    unsigned i,j;
    float mx = -x;
    float result = z;

    for (i=0; i<n; i+=j) {
        for (j=0; j < n>>3; ++j) {
            result += x * z;
            x = mx * x;
        }
        /*
        Solicit context-switch to exercise exception handler not saving state.
        However compiler saves/restores state around function calls, so test 
        won't fail if exception handler merely saves/restores incorrectly,
        only if the code for those cases is severely broken causing a crash.
        Delay till next tick to ensure another task gets to run (in this case
        since all tasks are at the same priority, it will be the highest other).
        Since vTaskDelay() doesn't check port_xSchedulerRunning, we need to do this (this is
        cheating because port_xSchedulerRunning is not part of the API, but this is a test).
        */
        extern unsigned port_xSchedulerRunning;
        if (port_xSchedulerRunning)
        	vTaskDelay(1);
    }

    return result;
}

/*
*********************************************************************************************************
*                                        APPLICATION TASKS
*
* These slave tasks run the crunch() function and write their result to a location per task.
* If co-processor context-switching is working properly, they will not interfere with each other.
*
*********************************************************************************************************
*/

#define TASK_INIT_PRIO          11
#define TASK_HIGH_PRIO          10
#define TASK0_PRIO              5
#define TASK1_PRIO              4
#define TASK2_PRIO              3
#define TASK3_PRIO              2

static TaskHandle_t Task_TCB[4];

void Task0(void *pdata)
{
    result[0] = crunch(TASK0_PARAMS);
    if (result[0] == 0) result[0] = -1;
    vTaskDelete(NULL);
    assert(0);                          // Should never come here
}

void Task1(void *pdata)
{
    result[1] = crunch(TASK1_PARAMS);
    if (result[1] == 0) result[1] = -1;
    vTaskDelete(NULL);
    assert(0);                          // Should never come here
}

void Task2(void *pdata)
{
    result[2] = crunch(TASK2_PARAMS);
    if (result[2] == 0) result[2] = -1;
    vTaskDelete(NULL);
    assert(0);                          // Should never come here
}

void Task3(void *pdata)
{
    result[3] = crunch(TASK3_PARAMS);
    if (result[3] == 0) result[3] = -1;
    vTaskDelete(NULL);
    assert(0);                          // Should never come here
}

/*
*********************************************************************************************************
*                                          APP INITIALZATION TASK
*
* It initializes the RTOS, creates the other tasks, controls the test and reports the results.
* After starting the other tasks (1 - 3) it also does the same as the other (slave) tasks.
*
*********************************************************************************************************
*/


static void Init_Task(void *pdata)
{
    static const unsigned task_prios[NTASKS] = {TASK0_PRIO, TASK1_PRIO, TASK2_PRIO, TASK3_PRIO};
    unsigned    t0, t1;
    int         i, busy;
    int         exit_code = 0;
    int         err = 0;

    /* Initialize the results. */
    for (i=0; i<NTASKS; ++i)
        result[i] = 0.0;

    /* Compute the expected values before multitasking starts (no CP exception). */
    expect[0] = crunch(TASK0_PARAMS);
    expect[1] = crunch(TASK1_PARAMS);
    expect[2] = crunch(TASK2_PARAMS);
    expect[3] = crunch(TASK3_PARAMS);

#ifdef DIAGNOSTICS
    for (i=0; i<NTASKS; ++i)
        printf("expect[%u] == %f\n", i, expect[i]);
#endif

    /* Create the application tasks (all are lower priority so wait for us). */
	err = xTaskCreate(Task0, "Task0", TASK_STK_SIZE_STD, NULL, TASK0_PRIO, &Task_TCB[0]);
    if (err != pdPASS)
    {
        putstr(TEST_PFX " FAILED to create Task0\n");
        goto done;
    }

	err = xTaskCreate(Task1, "Task1", TASK_STK_SIZE_STD, NULL, TASK1_PRIO, &Task_TCB[1]);
    if (err != pdPASS)
    {
        putstr(TEST_PFX " FAILED to create Task1\n");
        goto done;
    }

	err = xTaskCreate(Task2, "Task2", TASK_STK_SIZE_STD, NULL, TASK2_PRIO, &Task_TCB[2]);
    if (err != pdPASS)
    {
        putstr(TEST_PFX " FAILED to create Task2\n");
        goto done;
    }

	err = xTaskCreate(Task3, "Task3", TASK_STK_SIZE_STD, NULL, TASK3_PRIO, &Task_TCB[3]);
    if (err != pdPASS)
    {
        putstr(TEST_PFX " FAILED to create Task3\n");
        goto done;
    }

    /* The test begins here. */
    t0 = xTaskGetTickCount();

    /* Simulate round-robin of the application tasks every tick. Stop when all results are in. */
    do {
        busy = 0;
        for (i = 0; i < NTASKS; ++i) {
			vTaskPrioritySet(Task_TCB[i], TASK_HIGH_PRIO);
            vTaskDelay(1);
			vTaskPrioritySet(Task_TCB[i], task_prios[i]);
            busy |= result[i] == 0.0;
        }
    } while (busy);

    /* The test ends here. Pause to let application tasks shut down cleanly. */
    t1 = xTaskGetTickCount();
    vTaskDelay(NTASKS);

    /* Report results. */
    #ifdef DIAGNOSTICS
    printf("(%lu ticks)\n", t1-t0);
    for (i=0; i<NTASKS; ++i)
        printf("result[%u] == %f\n", i, result[i]);
    #endif
    for (i=0; i<NTASKS; ++i) {
        if (result[i] != expect[i]) {
            exit_code = 1;
            #ifdef XT_BOARD
            xtbsp_display_string("xt_coproc FAILED");
            #endif
            putstr(TEST_PFX " FAILED!\n");
            goto done;
        }
    }
    #ifdef XT_BOARD
    xtbsp_display_string("xt_coproc PASSED");
    #endif
    putstr(TEST_PFX " PASSED!\n");

done:
    #ifdef XT_SIMULATOR
    /* Shut down simulator and report error code as exit code to host (0 = OK). */
    _exit(exit_code);
    #endif

    /* Terminate this task. RTOS will continue to run timer, stats and idle tasks. */
    vTaskDelete(NULL);
    // Should never come here.
    assert(0);
}

/*
*********************************************************************************************************
*                                             C ENTRY POINT
*
* Initializes FreeRTOS after the platorm's run-time system has initialized the basic platform.
* Creates at least the first task, which can then create other tasks.
* Starts multitasking.
*
*********************************************************************************************************
*/

/* Hook functions for standalone tests */
#ifdef STANDALONE

#if configUSE_TICK_HOOK
void vApplicationTickHook( void )
{
}
#endif

void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    /* For some reason printing pcTaskName is not working */
    puts("\nStack overflow, stopping.");
    exit(0);
}

int main(int argc, char *argv[])
#else
int main_xt_coproc(int argc, char *argv[])
#endif
{
    int     err = 0;
    int     exit_code = 0;

    putstr("\nNumber of coprocessors = ");
    outbyte('0' + XCHAL_CP_NUM);
    putstr("\nYou should verify that there is a float coprocessor!\n");

    #ifdef XT_BOARD
    xtbsp_display_string("xt_coproc test");
    #endif
    putstr(TEST_PFX " running...\n");

    /* Create the control task initially with the high priority. */
	err = xTaskCreate(Init_Task, "Init_Task", TASK_STK_SIZE_STD, NULL, TASK_INIT_PRIO, NULL);
    if (err != pdPASS)
    {
        putstr(TEST_PFX " FAILED to create Init_Task\n");
        goto done;
    }

    /* Start task scheduler */
	vTaskStartScheduler();

done:
    exit_code = err;

#ifdef XT_SIMULATOR
    /* Shut down simulator and report error code as exit code to host (0 = OK). */
    _exit(exit_code);
#endif

    /* Does not reach here ('return' statement keeps compiler happy). */
    return 0;
}

