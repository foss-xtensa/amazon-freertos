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
*                                          C LIBRARY TEST
*
* This test exercises the C library thread safety mechanism in the Xtensa port of FreeRTOS.
* It verifies that every task that specifies C library context save gets its own private
* save area, and tasks that do not specify this get to share a global context area.
*
* This test is only meaningful if XT_USE_THREAD_SAFE_CLIB is enabled.
* Currently this test works only for the newlib  and xclib C libraries as they are the only
* ones supporting thread safety.
*
* Target  : All Xtensa configurable and Diamond preconfigured processors.
*********************************************************************************************************
*/

#include "FreeRTOS.h"

#include <stdio.h>
#include <stdlib.h>

#if XT_USE_THREAD_SAFE_CLIB > 0u
#include <sys/reent.h>
#else
#warning XT_USE_THREAD_SAFE_CLIB not defined, this test will do nothing.
#endif

#ifdef XT_BOARD
#include <xtensa/xtbsp.h>
#endif

#include <testcommon.h>

#include "task.h"
#include "queue.h"

/*
*********************************************************************************************************
*                                         SHARED DATA AND MACROS
*********************************************************************************************************
*/

/* Stack size for tasks that do not use the C library */
#define     TASK_STK_SIZE_MIN       (XT_STACK_MIN_SIZE + XT_STACK_EXTRA)

/* Stack size for tasks that use the C library and the coprocessors */
#define     TASK_STK_SIZE_STD       (0x1000 + XT_STACK_EXTRA_CLIB)

/* Uniform prefix for reporting PASS/FAIL test results. */
#define TEST_PFX    "Xtensa C library context switch test (xt_clib)"

#define NTASKS      1

unsigned int result[NTASKS];

// Hack to access TCB's content (xNewLib_reent) for test purposes
typedef struct
{
	volatile StackType_t	*pxTopOfStack;	/*< Points to the location of the last item placed on the tasks stack.  THIS MUST BE THE FIRST MEMBER OF THE TCB STRUCT. */

	#if ( portUSING_MPU_WRAPPERS == 1 )
		xMPU_SETTINGS	xMPUSettings;		/*< The MPU settings are defined as part of the port layer.  THIS MUST BE THE SECOND MEMBER OF THE TCB STRUCT. */
	#endif

	ListItem_t			xStateListItem;	/*< The list that the state list item of a task is reference from denotes the state of that task (Ready, Blocked, Suspended ). */
	ListItem_t			xEventListItem;		/*< Used to reference a task from an event list. */
	UBaseType_t			uxPriority;			/*< The priority of the task.  0 is the lowest priority. */
	StackType_t			*pxStack;			/*< Points to the start of the stack. */
	char				pcTaskName[ configMAX_TASK_NAME_LEN ];/*< Descriptive name given to the task when created.  Facilitates debugging only. */ /*lint !e971 Unqualified char types are allowed for strings and single characters only. */

	#if ( portSTACK_GROWTH > 0 )
		StackType_t		*pxEndOfStack;		/*< Points to the end of the stack on architectures where the stack grows up from low memory. */
	#endif

	#if ( portCRITICAL_NESTING_IN_TCB == 1 )
		UBaseType_t		uxCriticalNesting;	/*< Holds the critical section nesting depth for ports that do not maintain their own count in the port layer. */
	#endif

	#if ( configUSE_TRACE_FACILITY == 1 )
		UBaseType_t		uxTCBNumber;		/*< Stores a number that increments each time a TCB is created.  It allows debuggers to determine when a task has been deleted and then recreated. */
		UBaseType_t		uxTaskNumber;		/*< Stores a number specifically for use by third party trace code. */
	#endif

	#if ( configUSE_MUTEXES == 1 )
		UBaseType_t		uxBasePriority;		/*< The priority last assigned to the task - used by the priority inheritance mechanism. */
		UBaseType_t		uxMutexesHeld;
	#endif

	#if ( configUSE_APPLICATION_TASK_TAG == 1 )
		TaskHookFunction_t pxTaskTag;
	#endif

	#if( configNUM_THREAD_LOCAL_STORAGE_POINTERS > 0 )
		void *pvThreadLocalStoragePointers[ configNUM_THREAD_LOCAL_STORAGE_POINTERS ];
	#endif

	#if( configGENERATE_RUN_TIME_STATS == 1 )
		uint32_t		ulRunTimeCounter;	/*< Stores the amount of time the task has spent in the Running state. */
	#endif

	#if ( configUSE_NEWLIB_REENTRANT == 1 )
		/* Allocate a Newlib reent structure that is specific to this task.
		Note Newlib support has been included by popular demand, but is not
		used by the FreeRTOS maintainers themselves.  FreeRTOS is not
		responsible for resulting newlib operation.  User must be familiar with
		newlib and must provide system-wide implementations of the necessary
		stubs. Be warned that (at the time of writing) the current newlib design
		implements a system-wide malloc() that must be provided with locks. */
		struct	_reent xNewLib_reent;
	#endif

		// Truncated after this point

} clibTCB_t;
extern volatile clibTCB_t * volatile pxCurrentTCB;

/*
*********************************************************************************************************
*                                        APPLICATION TASKS
*********************************************************************************************************
*/

TaskHandle_t Task_TCB[NTASKS];

void Task_Func(void *pdata)
{
    int val = (int) pdata;
    unsigned cnt = 0;
    void *test_p;
    int err;

    srand(val);

#if XT_USE_THREAD_SAFE_CLIB > 0u

    while (cnt < 400) {
#if XSHAL_CLIB == XTHAL_CLIB_XCLIB || XSHAL_CLIB == XTHAL_CLIB_NEWLIB
        if (pxCurrentTCB)
        {
        	// Note that _impure_ptr (newlib) is redefined as _reent_ptr in the case of
        	// xclib.
            if ((uint32_t)_impure_ptr != (uint32_t)(&pxCurrentTCB->xNewLib_reent)) {
            	// A failure might mean that the hack definition of TCB in this file, xt_clib.c,
            	// is out of date with respect to the official definition in tasks.c.
                printf("Task %d, Bad reent ptr\n", val);
                exit(1);
            }
        }
        else {
            printf("Task %d, Bad TCB pointer!\n", val); // This means there is some corruption
            exit(2);
        }
#else
  #error Unsupported C library
#endif

        test_p = malloc(rand() % 500);
        if (!test_p) {
            printf("Task %d, malloc() failed\n", val);
            exit(3);
        }

        if (val == 0 && cnt % 100 == 99) {
        	printf("100...\n");
        }

        vTaskDelay(1);
        free(test_p);
        cnt++;
    }

#else

    // No thread-safe library, nothing to do
    test_p = 0;
    cnt    = 0;
    vTaskDelay(1);

#endif

    result[val] = 1;
    vTaskDelete(NULL);
}


/*
*********************************************************************************************************
*                                          APP INITIALZATION TASK
*
* It initializes the RTOS, creates the other tasks, controls the test and reports the results.
*
*********************************************************************************************************
*/

#define     INIT_TASK_STK_SIZE      (TASK_STK_SIZE_MIN)

static void Init_Task(void *pdata)
{
    int t0, t1;
    int i, busy;
    int err = 0;

    for (i = 0; i < NTASKS; ++i) {
        /* Create the application tasks (all are lower priority so wait for us). */
    	err = xTaskCreate(Task_Func, "Task", TASK_STK_SIZE_STD, (void*)i, 18, &Task_TCB[i]);

        if (err != pdPASS)
        {
            printf(TEST_PFX " FAILED to create Task\n");
            goto done;
        }
    }

    /* The test begins here. */
    t0 = xTaskGetTickCount();

    /* Simulate round-robin of the application tasks every tick. Stop when all results are in. */
    do {
        busy = 0;
        for (i = 0; i < NTASKS; ++i) {
			//vTaskPrioritySet(Task_TCB[i], 21);
            vTaskDelay(NTASKS);
			//vTaskPrioritySet(Task_TCB[i], 22);
            busy |= result[i] == 0.0;
        }
    } while (busy);

    /* The test ends here. Pause to let application tasks shut down cleanly. */
    t1 = xTaskGetTickCount();
    //vTaskDelay(NTASKS);

    printf(TEST_PFX " PASSED!\n");

done:
    #ifdef XT_SIMULATOR
    /* Shut down simulator and report error code as exit code to host (0 = OK). */
    exit(0);
    #endif

    /* Terminate this task. RTOS will continue to run timer, stats and idle tasks. */
    vTaskDelete(NULL);
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

int main(void)
#else
int main_xt_clib(int argc, char *argv[])
#endif
{
    int err = 0;
    int exit_code = 0;

    printf(TEST_PFX " running...\n");

    /* Create the control task initially with the high priority. */
	err = xTaskCreate(Init_Task, "Init_Task", INIT_TASK_STK_SIZE, NULL, 20, NULL);


    if (err != pdPASS)
    {
        printf(TEST_PFX " FAILED to create Init_Task\n");
        goto done;
    }

    /* Start task scheduler */
	vTaskStartScheduler();

done:
    exit_code = err;

#ifdef XT_SIMULATOR
    /* Shut down simulator and report error code as exit code to host (0 = OK). */
    exit(exit_code);
#endif

    /* Does not reach here ('return' statement keeps compiler happy). */
    return 0;
}

