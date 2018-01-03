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
*                                            EXAMPLE APPLICATION
*
* This is a simple example application for the Xtensa port of FreeRTOSI.
* It does not rely on a C library so can run on practically anything.
* If compiled with C library support (XT_USE_THREAD_SAFE_CLIB), uses C 
* library stdio output instead of putstr() and outbyte().
*
* Target  : All Xtensa configurable and Diamond preconfigured processors.
*
*********************************************************************************************************
*/

#include    <ctype.h>
#include    <string.h>
#include    <unistd.h>
#include    <assert.h>

#ifdef XT_BOARD
#include    <xtensa/xtbsp.h>
#endif

#include    "FreeRTOS.h"
#include    "semphr.h"
#include    "event_groups.h"

#include <testcommon.h>

#if XT_USE_THREAD_SAFE_CLIB > 0u
#include    <stdio.h>
#endif

// There should be no need to check port_xSchedulerRunning; from main, don't call protected putline function.
// But we'll keep it to test this flag.
extern unsigned int port_xSchedulerRunning;

/*
The first task to be created should have highest priority of those created by
main(), so that it runs first (it may create other tasks of higher priority).
Priority inheritance priorities are specified here too so all priorities are
defined in one place to make their relationships obvious.
*/

#define NEWLIB_MUTEX_PRIO       6  // prio above any task that uses C library
#define CONS_MUTEX_PRIO         5  // prio above any task doing line output
#define INIT_TASK_PRIO          4
#define COUNT_TASK_PRIO         3
#define REPORT_TASK_PRIO        2


#ifdef XT_BOARD
/* A low priority task puts something on the display for equipped boards. */
#define DISPLAY_TASK_PRIO       1
#endif


/*
*********************************************************************************************************
*                                              SHARED DATA    
*********************************************************************************************************
*/

/* Stack size for tasks that do not use the C library. */
#define     TASK_STK_SIZE_MIN       (XT_STACK_MIN_SIZE)

/* Stack size for tasks that use the C library and/or the coprocessors */
#define     TASK_STK_SIZE_STD       (0x1000 + XT_STACK_EXTRA_CLIB)

/* Mutex to avoid interleaving lines on console output. */
SemaphoreHandle_t    ConsMutex;

/* Flags set by each task on completion, just before it terminates. */
#define     TASK_TERM_INIT      (1<<0)
#define     TASK_TERM_REPORT    (1<<1)
#define     TASK_TERM_COUNT     (1<<2)
EventGroupHandle_t TaskTermFlags;

/* Queue for passing count. */
#define     QUEUE_SIZE          10
QueueHandle_t Queue;

/*
*********************************************************************************************************
*                                             LOCAL FUNCTIONS
*********************************************************************************************************
*/

#if XT_USE_THREAD_SAFE_CLIB > 0u
#define putstr(s)  printf(s)
#define outbyte(c) putchar(c);fflush(stdout)
#else
/* Output a simple string to the console. */
static void putstr(const char *s)
{
    char    c;
    unsigned err = 0;

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
Output a line to the console and terminate it with '\n'. Protected by a mutex to prevent lines
from different tasks interleaving. To allow this to be used before tasks are running (eg. in
main), the port_xSchedulerRunning flag is checked before using the mutex.
*/
static void putline(const char *s)
{
    if (port_xSchedulerRunning) {
        xSemaphoreTake(ConsMutex, portMAX_DELAY);
    }

    putstr(s);
    putstr("\n");

    if (port_xSchedulerRunning) {
        xSemaphoreGive(ConsMutex);
    }
}

/*
Print a 32-bit decimal unsigned integer as a string, into a buffer with room for 10 chars + NUL.
Return the pointer to the beginning of the string. The string is right-aligned in the buffer
with leading spaces (padding) that can be useful for right-aligning columns of numbers.
*/
static char *put_u32dec(char buf[11], const unsigned n)
{
    unsigned    err = 0;
    char        *p, *l;
    unsigned    quo, rem;

    /* Print the number in reverse. */
    buf[10] == '\0';
    p = &buf[9];
    for (quo = n; quo >= 10; quo /= 10) {
        rem = quo % 10;
        *p-- = '0' + rem;
    }
    assert(quo < 10);
    *p = '0' + quo;

    /* Pad the beginning of the buffer. */
    for (l = &buf[0]; l < p; ++l)
        *l = ' ';

    return p;
}

/*
*********************************************************************************************************
*                                          DISPLAY TASK
*
* Display something on display-equipped boards to show FreeRTOS Xtensa example is running.
* Meant to be lightweight and low-priority because LCD displays may require lots of busy-waiting.
*
*********************************************************************************************************
*/
#ifdef XT_BOARD
#define      DISPLAY_TASK_STK_SIZE   TASK_STK_SIZE_MIN

void Display_Task(void *pdata)
{
    while(1)
    {
        xtbsp_display_string("FreeRTOS Xtensa");
        vTaskDelay(configTICK_RATE_HZ);
        xtbsp_display_string("Counting Example");
        vTaskDelay(configTICK_RATE_HZ);
    }
}
#endif /* XT_BOARD */


/*
*********************************************************************************************************
*                                          COUNT TASK
*
* This task counts at regular intervals and places the cumulative count in the queue to Report_Task.
* If the queue is full, the count will be skipped and the next count posted a second later.
*
*********************************************************************************************************
*/


#define      COUNT_TASK_STK_SIZE     TASK_STK_SIZE_STD
TaskHandle_t Count_Task_TCB;

void Count_Task(void *pdata)
{
    unsigned count = 0;
    int err = 0;

    putline("[Count_Task] Started.");

    /* Count at regular intervals and place counter in queue. */
    putline("[Count_Task] Counting.");
    do {
    	xQueueSend(Queue, (void*)&count, portMAX_DELAY);
        ++count;
        vTaskDelay(configTICK_RATE_HZ / 5);
#ifdef XT_SIMULATOR
        /* On simulator, stop after a number of iterations (to bound run-time for testing). */
        if (count >= 250)
            break;
#endif
    } while (1);

    putline("");
    putline("[Count_Task] Terminating.");

    /* Set termination flag to alert Init_Task. */
    xEventGroupSetBits(TaskTermFlags, TASK_TERM_COUNT);

    /* Terminate this task. RTOS will continue to run timer, stats and idle tasks. */
    vTaskDelete(NULL);
    /* Does not reach here. */
}

/*
*********************************************************************************************************
*                                          REPORT TASK
*
* This task waits on a message queue for a series on numbers from Count_Task (these should be 
* incrementing by 1 in the sequence received) and reports progress in a readable format that
* fits in 80 columns.
* If it doesn't receive any count from the count task for a few seconds, it terminates.
*
*********************************************************************************************************
*/

#define       REPORT_TASK_STK_SIZE    TASK_STK_SIZE_STD
unsigned char Report_Task_Stack       [REPORT_TASK_STK_SIZE] __attribute__ ((aligned(0x10))); // Let's try one non heap task
TaskHandle_t  Report_Task_TCB;

void Report_Task(void *pdata)
{
    static unsigned prev = 0;
    unsigned count, tens, ones;
    int err = 0;
    char buf[11];

    putline("[Report_Task] Started.");

    /* Acquire console mutex, wait on queue and report, until count task signals termination. */
    /* NOTE: this can create a deadlock when Init_Task can't create Count_Task */
    // xSemaphoreTake(ConsMutex, portMAX_DELAY);
    do {
        err = xQueueReceive(Queue, &count, 3*configTICK_RATE_HZ);
        if (err == pdFAIL) // Timeout
            break;
        tens = count / 10;
        ones = count % 10;
        xSemaphoreTake(ConsMutex, portMAX_DELAY);
        if (count % 50 == 0) {
            put_u32dec(buf, tens);
            if (tens == 0)      // special case: suppress 0 in tens column if tens == 0
                buf[9] = ' ';
            putstr("\n  ");
            putstr("  ");
            putstr(buf);
            putstr("0+");
        }
        if (ones == 0)
            outbyte(' ');
        outbyte('0' + ones);
        xSemaphoreGive(ConsMutex);
        prev = count;
    } while (1);

    putline("");
    putline("[Report_Task] Terminating.");

    /* Set termination flag to alert Init_Task. */
    xEventGroupSetBits(TaskTermFlags, TASK_TERM_REPORT);

    /* Terminate this task. RTOS will continue to run timer, stats and idle tasks. */
    vTaskDelete(NULL);
    /* Does not reach here. */
}

/*
*********************************************************************************************************
*                                          APP INITIALZATION TASK
*
* This is the first application task created, before starting.
* It can create other tasks.
*
*********************************************************************************************************
*/

#define      INIT_TASK_STK_SIZE      TASK_STK_SIZE_STD

void Init_Task(void *pdata)
{
    int err = 0;
    int exit_code = 0;

    /* Create mutex for console access before putstr() can be used. */
    ConsMutex = xSemaphoreCreateMutex();
    if (ConsMutex == NULL)
        goto done;

    putline("[Init_Task] Started!");

    /* Create event flag group for task termination. */
    putline("[Init_Task] Creating event flag group for task termination.");
    TaskTermFlags = xEventGroupCreate();
    if (TaskTermFlags == NULL) {
        putline("...FAILED .1!");
        goto done;
    }

    /* Create queue for sequence of counts. */
    putline("[Init_Task] Creating queue for sequence of counts.");
    Queue = xQueueCreate(QUEUE_SIZE, sizeof(unsigned));
    if (Queue == NULL) {
        putline("...FAILED .2!");
        err = 101;
        goto done;
    }

    /* Create reporting task. */
    putline("[Init_Task] Creating reporting task Report_Task.");

    // This version supplies stack buffer (not auto-allocated on heap); just for testing
	//err = xTaskGenericCreate(Report_Task, "Report_Task", REPORT_TASK_STK_SIZE, NULL, REPORT_TASK_PRIO, Report_Task_TCB, Report_Task_Stack, NULL);
    err = xTaskCreate(Report_Task, "Report_Task", REPORT_TASK_STK_SIZE, NULL, REPORT_TASK_PRIO, &Report_Task_TCB);
    if (err != pdPASS)
    {
        putline("...FAILED! .3");
        goto done;
    }

    /* Create counting task. */
    putline("[Init_Task] Creating counting task Count_Task.");
	err = xTaskCreate(Count_Task, "Count_Task", COUNT_TASK_STK_SIZE, NULL, COUNT_TASK_PRIO, &Count_Task_TCB);
    if (err != pdPASS)
    {
        putline("...FAILED! .4");
        goto done;
    }

#ifdef XT_BOARD
    /* Create display task, if board has a display. */
    if (xtbsp_display_exists()) {
        putline("[Init_Task] Creating display task Display_Task.");
    	err = xTaskCreate(Display_Task, "Display_Task", DISPLAY_TASK_STK_SIZE, NULL, DISPLAY_TASK_PRIO, NULL);
        if (err != pdPASS)
        {
            putline("...FAILED! .5");
            goto done;
        }
    }
#endif /* XT_BOARD */

    /* Wait for counting and reporting tasks to finish. */
    putline("[Init_Task] Waiting for counting and reporting tasks to finish.");
    xEventGroupWaitBits(TaskTermFlags, TASK_TERM_REPORT | TASK_TERM_COUNT, 0, pdTRUE, portMAX_DELAY);

done:
    /* Clean up and shut down. */
    exit_code = (err != pdPASS);
    putline("[Init_Task] Cleaning up resources and terminating.");

    vQueueDelete(Queue);
    vEventGroupDelete(TaskTermFlags);
    vSemaphoreDelete(ConsMutex);

#ifdef XT_SIMULATOR
    /* This string indicates test passed. */
    if (exit_code == 0) {
        putline("PASS");
    }
    /* Shut down simulator and report error code as exit code to host (0 = OK). */
    _exit(exit_code);
#endif

    /* Terminate this task. RTOS will continue to run timer, stats and idle tasks. */
    vTaskDelete(NULL);
    /* Does not reach here. */
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
int main_example(int argc, char *argv[])
#endif
{
    unsigned err = 0;

    #ifdef XT_BOARD
    /* Display waypoint for debugging. Display task will identify this example later. */
    xtbsp_display_string("main()");
    #endif

    putline("\nFreeRTOS example on Xtensa.");
    putline("Xtensa port version " XTENSA_PORT_VERSION_STRING);

    /* Create only the Init_Task here. It will create the others. */
    putline("[main] Creating first task Init_Task.");
	err = xTaskCreate(Init_Task, "Init_Task", INIT_TASK_STK_SIZE, NULL, INIT_TASK_PRIO, NULL);

	if (err != pdPASS)
    {
        putline("FAILED! main");
        return 1;
    }

    /* Start task scheduler */
    putline("[main] Starting multi-tasking.");
	vTaskStartScheduler();

    /* If we got here then scheduler failed. */
    putline("vTaskStartScheduler FAILED!");
    return 1;
}

