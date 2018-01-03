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

#include    <stdio.h>
#include    <stdlib.h>

#include "FreeRTOS.h"


#ifdef XT_BOARD
#include    <xtensa/xtbsp.h>
#endif
#include    <xtensa/hal.h>

#include "xtensa_api.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

/* Task priorities. */
/*
 * NOTE: the consumer runs at a higher priority than the producer so as to
 * avoid overflowing the message queue.
*/
#define INIT_TASK_PRIO    5
#define TASK_0_PRIO       6
#define TASK_1_PRIO       8
#define TASK_2_PRIO       7

/* Test iterations */
#define TEST_ITER         10000

/* Uncomment this to exercise the s/w prioritization */
//#define XT_USE_SWPRI      1

/* SW interrupt number (computed at runtime) */
uint32_t uiSwIntNum = 0;

#ifdef XT_USE_SWPRI
/* Second (higher priority) SW interrupt number */
uint32_t uiSwInt2Num = 0;
volatile int iFlag = 0;
#endif

/* Variables used by exception test */
volatile int iExcCount = 0;

/* Stack size for tasks that do not use the C library. */
#define     TASK_STK_SIZE_MIN       (XT_STACK_MIN_SIZE)
/* Stack size for tasks that use the C library and/or the coprocessors */
#define     TASK_STK_SIZE_STD       (0x1000 + XT_STACK_EXTRA_CLIB)

/* Queue for passing count. */
#define     QUEUE_SIZE          16
#define		SEM_CNT				32
QueueHandle_t	xQueue;

/* Semaphore for test. */
SemaphoreHandle_t xSem;

/* Define the counters used in the demo application...  */

uint32_t  uiTask0Counter;
uint32_t  uiTask1Counter;
uint32_t  uiTask1MessagesSent;
uint32_t  uiTask2Counter;
uint32_t  uiTask2MessagesReceived;

/* Task TaskHanles */
TaskHandle_t      xTask0Handle;
TaskHandle_t      xTask1Handle;
TaskHandle_t      xTask2Handle;

/*
*******************************************************************************
* Illegal instruction handler.
*******************************************************************************
*/
void illegalInstHandler(XtExcFrame *frame)
{
    /* Move the PC past the illegal (3-byte) instruction */
    frame->pc += 3;
    puts("e");
}


/*
*******************************************************************************
* SW interrupt handler. Argument is pointer to semaphore handle.
* Interrupt is cleared in dispatcher so no need to do that here.
*******************************************************************************
*/
void softwareIntHandler(void* arg)
{
    SemaphoreHandle_t xSem = (SemaphoreHandle_t)arg;
    int err;

    /* Signal the semaphore */
	err = xSemaphoreGive(xSem);

#ifdef XT_USE_SWPRI
    if (uiSwInt2Num) {
        iFlag = 44;
        putchar('<');
        /* Higher priority handler should run right away and change flag */
        xt_set_intset(1 << uiSwInt2Num);
        if (iFlag == 44) {
            puts("Error: higher priority handler not run");
            exit(-1);
        }
        putchar('>');
    }
#endif
}


#ifdef XT_USE_SWPRI
/* Handler for higher priority interrupt (at same level but prioritized
   higher in software).
*/
void softwareHighHandler(void* arg)
{
    iFlag = 55;
    putchar('H');
}
#endif


/*
*******************************************************************************
* Task 0
*******************************************************************************
*/
static void Task0(void* pvData)
{
    //int err;

    /* This task simply sits in while-forever-sleep loop.  */
    while(1)
    {
        #ifdef DEMO_USE_PRINTF
        if (uiTask0Counter % 10 == 0) {
            printf(" %8u iterations of task_0,  system clock = %8u ticks\n",
                    uiTask0Counter, xTaskGetTickCount());
        }
        printf("."); fflush(stdout);
        #endif

        /* Increment the task counter.  */
        uiTask0Counter++;

        /* Sleep for 10 ticks.  */
		vTaskDelay(10);
    }
}


/*
*******************************************************************************
* Task 1
*******************************************************************************
*/
static void Task1(void *pvData)
{
    BaseType_t err = pdPASS;
    uint32_t i;

    /* Set up interrupt handling and enable interrupt */
    xt_set_interrupt_handler(uiSwIntNum, softwareIntHandler, (void*)xSem);
    xt_ints_on(1 << uiSwIntNum);

#ifdef XT_USE_SWPRI
    /* Set up the higher priority interrupt if available */
    if (uiSwInt2Num) {
        xt_set_interrupt_handler(uiSwInt2Num, softwareHighHandler, 0);
        xt_ints_on(1 << uiSwInt2Num);
    }
#endif

    /* Now send messages to task 2 and signal it */
    for (i = 0; i < TEST_ITER; i++) {
        /* Increment the task counter */
        uiTask1Counter++;

        /* Send message to queue 0 */
		err = xQueueSend(xQueue, &uiTask1MessagesSent, portMAX_DELAY);
        if (err != pdPASS)
            break;

        uiTask1MessagesSent++;
        putchar('+');

        /* Now trigger the interrupt handler */
        xt_set_intset(1 << uiSwIntNum);
    }

#ifdef DEMO_USE_PRINTF
    printf("Thread_1: sent %u messages\n", uiTask1MessagesSent);
#else
    puts("task 1 finish");
#endif
    vTaskDelete(NULL);
}


/*
*******************************************************************************
* Task 2
*******************************************************************************
*/
static void Task2(void* pvData)
{
    uint32_t  uiReceivedMessage;
    uint32_t  i;
    BaseType_t size;
    uint32_t  iok = 0;
    uint32_t  eok = 0;
    BaseType_t err = pdPASS;

    /* This task retrieves messages placed on the queue by task 1 */
    for (i = 0; i < TEST_ITER; i++) {
        /* Wait for the semaphore to be signalled */
		xSemaphoreTake(xSem, portMAX_DELAY);
        /* Increment the task counter */
        uiTask2Counter++;

        /* Retrieve a message from the queue */
		err = xQueueReceive(xQueue, &uiReceivedMessage, portMAX_DELAY);

        /* Check completion status and make sure the message is what we
           expected */
        if ((err != pdPASS) || (uiReceivedMessage != uiTask2MessagesReceived))
            break;

        /* Otherwise, all is okay.  Increment the received message count */
        uiTask2MessagesReceived++;
        putchar('-');
    }

#ifdef DEMO_USE_PRINTF
    printf("Thread 2: recd %u messages\n", uiTask2MessagesReceived);
#else
    puts("task 2 finish");
#endif

    if (uiTask1MessagesSent == uiTask2MessagesReceived) {
        puts("Interrupt Test PASS");
        iok = 1;
    }
    else {
        puts("Interrupt Test FAIL");
    }

    /* Now test exception handling */

    /* Install handler */
    xt_set_exception_handler(EXCCAUSE_ILLEGAL, illegalInstHandler);

    /* Force an illegal instruction. The 3-byte form of the illegal
       instruction should be present in all configs. The handler will
       push the PC past the offending instruction, so the loop should
       complete successfully.
     */
    for (i = 0; i < 10; i++) {
        asm volatile
           ("movi    a4, iExcCount\n"
            "_ill\n"
            "l32i    a5, a4, 0\n"
            "addi    a5, a5, 1\n"
            "s32i    a5, a4, 0\n" : :: "a4", "a5");
    }

    if (iExcCount == 10) {
        puts("Exception Test PASS");
        eok = 1;
    }
    else {
        puts("Exception Test FAIL");
    }

    if (iok && eok) {
        puts("Xtensa interrupt/exception test (xt_intr) PASSED!");
    }

    exit(0);
}


/*
*********************************************************************************************************
*                                          APP INITIALZATION TASK
*
*********************************************************************************************************
*/

static void initTask(void* pvData)
{
    BaseType_t err = 0;

    /* Create test semaphore. */
    xSem = xSemaphoreCreateCounting( SEM_CNT, 0 );
    /* Create queue for sequence of counts. */
	xQueue = xQueueCreate(QUEUE_SIZE, 1 * sizeof(uint32_t));

    /* Create the 3 test tasks. */
	err = xTaskCreate(Task0, "Task0", TASK_STK_SIZE_STD, (void*)0, TASK_0_PRIO, xTask0Handle);

    if (err != pdPASS)
    {
        puts("Task 0 create failed.");
        goto done;
    }

	err = xTaskCreate(Task1, "Task1", TASK_STK_SIZE_STD, (void*)0, TASK_1_PRIO, xTask1Handle);

    if (err != pdPASS)
    {
        puts("Task 1 create failed.");
        goto done;
    }

	err = xTaskCreate(Task2, "Task2", TASK_STK_SIZE_STD, (void*)0, TASK_2_PRIO, xTask2Handle);

    if (err != pdPASS)
    {
        puts("Task 2 create failed.");
        goto done;
    }

done:
    /* Clean up and shut down. */
    if (err != pdPASS) {
        exit(err);
    }

    vTaskDelete(NULL);
}

/*
*********************************************************************************************************
*                                             C ENTRY POINT
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
int main_xt_intr(int argc, char *argv[])
#endif
{
	uint32_t uiSwInts = 0;
    uint32_t x = 0;
	 /* Unbuffer stdout */
    setbuf(stdout, 0);

    puts("Xtensa interrupt/exception test (xt_intr) running...");

    /* Find a usable sw interrupt at the highest <= EXCM_LEVEL */

#if XCHAL_EXCM_LEVEL >= 4
    uiSwInts = XCHAL_INTLEVEL4_MASK & XCHAL_INTTYPE_MASK_SOFTWARE;
#endif

    if (uiSwInts == 0) {
#if XCHAL_EXCM_LEVEL == 3
        uiSwInts = XCHAL_INTLEVEL3_MASK & XCHAL_INTTYPE_MASK_SOFTWARE;
#endif
    }

    if (uiSwInts == 0) {
#if XCHAL_EXCM_LEVEL == 2
        uiSwInts = XCHAL_INTLEVEL2_MASK & XCHAL_INTTYPE_MASK_SOFTWARE;
#endif
    }

    if (uiSwInts == 0) {
        uiSwInts = XCHAL_INTLEVEL1_MASK & XCHAL_INTTYPE_MASK_SOFTWARE;
    }

    if (uiSwInts == 0) {
        puts("Can't find any sw interrupts at <= EXCM_LEVEL, test cannot run.\n");
        /* keep regressions happy */
        puts("Xtensa interrupt/exception test (xt_intr) PASSED!");
        exit(0);
    }

    /* Pick the first (lowest numbered) interrupt */
    while (!(uiSwInts & 0x1)) {
        x++;
        uiSwInts >>= 1;
    }
    uiSwIntNum = x;

#ifdef XT_USE_SWPRI
    /* Try to find another one (this would be higher priority) */
    uiSwInts >>= 1;
    x++;

    if (uiSwInts) {
        while (!(uiSwInts & 0x1)) {
            x++;
            uiSwInts >>= 1;
        }
        uiSwInt2Num = x;
    }

    if (uiSwInt2Num == 0) {
        puts("Warning: second sw interrupt not found, sw priority cannot be tested.");
    }
#endif
	xTaskCreate( initTask, "initTask", configMINIMAL_STACK_SIZE, (void *)NULL, INIT_TASK_PRIO, NULL );
    /* Finally start the scheduler. */
	vTaskStartScheduler();
	/* Will only reach here if there is insufficient heap available to start
	the scheduler. */
	for( ;; );
	return 0;
}
