
/*
 * FreeRTOS Kernel V10.0.0
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software. If you wish to use our Amazon
 * FreeRTOS name, please do so in a fair use way that does not cause confusion.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/*
 * Copyright (c) 2015-2019 Cadence Design Systems, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#if portUSING_MPU_WRAPPERS
# define MPU_WRAPPERS_INCLUDED_FROM_API_FILE
#endif

#include <stdlib.h>

#include <xtensa/hal.h>
#include <xtensa/config/core.h>
#if XCHAL_HAVE_INTERRUPTS
#include <xtensa/tie/xt_interrupt.h>
#endif

#include "xtensa_api.h"
#include "xtensa_rtos.h"

#include "FreeRTOS.h"
#include "task.h"

#if portUSING_MPU_WRAPPERS
/* Configure a number of standard MPU regions that are used by all tasks. */
extern BaseType_t prvSetupMPU( void ) PRIVILEGED_FUNCTION;

/*
 * Checks to see if being called from the context of an unprivileged task, and
 * if so raises the privilege level and returns false - otherwise does nothing
 * other than return true.
 */
BaseType_t xPortRaisePrivilege( void );

#endif

// Defined in xtensa_context.S.
extern void _xt_coproc_init( void );

// Defined in xtensa_vectors.S.
extern void _xt_task_start( void );

// Timer tick interval in cycles.
static uint32_t xt_tick_cycles;
TickType_t xMaxSuppressedTicks;

static uint32_t xt_tick_count;

#if ( configUSE_TICKLESS_IDLE != 0 )
// Flag to indicate tick handling should be skipped.
static volatile uint32_t xt_skip_tick;
#endif

// Duplicate of inaccessible xSchedulerRunning.
uint32_t port_xSchedulerRunning = 0U;

// Interrupt nesting level.
uint32_t port_interruptNesting  = 0U;

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

//-----------------------------------------------------------------------------
// Tick timer interrupt handler.
//-----------------------------------------------------------------------------
static void xt_tick_handler( void )
{
    uint32_t diff;

#if ( configUSE_TICKLESS_IDLE != 0 )
    if ( xt_skip_tick )
    {
        vTaskStepTick( xt_skip_tick );
        xt_tick_count += xt_skip_tick;
        xt_skip_tick = 0;
    }
#endif

    do
    {
        BaseType_t ret;
        uint32_t   interruptMask;
        uint32_t   ulOldCCompare = xt_get_ccompare( XT_TIMER_INDEX );

        // Set CCOMPARE for next tick.
        xt_set_ccompare( XT_TIMER_INDEX, ulOldCCompare + xt_tick_cycles );

        portbenchmarkIntLatency();

        // Interrupts upto configMAX_SYSCALL_INTERRUPT_PRIORITY must be
        // disabled before calling xTaskIncrementTick as it accesses the
        // kernel lists.
        interruptMask = portSET_INTERRUPT_MASK_FROM_ISR();
        {
            ret = xTaskIncrementTick();
            ++xt_tick_count;
        }
        portCLEAR_INTERRUPT_MASK_FROM_ISR( interruptMask );

        portYIELD_FROM_ISR( ret );

        diff = xt_get_ccount() - ulOldCCompare;
    }
    while ( diff > xt_tick_cycles );
}

//-----------------------------------------------------------------------------
// Tick timer init. Install interrupt handler, set up first tick, and
// enable timer interrupt.
//-----------------------------------------------------------------------------
static void xt_tick_timer_init( void )
{
    // Compute the number of cycles per tick.
    #ifdef XT_CLOCK_FREQ
    xt_tick_cycles = ( XT_CLOCK_FREQ / XT_TICK_PER_SEC );
    #else
    #ifdef XT_BOARD
    xt_tick_cycles = xtbsp_clock_freq_hz() / XT_TICK_PER_SEC;
    #else
    #error "No way to obtain processor clock frequency"
    #endif
    #endif

    xMaxSuppressedTicks = 0xFFFFFFFFU / xt_tick_cycles;
    xt_set_interrupt_handler( XT_TIMER_INTNUM, (xt_handler) xt_tick_handler, 0 );
    xt_set_ccompare( XT_TIMER_INDEX, xthal_get_ccount() + xt_tick_cycles );
    xt_tick_count = xTaskGetTickCount();
    xt_interrupt_enable( XT_TIMER_INTNUM );
}

//-----------------------------------------------------------------------------
// Tick timer stop. Disable timer interrupt and clear ccompare register.
//-----------------------------------------------------------------------------
static void xt_tick_timer_stop( void )
{
    xt_interrupt_disable( XT_TIMER_INTNUM );
    xt_set_ccompare( XT_TIMER_INDEX, 0 );
}

//-----------------------------------------------------------------------------
// Start the scheduler.
//-----------------------------------------------------------------------------
BaseType_t xPortStartScheduler( void )
{
    // Interrupts are disabled at this point and stack contains PS with
    // enabled interrupts when task context is restored.

    #if XCHAL_CP_NUM > 0
    // Initialize co-processor management for tasks. Leave CPENABLE alone.
    _xt_coproc_init();
    #endif

    // Set up and enable timer tick.
    xt_tick_timer_init();

    #if XT_USE_THREAD_SAFE_CLIB
    // Init C library
    vPortClibInit();
    #endif

    #if portUSING_MPU_WRAPPERS
    // Setup MPU
    if (prvSetupMPU() == pdFALSE)
       return pdFALSE;
    #endif

    port_xSchedulerRunning = 1U;

    // Cannot be directly called from C; never returns
    __asm__ volatile ("call0    _frxt_dispatch\n");

    // Should never get here.
    return pdFALSE;
}

//-----------------------------------------------------------------------------
// Stop the scheduler.
//-----------------------------------------------------------------------------
void vPortEndScheduler( void )
{
    xt_tick_timer_stop();
    port_xSchedulerRunning = 0U;
}

//-----------------------------------------------------------------------------
// Stack initialization.
// Reserve coprocessor save area if needed, construct a dummy stack frame and
// populate it for task startup. Return adjusted top-of-stack pointer, which
// is also the pointer to the dummy stack frame.
// (NOTE: the value returned from this function is expected to be stored in
// pxTCB->pxTopOfStack. In the task wrapper code, we will copy this value into
// pxTCB->pxEndOfStack, which will then be treated as the coprocessor state
// area pointer.
//-----------------------------------------------------------------------------
#if portUSING_MPU_WRAPPERS
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack,
				    TaskFunction_t pxCode,
				    void *pvParameters,
				    BaseType_t xRunPrivileged )
#else
StackType_t *pxPortInitialiseStack( StackType_t * pxTopOfStack,
                                    TaskFunction_t pxCode,
                                    void * pvParameters )
#endif
{
    StackType_t *sp, *tp;
    XtExcFrame  *frame;
    #if XCHAL_CP_NUM > 0
    uint32_t *p;
    #endif

    // Allocate enough space for coprocessor state, align base address. This is the
    // adjusted top-of-stack.
    sp = (StackType_t *) ((((uint32_t) pxTopOfStack) - (uint32_t) XT_CP_SIZE) & ~0xF);

    // Allocate interrupt stack frame. XT_STK_FRMSZ is always a multiple of 16 bytes
    // so 16-byte alignment is ensured.
    tp = sp - (XT_STK_FRMSZ/sizeof(StackType_t));
    frame = (XtExcFrame *) tp;

    // Clear the frame (do not use memset() because we don't depend on C library).
    for (; tp < sp; ++tp)
    {
        *tp = 0;
    }

    // Explicitly initialize certain saved registers.
    frame->pc   = (UBaseType_t) pxCode;             // task entrypoint
    frame->a0   = 0;                                // to terminate GDB backtrace
    frame->a1   = (UBaseType_t) sp;                 // physical top of stack frame
    frame->exit = (UBaseType_t) _xt_task_start;     // task start wrapper

    // Set initial PS to int level 0, EXCM disabled ('rfe' will enable), user mode.
    // Also set entry point argument parameter.
    #ifdef __XTENSA_CALL0_ABI__
    frame->a2 = (UBaseType_t) pvParameters;
    frame->ps = PS_UM | PS_EXCM;
    #else
    // + for windowed ABI also set WOE and CALLINC (pretend task was 'call4'd).
    frame->a6 = (UBaseType_t) pvParameters;
    frame->ps = PS_UM | PS_EXCM | PS_WOE | PS_CALLINC(1);
    #endif
    #if portUSING_MPU_WRAPPERS
    if(!xRunPrivileged) {
       frame->ps |= (1 << PS_RING_SHIFT);
    }
    #endif

    #ifdef XT_USE_SWPRI
    // Set the initial virtual priority mask value to all 1's.
    frame->vpri = 0xFFFFFFFF;
    #endif

    #if XCHAL_CP_NUM > 0
    // Init the coprocessor save area (see xtensa_context.h).
    p = (uint32_t *) sp;
    p[0] = 0;
    p[1] = 0;
    p[2] = (((uint32_t) p) + 12 + XCHAL_TOTAL_SA_ALIGN - 1) & -XCHAL_TOTAL_SA_ALIGN;
    #endif

    return (StackType_t *) frame;
}

//-----------------------------------------------------------------------------
// Tickless idle support. Suppress N ticks and sleep when directed by kernel.
//-----------------------------------------------------------------------------
#if ( configUSE_TICKLESS_IDLE != 0 )
void vPortSuppressTicksAndSleep( TickType_t target, TickType_t xExpectedIdleTime )
{
    eSleepModeStatus eSleepStatus;
    uint32_t ps;

    // Lock out all interrupts. Otherwise reading and using ccount can
    // get messy. Shouldn't be a problem here since we are about to go
    // to sleep, and the waiti will re-enable interrupts shortly.
    ps = XT_RSIL( XT_IRQ_LOCK_LEVEL );

    eSleepStatus = eTaskConfirmSleepModeStatus();
    if ( eSleepStatus == eAbortSleep )
    {
        // Abort, fall through.
    }
    else
    {
        uint32_t num_cycles;
        uint32_t first_blocked_tick;
        uint32_t ccompare;
        uint32_t skip_tick;
        uint32_t now;

        xExpectedIdleTime = target - xt_tick_count;
        // Compute number of cycles to sleep for, capped by max limit.
        // we use one less than the number of ticks because we are already
        // partway through the current tick. This is adjusted later below.
        if ( xExpectedIdleTime > xMaxSuppressedTicks )
        {
            skip_tick = xMaxSuppressedTicks - 1U;
        }
        else
        {
            skip_tick = xExpectedIdleTime - 1U;
        }

        num_cycles = xt_tick_cycles * skip_tick;
        first_blocked_tick = xt_get_ccompare( XT_TIMER_INDEX );
        xt_skip_tick = skip_tick;

        // Set up for timer interrupt and sleep.
        ccompare = first_blocked_tick + num_cycles;
        xt_set_ccompare( XT_TIMER_INDEX, ccompare );
        XT_WAITI( 0 );
        XT_RSIL( XT_IRQ_LOCK_LEVEL );

        skip_tick = xt_skip_tick;
        now = xt_get_ccount();

        // Awakened by non-timer interrupt, update tick counter here.

        if ( skip_tick )
        {
            // If there's more than a tick period from now to the timer
            // deadline try to move deadline to the next possible tick.
            // Otherwise update tick count for the passed ticks, but don't
            // change the deadline.

            if ( ccompare - now > xt_tick_cycles &&
                 ccompare - now <= INT32_MAX )
            {
                uint32_t prev_tick = first_blocked_tick - xt_tick_cycles;
                uint32_t actual_cycles = now - prev_tick;
                uint32_t ticks = actual_cycles / xt_tick_cycles;
                uint32_t diff;

                ccompare = first_blocked_tick + ticks * xt_tick_cycles;

                do
                {
                    vTaskStepTick( ticks );
                    xt_tick_count += ticks;
                    xt_set_ccompare( XT_TIMER_INDEX, ccompare );
                    diff = xt_get_ccount() - ccompare;
                    ccompare += xt_tick_cycles;
                    ticks = 1;

                } while ( diff <= INT32_MAX );
            }
            else
            {
                vTaskStepTick( skip_tick );
                xt_tick_count += skip_tick;
            }

            xt_skip_tick = 0;
        }
    }

    XT_WSR_PS( ps );
}
#endif

#if portUSING_MPU_WRAPPERS
extern void vPortResetPrivilege(BaseType_t previous);
void vPortEnterCritical( void )
{
  BaseType_t xRunningPrivileged = xPortRaisePrivilege();

  vTaskEnterCritical();
  vPortResetPrivilege( xRunningPrivileged );
}

void vPortExitCritical( void )
{
  BaseType_t xRunningPrivileged = xPortRaisePrivilege();

  vTaskExitCritical();
  vPortResetPrivilege( xRunningPrivileged );
}
#endif
