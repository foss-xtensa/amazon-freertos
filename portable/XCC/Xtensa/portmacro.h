
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

#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "xtensa_rtos.h"

#ifndef __ASSEMBLER__

#include <stdint.h>

#include <xtensa/tie/xt_core.h>
#include <xtensa/hal.h>
#include <xtensa/config/system.h>	/* required for XSHAL_CLIB */

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* Type definitions. */

#define portCHAR		int8_t
#define portFLOAT		float
#define portDOUBLE		double
#define portLONG		int32_t
#define portSHORT		int16_t
#define portSTACK_TYPE	uint32_t
#define portBASE_TYPE	int

typedef portSTACK_TYPE                 StackType_t;
typedef portBASE_TYPE                  BaseType_t;
typedef unsigned portBASE_TYPE	UBaseType_t;

#if( configUSE_16_BIT_TICKS == 1 )
	typedef uint16_t TickType_t;
	#define portMAX_DELAY ( TickType_t ) 0xffff
#else
	typedef uint32_t TickType_t;
	#define portMAX_DELAY ( TickType_t ) 0xffffffffUL
#endif
/*-----------------------------------------------------------*/

// portbenchmark
#include "portbenchmark.h"

// Critical section management. These cannot be nested. They should be used
// with a lot of care and cannot be called from interrupt context.
static inline void
portDISABLE_INTERRUPTS(void)
{
#if XCHAL_HAVE_INTERRUPTS
	XT_RSIL (XT_IRQ_LOCK_LEVEL);
#endif
	portbenchmarkINTERRUPT_DISABLE ();
}

static inline void
portENABLE_INTERRUPTS(void)
{
	portbenchmarkINTERRUPT_RESTORE (0);
#if XCHAL_HAVE_INTERRUPTS
	XT_RSIL (0);
#endif
}

// Nested critical sections. Nesting managed by FreeRTOS.
#define portCRITICAL_NESTING_IN_TCB	1

extern void vTaskEnterCritical(void);
extern void vTaskExitCritical(void);

#if portUSING_MPU_WRAPPERS

extern void vPortEnterCritical(void);
extern void vPortExitCritical(void);
#define portENTER_CRITICAL()        vPortEnterCritical()
#define portEXIT_CRITICAL()         vPortExitCritical()

#define portSTACK_ALIGNMENT         XCHAL_MPU_ALIGN
#define portPRIVILEGE_BIT           0x80000000UL
#define portIS_PRIVILEGED() \
    ({ \
        register unsigned code __asm("a2") = SYSCALL_is_priv; \
        __asm volatile ("syscall\n" \
                        : "+a"(code) :: "memory"); \
        code; \
    })
#define portRAISE_PRIVILEGE() \
    do { \
        register unsigned code __asm("a2") = SYSCALL_raise_priv; \
        __asm volatile ("syscall\n" \
                        : "+a"(code) :: "memory"); \
    } while (0)
#define portRESET_PRIVILEGE() (XT_WSR_PS(XT_RSR_PS() | PS_RING(1)))

#else

#define portENTER_CRITICAL()        vTaskEnterCritical()
#define portEXIT_CRITICAL()         vTaskExitCritical()
#define portPRIVILEGE_BIT           0UL

#endif

// These allow nested interrupt disabling and restoring via local registers or stack.
// They can be called from interrupts context.
static inline uint32_t
portENTER_CRITICAL_NESTED(void)
{
	uint32_t state;

#if XCHAL_HAVE_INTERRUPTS
	state = XT_RSIL (XT_IRQ_LOCK_LEVEL);
#else
	state = 0;
#endif
	portbenchmarkINTERRUPT_DISABLE ();
	return state;
}

static inline void
portEXIT_CRITICAL_NESTED(uint32_t state)
{
	portbenchmarkINTERRUPT_RESTORE (state);
#if XCHAL_HAVE_INTERRUPTS
	XT_WSR_PS (state);
	XT_RSYNC ();
#endif
}

// These FreeRTOS versions are similar to the nested versions above
#define portSET_INTERRUPT_MASK_FROM_ISR()            portENTER_CRITICAL_NESTED()
#define portCLEAR_INTERRUPT_MASK_FROM_ISR(state)     portEXIT_CRITICAL_NESTED(state)
BaseType_t xPortRaisePrivilege( void );

/*-----------------------------------------------------------*/

/* Architecture specifics. */
#define portSTACK_GROWTH			( -1 )
#define portTICK_PERIOD_MS			( ( TickType_t ) 1000 / configTICK_RATE_HZ )
#ifdef configBYTE_ALIGNMENT
#define portBYTE_ALIGNMENT			configBYTE_ALIGNMENT
#elif XCHAL_DATA_WIDTH < 16
#define portBYTE_ALIGNMENT			XCHAL_DATA_WIDTH
#else
#define portBYTE_ALIGNMENT			16
#endif
#define portNOP()					XT_NOP()
/*-----------------------------------------------------------*/

/* Fine resolution time */
#define portGET_RUN_TIME_COUNTER_VALUE()  xthal_get_ccount()

/* No need to do anything for the ccount timer. */
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() do {} while (0)

/* Kernel utilities. */
void vPortYield( void );
void _frxt_setup_switch( void );
#define portYIELD()       vPortYield()
#define portYIELD_FROM_ISR( xHigherPriorityTaskWoken )	\
	if ( ( xHigherPriorityTaskWoken ) != 0 ) {	\
		_frxt_setup_switch();			\
	}

/* Tickless idle */
#if ( configUSE_TICKLESS_IDLE != 0 )
#ifndef portSUPPRESS_TICKS_AND_SLEEP_ABS
extern void vPortSuppressTicksAndSleep( TickType_t target,
                                        TickType_t xExpectedIdleTime );
#define portSUPPRESS_TICKS_AND_SLEEP_ABS( target, xExpectedIdleTime ) \
        vPortSuppressTicksAndSleep( target, xExpectedIdleTime )
#endif
#endif

#endif //__ASSEMBLER__

/*-----------------------------------------------------------*/

/* Attributes for all the possible regions */

#define  portDFLT_MEM_TYPE              (XTHAL_MEM_BUFFERABLE | XTHAL_MEM_WRITEBACK)

#define  portDFLT_STACK_TYPE            portDFLT_MEM_TYPE
#define  portDFLT_STACK_ACCESS          XTHAL_AR_RWrw

#define  portDFLT_USERDEV_TYPE          XTHAL_MEM_DEVICE
#define  portDFLT_USERDEV_ACCESS        XTHAL_AR_RWXrwx

#define  portDFLT_KERNDEV_TYPE          XTHAL_MEM_DEVICE
#define  portDFLT_KERNDEV_ACCESS        XTHAL_AR_RWX

#define  portDFLT_SHARED_TYPE           portDFLT_MEM_TYPE
#define  portDFLT_SHARED_ACCESS         XTHAL_AR_RWXrwx

#define  portDFLT_KERNCODE_TYPE         portDFLT_MEM_TYPE
#define  portDFLT_KERNCODE_ACCESS       XTHAL_AR_RX

#define  portDFLT_KERNDATA_TYPE         portDFLT_MEM_TYPE
#define  portDFLT_KERNDATA_ACCESS       XTHAL_AR_RW

#define  portDFLT_UNUSED_MEM_TYPE       portDFLT_MEM_TYPE
#define  portDFLT_UNUSED_MEM_ACCESS     XTHAL_AR_RWXrx

#define portNUM_CONFIGURABLE_REGIONS    configNUM_CONFIGURABLE_REGIONS

#define portLEGACY_TASK_STACK_START     (((uint32_t)_bss_start) & -XCHAL_MPU_ALIGN)
#define portLEGACY_TASK_STACK_END       (((uint32_t)_bss_end + XCHAL_MPU_ALIGN - 1) & -XCHAL_MPU_ALIGN)

#define SYSCALL_raise_priv  10
#define SYSCALL_is_priv     12

#ifndef __ASSEMBLER__

/* vPortStoreTaskMPUSettings returns void instead of BaseType_t. Until it's fixed
 * errors from that function can be checked using this global variable
 */
extern volatile int xtMPUError;

/* xtMPUError values
 */
typedef enum {
  MPU_ERR_SETUP_MPU = 1,
  MPU_ERR_REGION_NOT_IN_RANGE,
  MPU_ERR_STACK_NOT_IN_RANGE,
  MPU_ERR_OVERLAP_OTHER_REGIONS,
  MPU_ERR_FIXED_REGION_NOT_ALIGNED,
  MPU_ERR_PRIVATE_NOT_ALIGNED,
  MPU_ERR_INIT_ENTRY,
  MPU_ERR_INIT_ENTRY_NOT_IN_MAP,
  MPU_ERR_INIT_MAP_NOT_VALID
} mpu_err_t;


#endif

/*---------- DO NOT EDIT MPU SETTINGS BELOW ----------------*/

#if portUSING_MPU_WRAPPERS

#ifndef XCHAL_HAVE_MPU
# define XCHAL_HAVE_MPU 0
#endif

#if (XCHAL_HAVE_MPU == 0)
# error "MPU Hardware required!"
#endif

#ifndef portNUM_CONFIGURABLE_REGIONS
# define portNUM_CONFIGURABLE_REGIONS 0
#endif

#if defined(configUSER_DEVICE_START) && defined(configUSER_DEVICE_END)
#  define portUSE_USER_DEVICE_SPACE	1
# else
#  define portUSE_USER_DEVICE_SPACE	0
#endif

#if defined(configSHARED_DATA_START) && defined(configSHARED_DATA_END) && (portUSE_FIXED_MPU_ENTRIES == 1)
#  define portUSE_SHARED_DATA	1
# else
#  define portUSE_SHARED_DATA	0
#endif

#if defined(configPRIVILEGE_DEVICE_START) && defined(configPRIVILEGE_DEVICE_END)
#  define portUSE_PRIVILEGED_DEVICE_SPACE	1
# else
#  define portUSE_PRIVILEGED_DEVICE_SPACE	0
#endif

#if defined(configLEGACY_TASK_STACK_START) && defined(configLEGACY_TASK_STACK_END)
#  define portLEGACY_UNPRIVILEGED_TASKS 1
# else
#  define portLEGACY_UNPRIVILEGED_TASKS 0
#endif

/* Number of MPU entries a restricted thread will use. */
#define portNUM_USED_MPU_ENTRIES     (1  /* entry zero*/                   + \
                                      2* ( portNUM_CONFIGURABLE_REGIONS    + \
                                           portUSE_USER_DEVICE_SPACE       + \
                                           portUSE_PRIVILEGED_DEVICE_SPACE + \
                                           portUSE_SHARED_DATA             + \
                                           portLEGACY_UNPRIVILEGED_TASKS   + \
                                           1 /*stack*/                     + \
                                           1 /*FreeRTOS code */            + \
                                           1 /*FreeRTOS data */              \
                                           ))

#define portNUM_MAX_SWAPPED_MPU_PAIRS (portNUM_CONFIGURABLE_REGIONS + \
                                       portLEGACY_UNPRIVILEGED_TASKS + 1)

#if XCHAL_MPU_ENTRIES < portNUM_USED_MPU_ENTRIES
# error "Require MPU with at least portNUM_USED_MPU_ENTRIES foreground entries"
#endif

#ifndef __ASSEMBLER__
extern uint32_t privileged_data_start[];
extern uint32_t privileged_data_end[];
extern uint32_t privileged_functions_start[];
extern uint32_t privileged_functions_end[];
extern uint32_t _bss_start[];
extern uint32_t _bss_end[];

#define portPRIVILEGED_CODE_START    ((uint32_t)privileged_functions_start & -XCHAL_MPU_ALIGN)
#define portPRIVILEGED_CODE_END      (((uint32_t)privileged_functions_end + (XCHAL_MPU_ALIGN - 1)) & -XCHAL_MPU_ALIGN)
#define portPRIVILEGED_DATA_START    ((uint32_t)privileged_data_start & -XCHAL_MPU_ALIGN)
#define portPRIVILEGED_DATA_END      (((uint32_t)privileged_data_end + (XCHAL_MPU_ALIGN - 1)) & -XCHAL_MPU_ALIGN)

typedef struct {
    // Define here mpu_settings, which is port dependent
    xthal_MPU_entry mpumap[portNUM_MAX_SWAPPED_MPU_PAIRS][2];
} xMPU_SETTINGS;

#endif //ASSEMBLER

#endif //portUSING_MPU_WRAPPERS

/*-----------------------------------------------------------*/

#ifndef __ASSEMBLER__

/* Task function macros as described on the FreeRTOS.org WEB site. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) void vFunction( void *pvParameters )

// porttrace
#if configUSE_TRACE_FACILITY_2
#include "porttrace.h"
#endif

// configASSERT_2 if requested
#if configASSERT_2
#include <stdio.h>
void exit(int);
#define configASSERT( x )   if (!(x)) { porttracePrint(-1); printf("\nAssertion failed in %s:%d\n", __FILE__, __LINE__); exit(-1); }
#endif


/* C library support -- only XCLIB and NEWLIB are supported. */

/* To enable thread-safe C library support, XT_USE_THREAD_SAFE_CLIB must be
   defined to be > 0 somewhere above or on the command line. */

#if (XT_USE_THREAD_SAFE_CLIB > 0u) && (XSHAL_CLIB == XTHAL_CLIB_XCLIB)
extern void vPortClibInit(void);

// No cleanup necessary at this time.
#define portCLEAN_UP_TCB(pxTCB)
#endif // XCLIB support

#if (XT_USE_THREAD_SAFE_CLIB > 0u) && (XSHAL_CLIB == XTHAL_CLIB_NEWLIB)
extern void vPortClibInit(void);

// This C library cleanup is not currently done by FreeRTOS when deleting a task
#include <stdio.h>
#define portCLEAN_UP_TCB(pxTCB)   vPortCleanUpTcbClib(&((pxTCB)->xNewLib_reent))
static inline void vPortCleanUpTcbClib(struct _reent *ptr)
{
    FILE * fp = &(ptr->__sf[0]);
    int i;
    for (i = 0; i < 3; ++i, ++fp) {
        fp->_close = NULL;
    }
}
#endif // NEWLIB support

#endif // __ASSEMBLER__

#ifdef __cplusplus
}
#endif

#endif /* PORTMACRO_H */

