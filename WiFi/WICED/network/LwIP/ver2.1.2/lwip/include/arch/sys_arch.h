/*
 * sys_arch.h
 *
 *  Created on: 2021Äê5ÔÂ8ÈÕ
 *      Author: nwz
 */

#ifndef LWIP_INCLUDE_ARCH_SYS_ARCH_H_
#define LWIP_INCLUDE_ARCH_SYS_ARCH_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdint.h>

#if !LWIP_NO_STDINT_H
typedef uint8_t   u8_t;
typedef int8_t    s8_t;
typedef uint16_t  u16_t;
typedef int16_t   s16_t;
typedef uint32_t  u32_t;
typedef int32_t   s32_t;
typedef uintptr_t mem_ptr_t;
#endif

#define SYS_ARCH_TIMEOUT 0xffffffffUL
#define SYS_MBOX_NULL ((xQueueHandle)0)
#define SYS_SEM_NULL  ((xSemaphoreHandle)0)

typedef SemaphoreHandle_t /*@only@*/sys_sem_t;
typedef QueueHandle_t /*@only@*/sys_mbox_t;
typedef TaskHandle_t /*@only@*/sys_thread_t;


#endif /* LWIP_INCLUDE_ARCH_SYS_ARCH_H_ */
