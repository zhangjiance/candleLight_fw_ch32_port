/*
 * cmsis_device.h
 *
 * The handful of CMSIS intrinsics candleLight's util.h uses, mapped onto plain
 * RISC-V CSRs.  Replaces the HPMicro device header; ch32fun already supplies
 * the register map, so nothing vendor specific is left here.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "ch32fun.h"

#ifndef __disable_irq
#define __disable_irq()  do { __asm volatile ("csrc mstatus, %0" :: "r"(0x8U) : "memory"); } while (0)
#endif
#ifndef __enable_irq
#define __enable_irq()   do { __asm volatile ("csrs mstatus, %0" :: "r"(0x8U) : "memory"); } while (0)
#endif

static inline uint32_t ch32_csr_read_mstatus(void)
{
    uint32_t v;

    __asm volatile ("csrr %0, mstatus" : "=r"(v));
    return v;
}

/* CMSIS intrinsics candleLight's util.c uses. */
#ifndef __ISB
#define __ISB()  __asm volatile ("fence" ::: "memory")
#endif
#ifndef __DSB
#define __DSB()  __asm volatile ("fence" ::: "memory")
#endif
#ifndef __DMB
#define __DMB()  __asm volatile ("fence" ::: "memory")
#endif
#ifndef __BKPT
#define __BKPT(x) __asm volatile ("ebreak")
#endif
#ifndef __get_PRIMASK
static inline uint32_t __get_PRIMASK(void)
{
    return (ch32_csr_read_mstatus() & 0x8U) ? 0U : 1U;
}
#endif
