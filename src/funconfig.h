/*
 * funconfig.h
 *
 * ch32fun configuration for the CH32L103 candleLight port.
 *
 * The clock tree is a board property: boards/<board>/board.cmake sets the
 * crystal and the resulting frequencies.  The values below are only fallbacks.
 *
 *     HSE 8 MHz  ->  PLL x12  ->  SYSCLK = HCLK = 96 MHz, APB1 = HCLK/2 = 48 MHz
 *
 * CH32L103 is plain rv32imac: there is no "xw" extension, so the WCH hardware
 * prologue (FUNCONF_ENABLE_HPE) is not available and interrupt handlers are
 * ordinary functions decorated by ch32fun.
 */
#ifndef _FUNCONFIG_H
#define _FUNCONFIG_H

#define CH32L103 1

/* ------------------------------------------------------------------ *
 * Clock tree (board.cmake wins)
 * ------------------------------------------------------------------ */
#ifndef FUNCONF_USE_HSI
#define FUNCONF_USE_HSI 0
#endif
#ifndef FUNCONF_USE_HSE
#define FUNCONF_USE_HSE 1
#endif
#ifndef FUNCONF_USE_PLL
#define FUNCONF_USE_PLL 1
#endif
#ifndef FUNCONF_PLL_MULTIPLIER
#define FUNCONF_PLL_MULTIPLIER 12
#endif
#ifndef FUNCONF_SYSTEM_CORE_CLOCK
#define FUNCONF_SYSTEM_CORE_CLOCK 96000000
#endif
#ifndef HSE_VALUE
#define HSE_VALUE 8000000
#endif

/* ------------------------------------------------------------------ *
 * Core / interrupts
 *
 * No xw extension on this part: the WCH fast interrupt entry cannot be used,
 * so handlers are plain functions (INTERRUPT_DECORATOR expands to nothing).
 * ------------------------------------------------------------------ */
#define FUNCONF_ENABLE_HPE 0

/* SysTick counts HCLK cycles; it is the time base for gs_usb timestamps. */
#define FUNCONF_SYSTICK_USE_HCLK 1

/* ------------------------------------------------------------------ *
 * printf transport: USART1 (PA9), 115200 8N1, development builds only.
 * ------------------------------------------------------------------ */
#define FUNCONF_USE_DEBUGPRINTF 0
#define FUNCONF_USE_UARTPRINTF 1
#ifndef FUNCONF_UART_PRINTF_BAUD
#define FUNCONF_UART_PRINTF_BAUD 115200
#endif

/* ------------------------------------------------------------------ *
 * Size knobs: a fault handler that prints costs more than it saves here.
 * ------------------------------------------------------------------ */
#define FUNCONF_DEBUG_HARDFAULT 0
#define FUNCONF_SUPPORT_CONSTRUCTORS 0
#define FUNCONF_ICACHE_EN 0

#endif /* _FUNCONFIG_H */
