/*
 * board.c - board primitives for boards/ch32l103_candle.
 *
 * Implements board.h on top of the peripheral registers ch32fun declares
 * (ch32l103hw.h): pins and polarity come from board_config.h, the clock tree
 * from user/funconfig.h + board.cmake.  Nothing here goes through a vendor HAL.
 *
 * Bring-up order matters: ch32fun's handle_reset() jumps straight to main()
 * with the system clock still on HSI, so main() calls SystemInit() before
 * board_init().
 */
#include "board.h"
#include "board_config.h"

#include "ch32fun.h"

/* HCLK cycles per millisecond, fixed by the (static) clock tree. */
static uint32_t cycles_per_ms;

/* the single periodic callback the application may install (may be NULL) */
static board_tick_cb tick_cb;

/*
 * SysTick is a free running 64bit up counter on the CH32L103 and is used as the
 * millisecond time base without an interrupt (the same scheme as the CH32V30x
 * board).  Only the low 32 bits are read: at 96 MHz that wraps every ~44 s,
 * which board_delay_ms()'s wrapping subtraction copes with, and it keeps the
 * divide out of libgcc (both operands stay 32 bit, so no __udivdi3).
 */
static void board_init_systick(void)
{
    cycles_per_ms = (uint32_t)(FUNCONF_SYSTEM_CORE_CLOCK / 1000U);

    SysTick->CTLR = 0;
    SysTick->SR = 0;
    SysTick->CNT = 0;
    SysTick->CMP = 0xffffffffffffffffull;
    SysTick->CTLR = 1 | (1 << 2);   /* STE | STCLK: count at HCLK */
}

uint32_t board_time_ms(void)
{
    return (uint32_t)SysTick->CNT / cycles_per_ms;
}

void board_delay_ms(uint32_t ms)
{
    uint32_t start = board_time_ms();

    while ((uint32_t)(board_time_ms() - start) < ms) {
        continue;
    }
}

/* -------------------------------------------------------------------------- */
/* Status LED (LED1 of the board)                                             */
/* -------------------------------------------------------------------------- */
#if BOARD_HAS_LED
static uint8_t led_level(uint8_t on)
{
    return BOARD_LED_ACTIVE_LOW ? (uint8_t)!on : on;
}

static void board_init_led(void)
{
    RCC->APB2PCENR |= BOARD_GPIO_RCC;

    /* Preload the off level (BOARD_LED_ACTIVE_LOW aware) in the output latch
     * before the pin becomes an output, so the LED does not flash on. */
    funDigitalWrite(BOARD_LED_PIN, led_level(0U) ? FUN_HIGH : FUN_LOW);

    funPinMode(BOARD_LED_PIN, GPIO_CFGLR_OUT_10Mhz_PP);
}
#if BOARD_HAS_LED2
/* Second LED (LED2).  Same off-level convention as LED1; a board whose second
 * LED has the opposite polarity overrides it with BOARD_LED2_ACTIVE_LOW. */
#ifndef BOARD_LED2_ACTIVE_LOW
#define BOARD_LED2_ACTIVE_LOW BOARD_LED_ACTIVE_LOW
#endif

static uint8_t led2_level(uint8_t on)
{
    return BOARD_LED2_ACTIVE_LOW ? (uint8_t)!on : on;
}

static void board_init_led2(void)
{
    RCC->APB2PCENR |= BOARD_GPIO_RCC;

    /* Preload the off level before the pin becomes an output, so the LED does
     * not flash on at boot.  Without this the second LED stays an input and the
     * application's TX/RX indicators never light. */
    funDigitalWrite(BOARD_LED2_PIN, led2_level(0U) ? FUN_HIGH : FUN_LOW);

    funPinMode(BOARD_LED2_PIN, GPIO_CFGLR_OUT_10Mhz_PP);
}
#endif /* BOARD_HAS_LED2 */
#endif /* BOARD_HAS_LED */

void board_led_write(uint8_t state)
{
#if BOARD_HAS_LED
    const uint8_t level = led_level((state != 0U) ? 1U : 0U);

    funDigitalWrite(BOARD_LED_PIN, level ? FUN_HIGH : FUN_LOW);
#else
    (void)state;
#endif
}

void board_led_toggle(void)
{
#if BOARD_HAS_LED
    static uint8_t on;

    on ^= 1U;
    board_led_write(on);
#endif
}

/* -------------------------------------------------------------------------- */
/* CAN transceiver termination                                                */
/* -------------------------------------------------------------------------- */
#if BOARD_HAS_TERMINATION
static bool termination_on;

static uint8_t term_level(bool on)
{
    return BOARD_TERM_ACTIVE_LOW ? (uint8_t)!on : (uint8_t)on;
}

void board_set_termination(bool on)
{
    termination_on = on;
    funDigitalWrite(BOARD_TERM_PIN, term_level(on) ? FUN_HIGH : FUN_LOW);
}

bool board_get_termination(void)
{
    return termination_on;
}

static void board_init_termination(void)
{
    RCC->APB2PCENR |= BOARD_GPIO_RCC;

    /* Off by default, like the application's board: the host turns it on, and
     * an unpowered bus with 120 ohm across it is better than a second
     * terminator on a bus that already has one.  Preload the level before the
     * pin becomes an output. */
    funPinMode(BOARD_TERM_PIN, GPIO_CFGLR_OUT_10Mhz_PP);
    board_set_termination(false);
}
#else /* !BOARD_HAS_TERMINATION */
void board_set_termination(bool on)
{
    (void)on;
}

bool board_get_termination(void)
{
    return false;
}
#endif /* BOARD_HAS_TERMINATION */


/* -------------------------------------------------------------------------- */
/* BOOT button                                                                */
/* -------------------------------------------------------------------------- */
#if BOARD_HAS_BOOT_BUTTON
static void board_init_boot_button(void)
{
    RCC->APB2PCENR |= BOARD_GPIO_RCC;

    /* The pull direction of a GPIO_CFGLR_IN_PUPD pin is the output latch:
     * 1 = pull-up.  The released level must be the "run the application"
     * level. */
    if (BOARD_BOOT_ACTIVE_LOW) {
        funDigitalWrite(BOARD_BOOT_PIN, FUN_HIGH);   /* pull-up */
    } else {
        funDigitalWrite(BOARD_BOOT_PIN, FUN_LOW);    /* pull-down */
    }
    funPinMode(BOARD_BOOT_PIN, GPIO_CFGLR_IN_PUPD);
}
#endif /* BOARD_HAS_BOOT_BUTTON */

bool board_read_boot_pin(void)
{
#if BOARD_HAS_BOOT_BUTTON
    const bool level = funDigitalRead(BOARD_BOOT_PIN) != 0;

    return BOARD_BOOT_ACTIVE_LOW ? !level : level;
#else
    return false;
#endif
}

/* -------------------------------------------------------------------------- */
/* Application UART                                                           */
/* -------------------------------------------------------------------------- */
void board_init_app_uart(void)
{
#if BOARD_HAS_APP_UART
    RCC->APB2PCENR |= BOARD_GPIO_RCC;
    RCC->APB1PCENR |= BOARD_APP_UART_RCC;

    /* TX idle high before the alternate function takes over, so the line does
     * not glitch low (the far end would read that as a start bit). */
    funDigitalWrite(BOARD_APP_UART_TX_PIN, FUN_HIGH);
    funPinMode(BOARD_APP_UART_TX_PIN, GPIO_CFGLR_OUT_50Mhz_AF_PP);

    /* RX with pull-up. */
    funDigitalWrite(BOARD_APP_UART_RX_PIN, FUN_HIGH);
    funPinMode(BOARD_APP_UART_RX_PIN, GPIO_CFGLR_IN_PUPD);
#endif /* BOARD_HAS_APP_UART */
}

/* -------------------------------------------------------------------------- */
/* Periodic tick                                                              */
/* -------------------------------------------------------------------------- */
void board_timer_create(uint32_t ms, board_tick_cb cb)
{
    tick_cb = cb;

    /*
     * Periodic update interrupt on the board's tick timer.  The prescaler gives
     * a 10 kHz tick so the period only depends on the milliseconds asked for,
     * independent of the clock (the TIM2..TIM4 cells count HCLK, which is what
     * the WCH examples prescale from too).
     */
    RCC->APB1PCENR |= BOARD_TICK_TIM_RCC;

    BOARD_TICK_TIM->PSC    = (uint16_t)((FUNCONF_SYSTEM_CORE_CLOCK / 10000U) - 1U);
    BOARD_TICK_TIM->ATRLR  = (uint16_t)((10000U * ms / 1000U) - 1U);
    BOARD_TICK_TIM->CNT    = 0;
    BOARD_TICK_TIM->INTFR  = (uint16_t)~TIM_FLAG_Update;
    BOARD_TICK_TIM->DMAINTENR |= TIM_IT_Update;
    NVIC_EnableIRQ(BOARD_TICK_TIM_IRQ);
    BOARD_TICK_TIM->CTLR1 |= TIM_CEN;
}

void BOARD_TICK_TIM_ISR(void) INTERRUPT_DECORATOR;
void BOARD_TICK_TIM_ISR(void)
{
    if ((BOARD_TICK_TIM->INTFR & TIM_FLAG_Update) == 0U) {
        return;
    }
    BOARD_TICK_TIM->INTFR = (uint16_t)~TIM_FLAG_Update;

    if (tick_cb) {
        tick_cb();
    }
}

/* -------------------------------------------------------------------------- */
/* Bring-up                                                                   */
/* -------------------------------------------------------------------------- */
void board_init(void)
{
    /* Same priority grouping the other CH32 projects use: two pre-emption
     * bits, so USB and the tick keep their default levels. */
    PFIC->CFGR = 0x500U;

#if BOARD_HAS_LED
    board_init_led();
#endif
#if BOARD_HAS_LED2
    board_init_led2();
#endif
#if BOARD_HAS_TERMINATION
    board_init_termination();
#endif
#if BOARD_HAS_BOOT_BUTTON
    board_init_boot_button();
#endif

    board_init_systick();
}
