/*
 * board.c - board primitives for boards/ch32l103_candle.
 *
 * Implements board.h on top of the peripheral registers ch32fun declares
 * (ch32l103hw.h): pins and polarity come from board_config.h, the clock tree
 * from src/funconfig.h + board.cmake.  Nothing here goes through a vendor HAL.
 *
 * Bring-up order matters: ch32fun's handle_reset() jumps straight to main()
 * with the system clock still on HSI, so main() calls SystemInit() before
 * board_init().
 */
#include "board_ch32.h"
#include "board_config.h"

#include "ch32fun.h"

/* HCLK cycles per millisecond, fixed by the (static) clock tree. */
#define BOARD_CYCLES_PER_MS  (FUNCONF_SYSTEM_CORE_CLOCK / 1000U)

/* Milliseconds since boot, advanced by the SysTick compare interrupt.  Keeping
 * the time base in a 32 bit counter (instead of dividing the 64 bit SysTick CNT
 * by a non power of two) keeps libgcc's __udivdi3 out of the image: it is
 * ~940 B on this part, a fifth of the firmware. */
static volatile uint32_t ms_ticks;

/* One tick per millisecond.  ch32fun's vector table holds a weak
 * SysTick_Handler, which this strong definition replaces.
 *
 * INTERRUPT_DECORATOR is what makes the compiler emit a trap return: without it
 * the function ends in `ret` and the core jumps back to whatever ra held.
 * Which decorator it expands to depends on FUNCONF_ENABLE_HPE (see
 * src/funconfig.h): with HPE the core stacks the registers in hardware and the
 * handler is a bare `mret`; without it the compiler saves the caller-saved
 * registers itself.  Both return with `mret`, both work on this part. */
void SysTick_Handler(void) INTERRUPT_DECORATOR;
void SysTick_Handler(void)
{
    SysTick->SR = 0;                        /* clear the compare flag */
    SysTick->CMP += BOARD_CYCLES_PER_MS;
    ms_ticks++;
}

/* -------------------------------------------------------------------------- */
/* Time base                                                                  */
/* -------------------------------------------------------------------------- */
/* SysTick counts HCLK (FUNCONF_SYSTICK_USE_HCLK) and interrupts every
 * millisecond; the handler reloads the compare value by adding one period, so
 * the base stays aligned to the counter without ever being restarted. */
static void board_init_systick(void)
{
    SysTick->CTLR = 0;
    SysTick->SR = 0;
    SysTick->CNT = 0;
    SysTick->CMP = BOARD_CYCLES_PER_MS;

    NVIC_EnableIRQ(SysTick_IRQn);
    SysTick->CTLR = 7;                      /* STE | STIE | STCLK (HCLK) */
}

uint32_t board_time_ms(void)
{
    return ms_ticks;
}

void board_delay_ms(uint32_t ms)
{
    const uint32_t start = ms_ticks;

    while ((uint32_t)(ms_ticks - start) < ms) {
    }
}

/* -------------------------------------------------------------------------- */
/* LEDs                                                                       */
/* -------------------------------------------------------------------------- */
static void board_led_pin_init(uint32_t pin)
{
    /* Preload the off level (BOARD_LED_ACTIVE_LOW aware) in the output latch
     * before the pin becomes an output, so the LED does not flash on. */
    const uint8_t off = BOARD_LED_ACTIVE_LOW ? 1U : 0U;

    funDigitalWrite(pin, off ? FUN_HIGH : FUN_LOW);
    funPinMode(pin, GPIO_CFGLR_OUT_10Mhz_PP);
}

static void board_led_pin_write(uint32_t pin, uint8_t on, uint8_t active_low)
{
    const uint8_t level = active_low ? (uint8_t)!on : on;

    funDigitalWrite(pin, level ? FUN_HIGH : FUN_LOW);
}

void board_led1_write(uint8_t on)
{
#if BOARD_HAS_LED1
    board_led_pin_write(BOARD_LED1_PIN, on != 0U, BOARD_LED_ACTIVE_LOW);
#else
    (void)on;
#endif
}

void board_led2_write(uint8_t on)
{
#if BOARD_HAS_LED2
    board_led_pin_write(BOARD_LED2_PIN, on != 0U, BOARD_LED_ACTIVE_LOW);
#else
    (void)on;
#endif
}

void board_led1_toggle(void)
{
#if BOARD_HAS_LED1
    static uint8_t on;

    on ^= 1U;
    board_led1_write(on);
#endif
}

void board_led2_toggle(void)
{
#if BOARD_HAS_LED2
    static uint8_t on;

    on ^= 1U;
    board_led2_write(on);
#endif
}

/* -------------------------------------------------------------------------- */
/* Transceiver termination                                                    */
/* -------------------------------------------------------------------------- */
static bool termination_on;

void board_set_termination(bool on)
{
#if BOARD_HAS_TERMINATION
    const uint8_t level = BOARD_TERM_ACTIVE_LOW ? (on ? 0U : 1U) : (on ? 1U : 0U);

    termination_on = on;
    funDigitalWrite(BOARD_TERM_PIN, level ? FUN_HIGH : FUN_LOW);
#else
    (void)on;
#endif
}

bool board_get_termination(void)
{
    return termination_on;
}

/* -------------------------------------------------------------------------- */
/* Bring-up                                                                   */
/* -------------------------------------------------------------------------- */
void board_init(void)
{
    /* Same priority grouping the other projects use: two pre-emption bits, so
     * USB and the CAN interrupt keep their default levels. */
    PFIC->CFGR = 0x500U;

    funGpioInitAll();

#if BOARD_HAS_LED1
    board_led_pin_init(BOARD_LED1_PIN);
#endif
#if BOARD_HAS_LED2
    board_led_pin_init(BOARD_LED2_PIN);
#endif

#if BOARD_HAS_TERMINATION
    /* Off by default: the host enables it through gs_usb, and an unpowered bus
     * with 120 ohm across it is better than a second terminator on a bus that
     * already has one. */
    funPinMode(BOARD_TERM_PIN, GPIO_CFGLR_OUT_10Mhz_PP);
    board_set_termination(false);
#endif

    board_init_systick();
}
