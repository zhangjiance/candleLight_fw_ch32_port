/*
 * timer.c - candleLight's millisecond timer on the board time base.
 *
 * timer_get() feeds the USB timestamps and the LED update interval, so it has
 * to be the same 1 ms counter the board tick advances.
 */
#include "ch32fun.h"

#include "board_ch32.h"
#include "timer.h"

void timer_init(void)
{
    /* board_init() already started the 1 ms tick. */
}

uint32_t timer_get(void)
{
    return board_time_ms();
}
