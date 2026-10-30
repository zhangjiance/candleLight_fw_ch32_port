/*
 * port_board.c - the board table candleLight's core expects.
 *
 * One CAN channel (CAN1) with the two status LEDs, and the transceiver
 * termination wired to board_set_termination().
 */
#include <stddef.h>

#include "ch32fun.h"

#include "board.h"          /* candleLight's board table definition */
#include "board_ch32.h"     /* this project's board primitives          */
#include "board_config.h"
#include "config.h"
#include "gs_usb.h"

#ifndef BOARD_LED2_ACTIVE_LOW
#define BOARD_LED2_ACTIVE_LOW BOARD_LED_ACTIVE_LOW
#endif

/*
 * The one place that applies a termination request: drives the 120 ohm switch
 * and mirrors the state on LED2 (lit = termination engaged).
 *
 * Both call paths end up here: gs_usb's GS_USB_BREQ_SET_TERMINATION goes
 * through set_term() in src/can_ch32_adapter.c, while candleLight's board
 * table hook (config.termination_set) goes through board_termination_cb().
 */
void board_termination_set_state(enum gs_can_termination_state state)
{
    /* gs_usb: 0 = off, anything else = on. */
    const bool on = state != (enum gs_can_termination_state)0;

    board_set_termination(on);              /* the switch itself */

#if BOARD_HAS_LED2
    const uint8_t level = BOARD_LED2_ACTIVE_LOW ? (uint8_t)!on : (uint8_t)on;

    funDigitalWrite(BOARD_LED2_PIN, level ? FUN_HIGH : FUN_LOW);
#endif
}

static void board_termination_cb(can_data_t *channel, enum gs_can_termination_state state)
{
    (void)channel;

    board_termination_set_state(state);
}

const struct board_config config = {
    .termination_set = board_termination_cb,
    .channel = {
        {
            .interface = CAN1,
            .leds = {
                /* The status LED is driven by src/status_led.c (off while the
                 * channel is stopped, steady on while it is enabled, dark
                 * pulse per frame), so candleLight's per-channel LED layer is
                 * left unbound here: a NULL port makes led_set() a no-op and
                 * keeps the two drivers from fighting over one pin.
                 * The second LED shows the termination state, see
                 * board_termination_cb() above. */
                [LED_RX] = {
                    .port = NULL,
                    .pin = 0U,
                    .active_high = true,
                },
                [LED_TX] = {
                    .port = NULL,
                    .pin = 0U,
                    .active_high = true,
                },
            },
        },
    },
};

void board_setup(USBD_GS_CAN_HandleTypeDef *hcan)
{
    (void)hcan;
}
