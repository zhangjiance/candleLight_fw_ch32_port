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

static void board_termination_cb(can_data_t *channel, enum gs_can_termination_state state)
{
    (void)channel;

    /* gs_usb: 0 = off, anything else = on. */
    board_set_termination(state != (enum gs_can_termination_state)0);
}

const struct board_config config = {
    .termination_set = board_termination_cb,
    .channel = {
        {
            .interface = CAN1,
            .leds = {
                [LED_RX] = { .port = GPIOA, .pin = 0U, .active_high = false },  /* LED1 = PA0 */
                [LED_TX] = { .port = GPIOA, .pin = 1U, .active_high = false },  /* LED2 = PA1 */
            },
        },
    },
};

void board_setup(USBD_GS_CAN_HandleTypeDef *hcan)
{
    (void)hcan;
}
