/*
 * can_ch32_adapter.c
 *
 * candleLight's CAN driver contract (can.h + can_drv.h) on top of the
 * CH32L103 bxCAN/CANFD driver in src/can_ch32.c.
 *
 * The shared firmware above this layer (can_common.c, usbd_gs_can.c) is
 * unchanged: it hands gs_host_frame structures down here and expects the
 * bit timing it programmed to reach the controller.
 */
#include <string.h>

#include "config.h"
#include "can.h"
#include "can_common.h"      /* can_channel_get_nr() */
#include "can_ch32.h"
#include "board.h"
#include "board_ch32.h"
#include "can_drv.h"
#include "gs_usb.h"
#include "hal_include.h"
#include "led.h"
#include "status_led.h"

/* gs_usb bit timing (segments in time quanta) -> our driver's form. */
static void bt_from_gs(const struct gs_device_bittiming *bt, can_ch32_timing_t *out)
{
    const uint32_t tq  = 1U + bt->prop_seg + bt->phase_seg1 + bt->phase_seg2;
    const uint32_t brp = (bt->brp == 0U) ? 1U : bt->brp;

    out->bitrate      = (uint32_t)(CAN_CLOCK_SPEED / (brp * tq));
    out->sample_point = (uint16_t)(((1U + bt->prop_seg + bt->phase_seg1) * 1000U) / tq);
    out->sjw          = (uint16_t)(bt->sjw == 0U ? 1U : bt->sjw);
}

/* -------------------------------------------------------------------------- */
/* can.h                                                                      */
/* -------------------------------------------------------------------------- */

void can_init(can_data_t *channel, const struct board_channel_config *config)
{
    memset(channel, 0, sizeof(*channel));

    /* can_channel owns state that is already live when can_init() is called:
     * the firmware initialises list_from_host before handing the channel over,
     * and can_disable() - called right after this - walks that list to purge
     * pending frames.  memset() would leave next/prev at NULL and the purge
     * would then follow garbage, so restore the head immediately. */
    INIT_LIST_HEAD(&channel->list_from_host);

#if defined(CONFIG_BXCAN)
    channel->instance = config->interface;
#else
    channel->instance = CAN1;
#endif

    channel->state   = GS_CAN_STATE_STOPPED;
    channel->feature = 0U;

    led_init(&channel->leds,
             config->leds[LED_RX].port, config->leds[LED_RX].pin, config->leds[LED_RX].active_high,
             config->leds[LED_TX].port, config->leds[LED_TX].pin, config->leds[LED_TX].active_high);
}

/*
 * One frame of lookahead: can_is_rx_pending() has to answer without consuming
 * the frame (it used to pull one out of the controller and drop it), so
 * whatever it takes is handed over to the next can_receive() call.
 */
static can_ch32_frame_t g_rx_next;
static bool g_rx_next_valid;

static bool adapter_rx_peek(void)
{
    if (g_rx_next_valid) {
        return true;
    }

    g_rx_next_valid = can_ch32_receive(&g_rx_next);

    return g_rx_next_valid;
}

static bool adapter_rx_pop(can_ch32_frame_t *f)
{
    if (adapter_rx_peek()) {
        *f = g_rx_next;
        g_rx_next_valid = false;
        return true;
    }

    return false;
}

bool can_is_rx_pending(can_data_t *channel)
{
    (void)channel;

    return adapter_rx_peek();
}

bool can_receive(can_data_t *channel, struct gs_host_frame *rx_frame)
{
    can_ch32_frame_t f;

    (void)channel;

    if (!adapter_rx_pop(&f)) {
        return false;
    }

    rx_frame->echo_id = 0xFFFFFFFFU;
    rx_frame->can_id  = f.id | (f.extended ? 0x80000000U : 0U) | (f.rtr ? 0x40000000U : 0U);
    rx_frame->can_dlc = f.len;
    rx_frame->channel = 0U;
    rx_frame->flags   = (uint8_t)((f.fd ? GS_CAN_FLAG_FD : 0U) |
                                  (f.brs ? GS_CAN_FLAG_BRS : 0U) |
                                  (f.esi ? GS_CAN_FLAG_ESI : 0U));
    rx_frame->reserved = 0U;
    memcpy(rx_frame->classic_can->data, f.data, f.len > 8U ? 8U : f.len);

    if (f.len > 8U) {
        memcpy(rx_frame->canfd->data, f.data, f.len);
    }

    led_indicate_trx(&channel->leds, LED_RX);
    status_led_activity(can_channel_get_nr(channel), true);
    return true;
}

bool can_send(can_data_t *channel, struct gs_host_frame *frame)
{
    can_ch32_frame_t f;

    memset(&f, 0, sizeof(f));
    f.extended = (frame->can_id & 0x80000000U) != 0U;
    f.rtr      = (frame->can_id & 0x40000000U) != 0U;
    f.id       = frame->can_id & (f.extended ? 0x1FFFFFFFU : 0x7FFU);
    f.fd       = (frame->flags & GS_CAN_FLAG_FD) != 0U;
    f.brs      = (frame->flags & GS_CAN_FLAG_BRS) != 0U;
    f.len      = (uint8_t)(frame->can_dlc > CAN_CH32_MAX_LEN ? CAN_CH32_MAX_LEN : frame->can_dlc);
    memcpy(f.data, frame->canfd->data, f.len);

    if (!can_ch32_send(&f)) {
        return false;
    }

    led_indicate_trx(&channel->leds, LED_TX);
    status_led_activity(can_channel_get_nr(channel), false);
    return true;
}

/* -------------------------------------------------------------------------- */
/* can_drv.h                                                                  */
/* -------------------------------------------------------------------------- */

/*
 * The controller itself is brought up from the main loop (can_drv_poll()), not
 * from here: can_drv_enable() runs inside the USB interrupt (the gs_usb class
 * handles GS_USB_BREQ_MODE there), and anything slow or blocking in that
 * context stalls EP0/EP2 handling - the host then reports "bulk TX timeout on
 * EP 0x2" even though the frame never reached us.
 */
static volatile bool s_start_pending;

void can_drv_enable(can_data_t *channel)
{
    channel->state = GS_CAN_STATE_ERROR_ACTIVE;
    led_set_mode(&channel->leds, LED_MODE_NORMAL);
    s_start_pending = true;
}

/* Called from the main loop: performs the deferred controller start. */
void can_drv_poll(can_data_t *channel)
{
    can_ch32_timing_t nom, dat;

    if (!s_start_pending) {
        return;
    }
    s_start_pending = false;

    /* Controller mode comes straight from the gs_usb MODE_START request (Linux:
     * `ip link set can0 type can ... loopback on` / `listen-only on`). */
    const bool listen_only = (channel->feature & GS_CAN_FEATURE_LISTEN_ONLY) != 0U;
    const bool loopback    = (channel->feature & GS_CAN_FEATURE_LOOP_BACK) != 0U;

    bt_from_gs(&channel->bittiming, &nom);
#if defined(CONFIG_CANFD)
    bt_from_gs(&channel->data_bittiming, &dat);
#else
    dat = nom;
#endif

    if (!can_ch32_init(CAN_CLOCK_SPEED, &nom, &dat, true, listen_only, loopback)) {
        channel->state = GS_CAN_STATE_ERROR_PASSIVE;
        return;
    }

    channel->state = GS_CAN_STATE_ERROR_ACTIVE;
    led_set_mode(&channel->leds, LED_MODE_NORMAL);
    status_led_channel(can_channel_get_nr(channel), true);
}

void can_drv_disable(can_data_t *channel)
{
    s_start_pending = false;
    g_rx_next_valid = false;

    can_ch32_stop();

    channel->state = GS_CAN_STATE_STOPPED;
    led_set_mode(&channel->leds, LED_MODE_OFF);
    status_led_channel(can_channel_get_nr(channel), false);
}

void can_drv_read_reg_status(can_data_t *channel)
{
    can_ch32_state_t st;

    can_ch32_get_state(&st);
    channel->reg_status.esr = (st.tx_error_count << 16) | (st.rx_error_count << 24) |
                              (st.warning ? 1U : 0U) | (st.error_passive ? 2U : 0U) |
                              (st.bus_off ? 4U : 0U);
}

bool can_drv_bus_error_pending(const can_data_t *channel)
{
    (void)channel;

    return false;           /* no error frame reporting yet */
}

bool can_drv_handle_bus_error(const can_data_t *channel, struct gs_host_frame *frame)
{
    (void)channel;
    (void)frame;

    return false;
}

enum gs_can_state can_drv_get_state(const can_data_t *channel)
{
    can_ch32_state_t st;

    (void)channel;

    can_ch32_get_state(&st);

    if (st.bus_off) {
        return GS_CAN_STATE_BUS_OFF;
    }
    if (st.error_passive) {
        return GS_CAN_STATE_ERROR_PASSIVE;
    }
    if (st.warning) {
        return GS_CAN_STATE_ERROR_WARNING;
    }

    return GS_CAN_STATE_ERROR_ACTIVE;
}

void can_drv_get_device_state(const can_data_t *channel, struct gs_device_state *state)
{
    can_ch32_state_t st;

    (void)channel;

    can_ch32_get_state(&st);
    state->state = (uint32_t)can_drv_get_state(channel);
    state->rxerr = st.rx_error_count;
    state->txerr = st.tx_error_count;
}

void can_drv_handle_state_change(const can_data_t *channel, struct gs_host_frame *frame)
{
    (void)channel;
    (void)frame;
}

void can_drv_handle_bus_off_recovery(can_data_t *channel)
{
    (void)channel;

    /* bxCAN-style recovery: re-enter initialisation and leave it again.  That is
     * what clears the bus-off state when ABOM is off, which is candleLight's
     * model (the host asks for the restart). */
    RCC->APB1PCENR |= RCC_APB1Periph_CAN1;

    CAN1->CTLR |= CAN_CTLR_INRQ;
    for (uint32_t guard = 100000U; guard != 0U; guard--) {
        if ((CAN1->STATR & CAN_STATR_INAK) != 0U) {
            break;
        }
    }
    CAN1->CTLR &= ~(uint32_t)CAN_CTLR_INRQ;
    for (uint32_t guard = 100000U; guard != 0U; guard--) {
        if ((CAN1->STATR & CAN_STATR_INAK) == 0U) {
            break;
        }
    }
}

void can_drv_get_device_tdc(const can_data_t *channel, struct gs_device_tdc *tdc)
{
    (void)channel;

    tdc->tdco = 0U;
    tdc->tdcf = 0U;
}

/* -------------------------------------------------------------------------- */
/* Capability tables the gs_usb layer advertises                              */
/* -------------------------------------------------------------------------- */
#define GS_FEATURES (GS_CAN_FEATURE_LISTEN_ONLY | \
                     GS_CAN_FEATURE_LOOP_BACK |   \
                     GS_CAN_FEATURE_HW_TIMESTAMP |\
                     GS_CAN_FEATURE_IDENTIFY |    \
                     GS_CAN_FEATURE_FD |          \
                     GS_CAN_FEATURE_BT_CONST_EXT |\
                     GS_CAN_FEATURE_TERMINATION | \
                     GS_CAN_FEATURE_GET_STATE)

const struct gs_device_bt_const CAN_btconst = {
    .feature  = GS_FEATURES,
    .fclk_can = CAN_CLOCK_SPEED,
    .btc = {
        .tseg1_min = 1, .tseg1_max = 16,
        .tseg2_min = 1, .tseg2_max = 8,
        .sjw_max   = 4,
        .brp_min   = 1, .brp_max = 1024, .brp_inc = 1,
    },
};

const struct gs_device_bt_const_extended CAN_btconst_ext = {
    .feature  = GS_FEATURES,
    .fclk_can = CAN_CLOCK_SPEED,
    .btc = {
        .tseg1_min = 1, .tseg1_max = 16,
        .tseg2_min = 1, .tseg2_max = 8,
        .sjw_max   = 4,
        .brp_min   = 1, .brp_max = 1024, .brp_inc = 1,
    },
    .dbtc = {
        .tseg1_min = 1, .tseg1_max = 16,
        .tseg2_min = 1, .tseg2_max = 8,
        .sjw_max   = 4,
        .brp_min   = 1, .brp_max = 32, .brp_inc = 1,
    },
};

const struct gs_device_tdc_const CAN_tdc_const = {
    .tdco_min = 0,
    .tdco_max = 63,
    .tdcf_min = 0,
    .tdcf_max = 63,
    .mode     = GS_CAN_TDC_MODE_OFF | GS_CAN_TDC_MODE_AUTO,
};

/* src/port_board.c: applies the 120 ohm switch and its indicator LED. */
void board_termination_set_state(enum gs_can_termination_state state);

enum gs_can_termination_state get_term(can_data_t *channel)
{
    (void)channel;

    return (enum gs_can_termination_state)(board_get_termination() ? 1U : 0U);
}

void set_term(can_data_t *channel, enum gs_can_termination_state state)
{
    (void)channel;

    /* This is the path gs_usb takes (GS_USB_BREQ_SET_TERMINATION). */
    board_termination_set_state(state);
}
