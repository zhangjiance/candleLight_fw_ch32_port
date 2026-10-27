/*
 * can_ch32.c - CH32L103 CAN / CANFD controller driver.
 *
 * See can_ch32.h for the framing model.  Everything here is register access on
 * ch32fun's CAN_TypeDef; the WCH CAN library is deliberately not linked.
 */
#include "can_ch32.h"

#include "ch32fun.h"
#include "board_config.h"

/* Payload buffers the controller DMAs through: one 64 byte buffer per transmit
 * mailbox and per receive FIFO.  They must be word aligned and stay alive for
 * as long as the frame is in flight. */
static uint8_t tx_buf[CAN_CH32_NUM_TX_MB][CAN_CH32_MAX_LEN] __attribute__((aligned(4)));
static uint8_t rx_buf[CAN_CH32_NUM_RX_FIFO][CAN_CH32_MAX_LEN] __attribute__((aligned(4)));

static bool fd_enabled;

/* DLC <-> byte count, CAN FD wide. */
static const uint8_t dlc_len[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };

static uint8_t len_to_dlc(uint8_t len)
{
    for (uint8_t d = 0; d < 16U; d++) {
        if (dlc_len[d] >= len) {
            return d;
        }
    }
    return 15U;
}

/* -------------------------------------------------------------------------- */
/* Bit timing                                                                 */
/* -------------------------------------------------------------------------- */
/* bxCAN timing: the bit is BRP * (1 + TS1 + TS2) time quanta of PCLK1.  Pick the
 * smallest prescaler that can still hit the target rate, then place the sample
 * point as close to the requested one as the 16/8 tq limits allow. */
static bool solve_timing(uint32_t pclk1, const can_ch32_timing_t *t,
                         uint32_t *brp, uint32_t *ts1, uint32_t *ts2, uint32_t *sjw)
{
    uint32_t best_brp = 0, best_ts1 = 0, best_ts2 = 0;
    uint32_t best_err = 0xFFFFFFFFU;

    for (uint32_t brp_try = 1; brp_try <= 1024U; brp_try++) {
        const uint32_t tq = pclk1 / (t->bitrate * brp_try);   /* tq per bit */

        if (tq < 8U || tq > 25U) {                            /* 1+16+8 .. sane */
            continue;
        }
        /* Sample point = (1 + TS1) / tq; keep TS1 in 1..16, TS2 in 1..8. */
        uint32_t ts1_try = (tq * t->sample_point) / 1000U;
        if (ts1_try < 1U) {
            ts1_try = 1U;
        }
        if (ts1_try > 16U) {
            ts1_try = 16U;
        }
        if (ts1_try + 1U >= tq) {
            ts1_try = tq - 2U;
        }
        const uint32_t ts2_try = tq - 1U - ts1_try;
        if (ts2_try < 1U || ts2_try > 8U) {
            continue;
        }
        /* How far off the requested sample point are we? */
        const uint32_t want = tq * t->sample_point;
        const uint32_t got = (1U + ts1_try) * 1000U;
        const uint32_t err = (want > got) ? (want - got) : (got - want);

        if (err < best_err) {
            best_err = err;
            best_brp = brp_try;
            best_ts1 = ts1_try;
            best_ts2 = ts2_try;
        }
        if (err == 0U) {
            break;
        }
    }

    if (best_brp == 0U) {
        return false;
    }

    *brp = best_brp;
    *ts1 = best_ts1;
    *ts2 = best_ts2;
    *sjw = (t->sjw == 0U) ? 1U : ((t->sjw > best_ts2) ? best_ts2 : t->sjw);
    return true;
}

/* -------------------------------------------------------------------------- */
/* Init / stop                                                                */
/* -------------------------------------------------------------------------- */
static void can_pins_init(void)
{
    /* TX: alternate function push-pull.  RX: input with pull-up (an unpowered
     * transceiver then reads recessive instead of floating). */
    funDigitalWrite(BOARD_CAN_TX_PIN, FUN_HIGH);
    funPinMode(BOARD_CAN_TX_PIN, GPIO_CFGLR_OUT_50Mhz_AF_PP);
    funDigitalWrite(BOARD_CAN_RX_PIN, FUN_HIGH);
    funPinMode(BOARD_CAN_RX_PIN, GPIO_CFGLR_IN_PUPD);

    /* CAN_RX = PB8, CAN_TX = PB9: CAN_RM[1:0] = 10 (the reset mapping is
     * PA11/PA12, which is what the EVT example uses). */
    RCC->APB2PCENR |= RCC_APB2Periph_AFIO;
    AFIO->PCFR1 = (AFIO->PCFR1 & ~AFIO_PCFR1_CAN_RM) | AFIO_PCFR1_CAN_RM_1;
}

bool can_ch32_init(uint32_t pclk1, const can_ch32_timing_t *nominal,
                   const can_ch32_timing_t *data, bool fd, bool listen_only)
{
    uint32_t brp, ts1, ts2, sjw;

    if (!solve_timing(pclk1, nominal, &brp, &ts1, &ts2, &sjw)) {
        return false;
    }

    fd_enabled = fd;

    RCC->APB1PCENR |= RCC_APB1Periph_CAN1;
    can_pins_init();

    /* Leave sleep, enter initialisation. */
    CAN1->CTLR &= ~(uint32_t)CAN_CTLR_SLEEP;
    CAN1->CTLR |= CAN_CTLR_INRQ;
    for (uint32_t guard = 100000U; guard != 0U; guard--) {
        if ((CAN1->STATR & CAN_STATR_INAK) != 0U) {
            break;
        }
    }

    /* Arbitration phase timing. */
    CAN1->BTIMR = ((brp - 1U) & CAN_BTIMR_BRP) |
                  (((uint32_t)(ts1 - 1U) << 16) & CAN_BTIMR_TS1) |
                  (((uint32_t)(ts2 - 1U) << 20) & CAN_BTIMR_TS2) |
                  (((uint32_t)(sjw - 1U) << 24) & CAN_BTIMR_SJW) |
                  (listen_only ? CAN_BTIMR_SILM : 0U);

    if (fd) {
        uint32_t dbrp, dts1, dts2, dsjw;

        if (data == NULL || !solve_timing(pclk1, data, &dbrp, &dts1, &dts2, &dsjw)) {
            return false;
        }

        /* Data phase timing.  CANFD_BTR is written here rather than by a vendor
         * helper because the EVT's CANFD_Init() copies the arbitration values
         * into it, which would force the data phase to the nominal rate. */
        CAN1->CANFD_BTR = (1UL << 23) |                                  /* TDCE  */
                          (((dbrp - 1U) << 16) & 0x001F0000U) |
                          (((dts1 - 1U) << 8) & 0x00001F00U) |
                          (((dts2 - 1U) << 4) & 0x000000F0U) |
                          ((dsjw - 1U) & 0x0000000FU);

        /* Transmitter delay compensation: TDCO >= 2 (EVT note), and the filter
         * window from the transceiver loop delay (~120 ns for a TJA1042 class
         * part): TDC_FILTER = t_loop * f_sysclk / 3. */
        CAN1->CANFD_TDCT = (2UL) | ((4UL & 0x3FU) << 8);

        /* Every mailbox may switch to the fast data phase; the transmit path
         * picks a mailbox by the frame's BRS flag. */
        CAN1->CANFD_CR |= CANFD_CR_TX_FD;
        CAN1->CANFD_CR = (CAN1->CANFD_CR & ~CANFD_CR_TX_BRS_B) | (0x0EU & CANFD_CR_TX_BRS_B);
    } else {
        CAN1->CANFD_CR &= ~CANFD_CR_TX_FD;
    }

    /* Accept everything: the gs_usb host configures filtering in software
     * (candleLight's can_common), and the CAN driver here has one channel. */
    CAN1->FCTLR |= CAN_FCTLR_FINIT;
    for (uint32_t bank = 0; bank < 14U; bank++) {
        CAN1->sFilterRegister[bank].FR1 = 0U;
        CAN1->sFilterRegister[bank].FR2 = 0U;
    }
    CAN1->FMCFGR = 0U;                       /* all banks: 32 bit mask mode */
    CAN1->FSCFGR = 0U;
    CAN1->FAFIFOR = 0U;                      /* all to FIFO 0 */
    CAN1->FWR = (uint16_t)CAN_FWR_FACT;      /* activate all banks */
    CAN1->FCTLR &= (uint8_t)~CAN_FCTLR_FINIT;

    /* Receive payload buffers: the controller writes the frames straight into
     * them, so hand the addresses over once. */
    for (uint32_t i = 0; i < CAN_CH32_NUM_RX_FIFO; i++) {
        CAN1->CANFD_DMA_R[i] = (uint32_t)(uintptr_t)rx_buf[i];
    }

    CAN1->INTENR = 0U;                       /* polled for now */

    CAN1->CTLR &= ~(uint32_t)CAN_CTLR_INRQ;
    for (uint32_t guard = 100000U; guard != 0U; guard--) {
        if ((CAN1->STATR & CAN_STATR_INAK) == 0U) {
            break;
        }
    }

    return true;
}

void can_ch32_stop(void)
{
    CAN1->CTLR |= CAN_CTLR_INRQ;
    CAN1->CTLR |= CAN_CTLR_SLEEP;
    RCC->APB1PCENR &= ~RCC_APB1Periph_CAN1;
    fd_enabled = false;
}

/* -------------------------------------------------------------------------- */
/* Transmit                                                                   */
/* -------------------------------------------------------------------------- */
bool can_ch32_send(const can_ch32_frame_t *f)
{
    if (f->len > CAN_CH32_MAX_LEN || (f->fd && !fd_enabled)) {
        return false;
    }

    static const uint32_t tme[CAN_CH32_NUM_TX_MB] = {
        CAN_TSTATR_TME0, (CAN_TSTATR_TME0 << 1), (CAN_TSTATR_TME0 << 2)
    };

    uint32_t mb = CAN_CH32_NUM_TX_MB;

    for (uint32_t i = 0; i < CAN_CH32_NUM_TX_MB; i++) {
        if ((CAN1->TSTATR & tme[i]) != 0U) {
            mb = i;
            break;
        }
    }
    if (mb == CAN_CH32_NUM_TX_MB) {
        return false;                        /* all mailboxes busy */
    }

    /* The controller reads the payload itself, so copy it into the mailbox's own
     * buffer (the caller's buffer may go away). */
    for (uint32_t i = 0; i < f->len; i++) {
        tx_buf[mb][i] = f->data[i];
    }

    uint32_t mir = ((uint32_t)len_to_dlc(f->len) & 0x0FU) |
                   (f->rtr ? (1UL << 1) : 0U) |
                   (f->extended ? (1UL << 2) : 0U);

    if (f->extended) {
        mir |= (f->id & 0x1FFFFFFFU) << 3;
    } else {
        mir |= (f->id & 0x7FFU) << 21;
    }

    CAN1->sTxMailBox[mb].TXMIR   = mir;
    CAN1->sTxMailBox[mb].TXMDTR  = 0U;
    CAN1->sTxMailBox[mb].TXMDLR  = 0U;
    CAN1->CANFD_DMA_T[mb]        = (uint32_t)(uintptr_t)tx_buf[mb];
    CAN1->sTxMailBox[mb].TXMIR  |= (1UL << 0);          /* TXRQ */

    return true;
}

/* -------------------------------------------------------------------------- */
/* Receive                                                                    */
/* -------------------------------------------------------------------------- */
static bool can_read_fifo(uint32_t fifo, can_ch32_frame_t *f)
{
    volatile uint32_t *rfifo = (fifo == 0U) ? &CAN1->RFIFO0 : &CAN1->RFIFO1;

    if ((*rfifo & CAN_RFIFO0_FMP0) == 0U) {
        return false;
    }

    const uint32_t mir = CAN1->sFIFOMailBox[fifo].RXMIR;
    const uint32_t dlc = mir & 0x0FU;

    f->extended = ((mir >> 2) & 1U) != 0U;
    f->rtr      = ((mir >> 1) & 1U) != 0U;
    f->id       = f->extended ? ((mir >> 3) & 0x1FFFFFFFU) : ((mir >> 21) & 0x7FFU);
    f->len      = dlc_len[dlc];
    /* RXMIR bit0 = FDF (1 = FD frame), bit1 = RTR, bit2 = IDE (CH32L103 RM
     * 22.8.23).  The flag is only meaningful when the controller runs in FD
     * mode; a classic frame always reads 0 there. */
    f->fd       = ((mir & 0x1U) != 0U);

    for (uint32_t i = 0; i < f->len && i < CAN_CH32_MAX_LEN; i++) {
        f->data[i] = rx_buf[fifo][i];
    }

    /* Release the FIFO output mailbox. */
    *rfifo |= CAN_RFIFO0_RFOM0;
    return true;
}

bool can_ch32_receive(can_ch32_frame_t *f)
{
    for (uint32_t fifo = 0; fifo < CAN_CH32_NUM_RX_FIFO; fifo++) {
        if (can_read_fifo(fifo, f)) {
            return true;
        }
    }
    return false;
}

void can_ch32_irq_handler(void)
{
    can_ch32_frame_t f;

    while (can_ch32_receive(&f)) {
        /* The gs_usb layer polls; nothing to do here yet. */
    }
}

/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */
void can_ch32_get_state(can_ch32_state_t *st)
{
    const uint32_t esr = CAN1->ERRSR;

    st->tx_error_count = (esr >> 16) & 0xFFU;
    st->rx_error_count = (esr >> 24) & 0xFFU;
    st->warning        = (esr & 0x1U) != 0U;
    st->error_passive  = (esr & 0x2U) != 0U;
    st->bus_off        = (esr & 0x4U) != 0U;
}
