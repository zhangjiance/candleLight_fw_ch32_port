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

/*
 * The controller's receive FIFO holds only one FD frame.  The CAN interrupt
 * moves frames into this ring immediately; the main loop then drains it into
 * the USB frame pool.  Without the ring a slow USB iteration overwrites the
 * next frame in hardware.
 *
 * Single producer (IRQ) / single consumer (main loop), so head/tail are just
 * volatile indices; the ring size must stay a power of two.
 */
#define CAN_CH32_RX_RING 32U
static can_ch32_frame_t rx_ring[CAN_CH32_RX_RING];
static volatile uint32_t rx_ring_head;
static volatile uint32_t rx_ring_tail;

_Static_assert((CAN_CH32_RX_RING & (CAN_CH32_RX_RING - 1U)) == 0U,
               "CAN_CH32_RX_RING must be a power of two");

static bool fd_enabled;

/* Programmed transmitter delay offset (CANFD_TDCT.TDCO), needed to turn
 * CANFD_PSR.TDCV into the net measured delay. */
static uint8_t s_tdc_offset;

/* Minimum delay of the internal delay counter: rejects dominant glitches
 * on RX (~83 ns at 48 MHz).  Overridden by a host SET_TDC. */
#define CAN_CH32_DEFAULT_TDC_FILTER 4U

/* Frames lost to a FIFO overrun since start-up (reported by the adapter). */
static uint32_t s_rx_overruns;
static uint32_t s_rx_overruns_reported;

uint32_t can_ch32_rx_overruns(void)
{
    return s_rx_overruns;
}

uint32_t can_ch32_tdcv(void)
{
    const uint32_t measured = (CAN1->CANFD_PSR >> 16) & 0xFFU;

    /* Nothing measured yet, or the offset is all there is so far. */
    if (measured <= s_tdc_offset) {
        return 0U;
    }

    return measured - s_tdc_offset;
}

bool can_ch32_rx_overrun_pending(void)
{
    return s_rx_overruns != s_rx_overruns_reported;
}

bool can_ch32_take_rx_overrun(void)
{
    if (s_rx_overruns == s_rx_overruns_reported) {
        return false;
    }

    s_rx_overruns_reported = s_rx_overruns;
    return true;
}

/* CAN_ERRSR.LEC: 0 = no error, 7 = "set by software" (latch armed). */
static bool errsr_lec_is_error(uint8_t lec)
{
    return (lec != 0U) && (lec != 7U);
}

bool can_ch32_error_pending(void)
{
    RCC->APB1PCENR |= RCC_APB1Periph_CAN1;

    return errsr_lec_is_error((uint8_t)((CAN1->ERRSR >> 4) & 0x7U));
}

bool can_ch32_take_error(can_ch32_error_t *e)
{
    RCC->APB1PCENR |= RCC_APB1Periph_CAN1;

    const uint32_t esr = CAN1->ERRSR;
    const uint8_t lec = (uint8_t)((esr >> 4) & 0x7U);

    if (!errsr_lec_is_error(lec)) {
        return false;
    }

    e->lec           = lec;
    e->tec           = (uint8_t)((esr >> 16) & 0xFFU);
    e->rec           = (uint8_t)((esr >> 24) & 0xFFU);
    e->warning       = (esr & 0x1U) != 0U;
    e->error_passive = (esr & 0x2U) != 0U;
    e->bus_off       = (esr & 0x4U) != 0U;

    /* Re-arm the latch: bxCAN clears LEC by writing 7 ("set by software").
     * The other bits of the register are read-only, so the plain write is
     * safe. */
    CAN1->ERRSR = 0x70U;

    return true;
}

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
                         uint32_t brp_max,
                         uint32_t *brp, uint32_t *ts1, uint32_t *ts2, uint32_t *sjw)
{
    uint32_t best_brp = 0, best_ts1 = 0, best_ts2 = 0;
    uint32_t best_err = 0xFFFFFFFFU;

    if (t->bitrate == 0U) {
        return false;                     /* host sent no bittiming yet */
    }

    for (uint32_t brp_try = 1; brp_try <= brp_max; brp_try++) {
        const uint32_t denom = t->bitrate * brp_try;

        /* The prescaler has to divide the clock exactly, otherwise the real
         * bit rate differs from the requested one (e.g. 527 kbit/s for a
         * 500 kbit/s request) - loopback would still work, a real bus would
         * not. */
        if ((pclk1 % denom) != 0U) {
            continue;
        }

        const uint32_t tq = pclk1 / denom;                    /* tq per bit */

        if (tq < 8U || tq > 25U) {                            /* 1+16+8 .. sane */
            continue;
        }
        /*
         * Sample point = (1 + TS1) / tq with TS1 in 1..16 and TS2 in 1..8.
         *
         * The rounded value tq*sp/1000 is only one candidate: at tq = 12 a
         * 75% request rounds to TS1 = 9 (83.3%) while TS1 = 8 gives exactly
         * 75%.  Both neighbours are therefore evaluated and the closer one
         * wins.
         */
        const uint32_t want = tq * t->sample_point;
        const uint32_t ts1_base = want / 1000U;
        uint32_t err = 0xFFFFFFFFU;
        uint32_t ts1_best = 0U;

        for (uint32_t step = 0U; step < 2U; step++) {
            int32_t ts1_try = (int32_t)ts1_base - (int32_t)step;

            if (ts1_try < 1) {
                continue;
            }
            if (ts1_try > 16) {
                ts1_try = 16;
            }

            const int32_t ts2_try = (int32_t)tq - 1 - ts1_try;

            if ((ts2_try < 1) || (ts2_try > 8)) {
                continue;
            }

            const uint32_t got = (uint32_t)(1 + ts1_try) * 1000U;
            const uint32_t try_err = (want > got) ? (want - got) : (got - want);

            if (try_err < err) {
                err = try_err;
                ts1_best = (uint32_t)ts1_try;
            }
        }

        if (err == 0xFFFFFFFFU) {
            continue;
        }

        if (err < best_err) {
            best_err = err;
            best_brp = brp_try;
            best_ts1 = ts1_best;
            best_ts2 = tq - 1U - ts1_best;
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

#ifndef BOARD_CAN_RM
/*
 * CAN_RM[1:0] pin mapping (AFIO->PCFR1 bits 14:13), from the CH32L103
 * datasheet table "CAN1 复用功能重映射":
 *
 *   CAN_RM = 00 : TX = PA12, RX = PA11      (USB D+/D- pins - never use)
 *   CAN_RM = 10 : TX = PB9,  RX = PB8       (this board)
 *   CAN_RM = 11 : TX = PD1,  RX = PD0
 *
 * 01 is not defined.  A board on another pin pair overrides BOARD_CAN_RM.
 */
#define BOARD_CAN_RM 2U
#endif

    RCC->APB2PCENR |= RCC_APB2Periph_AFIO;
    AFIO->PCFR1 = (AFIO->PCFR1 & ~AFIO_PCFR1_CAN_RM) |
                  ((BOARD_CAN_RM << 13) & AFIO_PCFR1_CAN_RM);
}

static bool can_wait_inak(bool want)
{
    for (uint32_t guard = 200000U; guard != 0U; guard--) {
        const bool inak = (CAN1->STATR & CAN_STATR_INAK) != 0U;

        if (inak == want) {
            return true;
        }
    }
    return false;
}

/*
 * Resolve one phase's timing.
 *
 * The host already computed prop_seg/phase_seg1/phase_seg2/brp from the
 * constants this port advertises (bt_const / bt_const_ext), so those numbers
 * are used as they are - re-deriving them here used to pick a different
 * sample point than the one the host logged and validated (e.g. 75% instead
 * of the 81.25% the kernel chose for "sample-point 0.8" at 500 kbit/s on a
 * 48 MHz CAN clock).  solve_timing() is only the fallback for missing or
 * out-of-range input.
 */
static bool timing_resolve(const can_ch32_timing_t *t, uint32_t pclk1,
                           uint32_t brp_max,
                           uint32_t *brp, uint32_t *ts1, uint32_t *ts2,
                           uint32_t *sjw)
{
    if ((t->brp >= 1U) && (t->brp <= brp_max) &&
        (t->ts1 >= 1U) && (t->ts1 <= 16U) &&
        (t->ts2 >= 1U) && (t->ts2 <= 8U) &&
        (t->sjw >= 1U) && (t->sjw <= 4U)) {
        *brp = t->brp;
        *ts1 = t->ts1;
        *ts2 = t->ts2;
        *sjw = (t->sjw > t->ts2) ? t->ts2 : t->sjw;

        return true;
    }

    return solve_timing(pclk1, t, brp_max, brp, ts1, ts2, sjw);
}

/* Store what was really programmed, so the host can read it back. */
static void timing_store(can_ch32_timing_t *t, uint32_t brp, uint32_t ts1,
                         uint32_t ts2, uint32_t sjw)
{
    const uint32_t tq = 1U + ts1 + ts2;

    t->brp          = (uint16_t)brp;
    t->ts1          = (uint16_t)ts1;
    t->ts2          = (uint16_t)ts2;
    t->sjw          = (uint16_t)sjw;
    t->sample_point = (uint16_t)(((1U + ts1) * 1000U) / tq);
}


bool can_ch32_init(uint32_t pclk1, can_ch32_timing_t *nominal,
                   can_ch32_timing_t *data, bool fd,
                   bool listen_only, bool loopback,
                   bool tdc_off, bool tdc_set, uint8_t tdco, uint8_t tdcf)
{
    uint32_t brp, ts1, ts2, sjw;

    if (!timing_resolve(nominal, pclk1, 1024U, &brp, &ts1, &ts2, &sjw)) {
        return false;
    }

    timing_store(nominal, brp, ts1, ts2, sjw);

    fd_enabled = fd;
    s_rx_overruns = 0U;
    s_rx_overruns_reported = 0U;
    rx_ring_head = 0U;
    rx_ring_tail = 0U;

    RCC->APB1PCENR |= RCC_APB1Periph_CAN1;

    /* A restart must not race the RX interrupt while the cell is reset. */
    CAN1->INTENR = 0U;
    NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);

    /*
     * Start-up sequence of the vendor driver (EVT ch32l103_can.c CAN_Init()).
     * The absolute-address block below is not decoration: it is what actually
     * brings the CANFD cell's protocol engine up.  Skipping it leaves a cell
     * that answers register reads (BTIMR loads, ERRSR is 0) but never completes
     * a single transmission - every frame times out and the TEC runs into
     * bus-off, even in internal loopback.
     */

    /* RES_Exception, classical frame format. */
    CAN1->CANFD_CR |= (1UL << 7);
    CAN1->CANFD_CR &= ~CANFD_CR_TX_FD;

    /* Reset the cell (APB1), exactly as the vendor does at the top. */
    RCC->APB1PRSTR |= RCC_APB1Periph_CAN1;
    RCC->APB1PRSTR &= ~RCC_APB1Periph_CAN1;

    can_pins_init();

    /* Phase 1: leave sleep, enter initialisation, load a *valid* placeholder
     * BTIMR (the cell needs valid timing before it can leave initialisation),
     * then leave initialisation again. */
    CAN1->CTLR &= ~(uint32_t)CAN_CTLR_SLEEP;
    CAN1->CTLR |= CAN_CTLR_INRQ;
    if (!can_wait_inak(true)) {
        return false;
    }
    CAN1->BTIMR = 0xC1100000UL | ((pclk1 / 4000000U) - 1U);
    CAN1->CTLR &= ~(uint32_t)CAN_CTLR_INRQ;
    if (!can_wait_inak(false)) {
        return false;
    }

    /* Vendor's cell-enable block (absolute addresses in the original). */
    CAN1->FAFIFOR |= 1U;
    CAN1->sFilterRegister[0].FR1 = 0U;
    CAN1->sFilterRegister[0].FR2 = 0U;
    CAN1->FWR |= 1U;
    CAN1->FCTLR &= ~(uint32_t)CAN_FCTLR_FINIT;
    CAN1->sTxMailBox[0].TXMIR |= 0x3U;             /* TXRQ|RTR, as shipped */
    for (uint32_t guard = 100000U; guard != 0U; guard--) {
        if ((CAN1->RFIFO0 & 0x3U) != 0U) {
            break;
        }
    }
    CAN1->RFIFO0 = 0x38U;

    RCC->APB1PRSTR |= RCC_APB1Periph_CAN1;
    RCC->APB1PRSTR &= ~RCC_APB1Periph_CAN1;

    CAN1->FCTLR |= CAN_FCTLR_FINIT;
    CAN1->FAFIFOR |= 1U;
    CAN1->FWR |= 1U;
    CAN1->FCTLR &= ~(uint32_t)CAN_FCTLR_FINIT;

    CAN1->CANFD_CR |= (1UL << 7);
    CAN1->CANFD_CR &= ~CANFD_CR_TX_FD;

    /* Phase 2: initialise with the real timing and the requested mode. */
    CAN1->CTLR &= ~(uint32_t)CAN_CTLR_SLEEP;
    CAN1->CTLR |= CAN_CTLR_INRQ;
    if (!can_wait_inak(true)) {
        return false;
    }

    const uint32_t btimr = ((brp - 1U) & CAN_BTIMR_BRP) |
                           (((uint32_t)(ts1 - 1U) << 16) & CAN_BTIMR_TS1) |
                           (((uint32_t)(ts2 - 1U) << 20) & CAN_BTIMR_TS2) |
                           (((uint32_t)(sjw - 1U) << 24) & CAN_BTIMR_SJW) |
                           (listen_only ? CAN_BTIMR_SILM : 0U) |     /* silent       */
                           (loopback ? CAN_BTIMR_LBKM : 0U);         /* self-receive */
    CAN1->BTIMR = btimr;

    if (fd) {
        uint32_t dbrp, dts1, dts2, dsjw;

        if ((data == NULL) ||
            !timing_resolve(data, pclk1, 32U, &dbrp, &dts1, &dts2, &dsjw)) {
            return false;
        }

        timing_store(data, dbrp, dts1, dts2, dsjw);

        /*
         * Transmitter delay compensation can only be enabled while the data
         * phase prescaler is 1 or 2: the SSP counter runs on the CAN clock,
         * so BTR_BRP_FD has to stay 0 or 1 (CH32L103 CANFD example / ISO
         * 11898-1).  This limits the timing, it does not justify changing it:
         *
         *   1 Mbit/s at 48 MHz with a 75% sample point needs DBRP = 3
         *   (16 tq: TS1 11 / TS2 4).  Forcing DBRP <= 2 would only leave
         *   24 tq (TS1 16 / TS2 7), i.e. 70.8% - not 75%, so a peer that
         *   really runs 75% at 1 Mbit/s is no longer matched.
         *
         * 2 Mbit/s and above do fit DBRP <= 2 (12 tq at 2/4 Mbit/s), so TDC
         * stays on there.  Below 2 Mbit/s the loop delay is small enough
         * that TDC is not needed, and the default TDCO (derived from TS1) is
         * not scaled for the longer bit time either - so keep TDCE = 0 and
         * program the host's exact timing.
         *
         * TDC is a transmitter-local mechanism: it does not have to match
         * the peer (PCAN, another candleLight, ...), only the bitrate and
         * sample point do.  The hard rule is the DBRP <= 2 limit above.
         *
         * A host SET_TDC with mode OFF disables TDC unconditionally.
         */
        const bool tdce = !tdc_off &&
                          (data->bitrate > nominal->bitrate) &&
                          (data->bitrate >= 2000000U) &&
                          (dbrp <= 2U);

        /*
         * Transmitter delay compensation: the cell measures the TX->RX loop
         * delay itself (CANFD_PSR.TDCV = measured + FD_TDCO); TDCT only holds
         * the minimum-delay filter and the offset added on top.
         *
         * Defaults: offset = the data-phase SEG1 (TS1) in time quanta, which
         * is the usual rule of thumb, plus a small filter that only rejects
         * dominant glitches on RX.  A host SET_TDC overrides both.
         */
        const uint32_t tdco_val = (tdc_set ? tdco : (uint8_t)(dts1 - 1U)) & 0x3FU;
        const uint32_t tdcf_val = (tdc_set ? tdcf : CAN_CH32_DEFAULT_TDC_FILTER) & 0x3FU;

        s_tdc_offset = tdce ? (uint8_t)tdco_val : 0U;

        CAN1->CANFD_BTR = (tdce ? (1UL << 23) : 0UL) |                  /* TDCE */
                          (((dbrp - 1U) << 16) & 0x001F0000U) |
                          (((dts1 - 1U) << 8) & 0x00001F00U) |
                          (((dts2 - 1U) << 4) & 0x000000F0U) |
                          ((dsjw - 1U) & 0x0000000FU);

        CAN1->CANFD_TDCT = (tdcf_val << 8) | tdco_val;
        CAN1->CANFD_CR &= ~(CANFD_CR_TX_BRS_B | CANFD_CR_USER_ESI_B);
    } else {
        s_tdc_offset = 0U;               /* no data phase, no TDC */
    }

    /* Accept everything: candleLight filters in software, one channel here. */
    CAN1->FCTLR |= CAN_FCTLR_FINIT;
    for (uint32_t bank = 0; bank < 14U; bank++) {
        CAN1->sFilterRegister[bank].FR1 = 0U;
        CAN1->sFilterRegister[bank].FR2 = 0U;
    }
    CAN1->FMCFGR = 0U;
    CAN1->FSCFGR = 0U;
    CAN1->FAFIFOR = 0U;
    CAN1->FWR = (uint16_t)CAN_FWR_FACT;
    CAN1->FCTLR &= (uint8_t)~CAN_FCTLR_FINIT;

    /* Receive payload buffer for the (single) FIFO. */
    CAN1->CANFD_DMA_R[0] = (uint32_t)(uintptr_t)rx_buf[0];

    CAN1->CTLR &= ~(uint32_t)CAN_CTLR_INRQ;
    if (!can_wait_inak(false)) {
        return false;
    }

    /*
     * Let the CAN interrupt empty the one-deep hardware FIFO into rx_ring.
     * FOVIE0 is only for counting/reporting an overrun; the ring-full case is
     * counted in can_ch32_irq_handler().
     */
    CAN1->INTENR = CAN_INTENR_FMPIE0 | CAN_INTENR_FOVIE0;
    NVIC_ClearPendingIRQ(USB_LP_CAN1_RX0_IRQn);
    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0U);
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    return true;
}

void can_ch32_stop(void)
{
    /* Keep the APB1 clock running: the host polls the controller state
     * (GS_USB_BREQ_GET_STATE / DEVICE_STATE) even while the channel is
     * stopped, and touching the cell with its clock gated can stall the
     * core - which in turn stalls the USB interrupt and makes every bulk
     * OUT transfer on EP 0x02 time out. */
    RCC->APB1PCENR |= RCC_APB1Periph_CAN1;

    CAN1->INTENR = 0U;
    NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    rx_ring_head = 0U;
    rx_ring_tail = 0U;

    CAN1->CTLR |= CAN_CTLR_INRQ;
    CAN1->CTLR |= CAN_CTLR_SLEEP;
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

    /* Identifier register: TXRQ/RTR/IDE + identifier only.  The DLC lives in
     * TXMDTR (bxCAN TDTR[3:0]); writing it here as well used to make the
     * controller send every classic frame with DLC 0 and no payload. */
    uint32_t mir = (f->rtr ? (1UL << 1) : 0U) |
                   (f->extended ? (1UL << 2) : 0U);

    if (f->extended) {
        mir |= (f->id & 0x1FFFFFFFU) << 3;
    } else {
        mir |= (f->id & 0x7FFU) << 21;
    }

    uint32_t lo = 0U;
    uint32_t hi = 0U;

    for (uint32_t i = 0U; (i < 4U) && (i < f->len); i++) {
        lo |= (uint32_t)f->data[i] << (8U * i);
    }
    for (uint32_t i = 4U; (i < 8U) && (i < f->len); i++) {
        hi |= (uint32_t)f->data[i] << (8U * (i - 4U));
    }

    CAN1->sTxMailBox[mb].TXMIR   = mir;
    CAN1->sTxMailBox[mb].TXMDTR  = (uint32_t)len_to_dlc(f->len) & 0x0FU;   /* DLC */

    if (f->fd) {
        /* FD frame: CANFD_CR.TX_FD selects the FD format and the payload is
         * fetched from this mailbox's DMA buffer (vendor CANFD_Transmit()).
         * BRS/ESI are the mailbox's own bits. */
        const uint32_t brs_bit = 1UL << (1U + mb);
        const uint32_t esi_bit = 1UL << (4U + mb);

        CAN1->CANFD_CR |= CANFD_CR_TX_FD;
        CAN1->CANFD_CR = (CAN1->CANFD_CR & ~(brs_bit | esi_bit)) |
                         (f->brs ? brs_bit : 0U) | (f->esi ? esi_bit : 0U);
        CAN1->CANFD_DMA_T[mb] = (uint32_t)(uintptr_t)tx_buf[mb];
    } else {
        /* Classic frame: TX_FD must be clear, otherwise the cell sends the
         * mailbox as an FD frame and reads the payload from the DMA buffer
         * (i.e. whatever was in it).  A classic frame carries its data in
         * TDLR/TDHR (vendor CAN_Transmit()). */
        CAN1->CANFD_CR &= ~CANFD_CR_TX_FD;
        CAN1->sTxMailBox[mb].TXMDLR = lo;
        CAN1->sTxMailBox[mb].TXMDHR = hi;
    }
    /* Full value again with TXRQ set: no read-modify-write on a register whose
     * request bits read back as zero. */
    CAN1->sTxMailBox[mb].TXMIR   = mir | (1UL << 0);

    return true;
}

/* -------------------------------------------------------------------------- */
/* Receive                                                                    */
/* -------------------------------------------------------------------------- */
static bool can_read_fifo(uint32_t fifo, can_ch32_frame_t *f)
{
    /* One receive FIFO on this part: RFIFO0 is the only status register. */
    volatile uint32_t *rfifo = &CAN1->RFIFO0;

    (void)fifo;

    /* Overrun means at least one frame was lost: FULL0/FOVR0 are read/clear-
     * by-writing-0 bits, so write them back as 0 and count the event.  Not
     * clearing them would keep the flag set for the rest of the session. */
    const uint32_t status = *rfifo;

    if ((status & CAN_RFIFO0_FOVR0) != 0U) {
        s_rx_overruns++;
        *rfifo = status & ~(uint32_t)(CAN_RFIFO0_FOVR0 | CAN_RFIFO0_FULL0);
    }

    if ((status & CAN_RFIFO0_FMP0) == 0U) {
        return false;
    }

    const uint32_t mir  = CAN1->sFIFOMailBox[fifo].RXMIR;
    const uint32_t mdtr = CAN1->sFIFOMailBox[fifo].RXMDTR;

    /* DLC, BRS and ESI live in RXMDTR (vendor CANFD_Receive()); the data
     * registers carry the payload of a classic frame, DLC > 8 means the
     * payload was DMA-ed into CANFD_DMA_R[fifo]. */
    const uint32_t dlc = mdtr & 0x0FU;

    f->extended = ((mir >> 2) & 1U) != 0U;
    f->rtr      = ((mir >> 1) & 1U) != 0U;
    f->id       = f->extended ? ((mir >> 3) & 0x1FFFFFFFU) : ((mir >> 21) & 0x7FFU);
    f->fd       = ((mir & 0x1U) != 0U);          /* RXMIR bit0 = FDF */
    f->brs      = ((mdtr >> 4) & 1U) != 0U;
    f->esi      = ((mdtr >> 5) & 1U) != 0U;
    f->len      = (uint8_t)dlc_len[dlc];

    if (dlc <= 8U) {
        const uint32_t dlr = CAN1->sFIFOMailBox[fifo].RXMDLR;
        const uint32_t dhr = CAN1->sFIFOMailBox[fifo].RXMDHR;

        for (uint32_t i = 0U; (i < 4U) && (i < f->len); i++) {
            f->data[i] = (uint8_t)(dlr >> (8U * i));
        }
        for (uint32_t i = 4U; (i < 8U) && (i < f->len); i++) {
            f->data[i] = (uint8_t)(dhr >> (8U * (i - 4U)));
        }
    } else {
        for (uint32_t i = 0U; (i < f->len) && (i < CAN_CH32_MAX_LEN); i++) {
            f->data[i] = rx_buf[fifo][i];
        }
    }

    /* Release the FIFO output mailbox. */
    *rfifo |= CAN_RFIFO0_RFOM0;
    return true;
}

bool can_ch32_receive(can_ch32_frame_t *f)
{
    RCC->APB1PCENR |= RCC_APB1Periph_CAN1;

    const uint32_t tail = rx_ring_tail;
    if (tail == rx_ring_head) {
        return false;
    }

    *f = rx_ring[tail];
    __sync_synchronize();
    rx_ring_tail = (tail + 1U) & (CAN_CH32_RX_RING - 1U);
    return true;
}

void can_ch32_irq_handler(void)
{
    can_ch32_frame_t f;

    RCC->APB1PCENR |= RCC_APB1Periph_CAN1;

    /*
     * Drain the one-deep hardware FIFO into the ring.  If the ring is full,
     * keep reading and dropping: leaving a frame pending would retrigger the
     * interrupt immediately, and the main loop gets the next chance to catch
     * up once it has moved frames to USB.
     */
    while (can_read_fifo(0U, &f)) {
        const uint32_t head = rx_ring_head;
        const uint32_t next = (head + 1U) & (CAN_CH32_RX_RING - 1U);

        if (next == rx_ring_tail) {
            s_rx_overruns++;
            continue;
        }

        rx_ring[head] = f;
        __sync_synchronize();
        rx_ring_head = next;
    }
}

/* USB LP and CAN1 RX0 share this vector on the CH32L103; the USB stack uses
 * USBFS_IRQn, so RX0 is free for the CAN receive FIFO. */
void USB_LP_CAN1_RX0_IRQHandler(void) INTERRUPT_DECORATOR;
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    can_ch32_irq_handler();
}

/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */
void can_ch32_get_state(can_ch32_state_t *st)
{
    RCC->APB1PCENR |= RCC_APB1Periph_CAN1;

    const uint32_t esr = CAN1->ERRSR;

    st->tx_error_count = (esr >> 16) & 0xFFU;
    st->rx_error_count = (esr >> 24) & 0xFFU;
    st->warning        = (esr & 0x1U) != 0U;
    st->error_passive  = (esr & 0x2U) != 0U;
    st->bus_off        = (esr & 0x4U) != 0U;
}
