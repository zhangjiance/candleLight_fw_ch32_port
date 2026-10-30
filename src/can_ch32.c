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

    if (t->bitrate == 0U) {
        return false;                     /* host sent no bittiming yet */
    }

    for (uint32_t brp_try = 1; brp_try <= 1024U; brp_try++) {
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

bool can_ch32_init(uint32_t pclk1, const can_ch32_timing_t *nominal,
                   const can_ch32_timing_t *data, bool fd,
                   bool listen_only, bool loopback)
{
    uint32_t brp, ts1, ts2, sjw;

    if (!solve_timing(pclk1, nominal, &brp, &ts1, &ts2, &sjw)) {
        return false;
    }

    fd_enabled = fd;

    RCC->APB1PCENR |= RCC_APB1Periph_CAN1;

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

        if (data == NULL || !solve_timing(pclk1, data, &dbrp, &dts1, &dts2, &dsjw)) {
            return false;
        }

        CAN1->CANFD_BTR = (1UL << 23) |                                  /* TDCE  */
                          (((dbrp - 1U) << 16) & 0x001F0000U) |
                          (((dts1 - 1U) << 8) & 0x00001F00U) |
                          (((dts2 - 1U) << 4) & 0x000000F0U) |
                          ((dsjw - 1U) & 0x0000000FU);

        CAN1->CANFD_TDCT = (2UL) | ((4UL & 0x3FU) << 8);
        CAN1->CANFD_CR &= ~(CANFD_CR_TX_BRS_B | CANFD_CR_USER_ESI_B);
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

    CAN1->INTENR = 0U;

    CAN1->CTLR &= ~(uint32_t)CAN_CTLR_INRQ;
    if (!can_wait_inak(false)) {
        return false;
    }

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

    if ((*rfifo & CAN_RFIFO0_FMP0) == 0U) {
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
    RCC->APB1PCENR |= RCC_APB1Periph_CAN1;

    const uint32_t esr = CAN1->ERRSR;

    st->tx_error_count = (esr >> 16) & 0xFFU;
    st->rx_error_count = (esr >> 24) & 0xFFU;
    st->warning        = (esr & 0x1U) != 0U;
    st->error_passive  = (esr & 0x2U) != 0U;
    st->bus_off        = (esr & 0x4U) != 0U;
}
