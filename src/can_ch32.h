/*
 * can_ch32.h
 *
 * CH32L103 CAN / CANFD controller driver (bxCAN compatible registers plus the
 * WCH CANFD extension).  Register level: no vendor library, only ch32fun's
 * register definitions.
 *
 * Framing model, from the CH32L103 EVT:
 *   - the arbitration phase timing lives in CAN->BTIMR
 *   - the data phase timing lives in CAN->CANFD_BTR (BRP_FD/TS1_FD/TS2_FD/
 *     SJW_FD) and CAN->CANFD_TDCT (TDCO/TDC_FILTER); CANFD_Init() in the EVT
 *     copies the arbitration values there, which is why this driver writes
 *     CANFD_BTR itself
 *   - FD payloads are moved by the controller: CAN->CANFD_DMA_T[mailbox] and
 *     CAN->CANFD_DMA_R[fifo] hold the *address* of a 64 byte buffer, so every
 *     mailbox/FIFO needs its own buffer here
 *   - BRS/ESI are per *mailbox* settings (CANFD_CR), not per frame, so the
 *     transmit path picks a mailbox whose BRS setting matches the frame
 */
#ifndef CAN_CH32_H
#define CAN_CH32_H

#include <stdint.h>
#include <stdbool.h>

#define CAN_CH32_MAX_LEN     64U
#define CAN_CH32_NUM_TX_MB   3U
/* The CH32L103 CANFD cell has one receive FIFO (CANFD_DMA_R[1], RFIFO0
 * only) - the vendor struct has no RFIFO1/DMA_R[1]. */
#define CAN_CH32_NUM_RX_FIFO 1U

/* One frame in either direction; len is the real byte count (0..64), fd tells
 * whether it is an FD frame, brs whether the data phase switched rate. */
typedef struct {
    uint32_t id;
    uint8_t  data[CAN_CH32_MAX_LEN];
    uint8_t  len;
    bool     extended;
    bool     rtr;
    bool     fd;
    bool     brs;
    bool     esi;
    uint16_t timestamp;     /* free running CAN time base, 1 tick = 1 bit time */
} can_ch32_frame_t;

typedef struct {
    uint32_t bitrate;       /* bit/s */
    uint16_t sample_point;  /* per mille, e.g. 875 = 87.5% */
    uint16_t sjw;           /* in time quanta */

    /* The host (Linux gs_usb, cangaroo, ...) computes the segments itself
     * from the bittiming constants this port advertises, so its numbers are
     * authoritative: can_ch32_init() programs them verbatim when they are
     * within hardware range, and only solves them itself otherwise (0 =
     * not provided).  After a successful init the fields hold what was
     * really programmed. */
    uint16_t brp;           /* prescaler, 1..1024 (data phase 1..32) */
    uint16_t ts1;           /* prop_seg + phase_seg1, 1..16 */
    uint16_t ts2;           /* phase_seg2, 1..8 */
} can_ch32_timing_t;

/* Bring the controller up.  pclk1 is the APB1 clock the CAN cell runs from.
 * data is ignored when fd is false.  Returns false if the timing could not be
 * represented in the hardware (wrong clock or impossible sample point). */
/* listen_only : BTIMR.SILM - receive only, never drive the bus.
 * loopback    : BTIMR.LBKM - the controller receives its own frames internally,
 *               so a node without a transceiver (bench board) can still transmit
 *               and see the frames come back on the RX path. */
bool can_ch32_init(uint32_t pclk1,
                   can_ch32_timing_t *nominal,
                   can_ch32_timing_t *data,
                   bool fd,
                   bool listen_only,
                   bool loopback,
                   bool tdc_set,
                   uint8_t tdco,
                   uint8_t tdcf);

/* True once can_ch32_init() ran with fd == true and succeeded.  The bench
 * transmitter uses it to fall back to a classic frame if the FD
 * initialisation did not take. */
bool can_ch32_fd_enabled(void);

void can_ch32_stop(void);

/* Queue one frame.  Returns false when no mailbox is free (or the frame cannot
 * be represented, e.g. FD requested but the controller is in classic mode). */
bool can_ch32_send(const can_ch32_frame_t *f);

/* Pull one received frame out of the FIFOs.  Returns false when both are
 * empty. */
bool can_ch32_receive(can_ch32_frame_t *f);

/* Frames dropped by a receive FIFO overrun since the last start. */
uint32_t can_ch32_rx_overruns(void);

/* Measured transmitter delay (CANFD_PSR.TDCV) minus the programmed
 * offset, i.e. the pure loop delay the hardware measured; 0 until an
 * FD frame with BRS has been sent. */
uint32_t can_ch32_tdcv(void);

/* One bus error as reported by the cell (CAN_ERRSR), plus the error counters
 * at the time it happened. */
typedef struct {
    uint8_t lec;            /* last error code, 1..6 */
    uint8_t tec;            /* transmit error counter */
    uint8_t rec;            /* receive error counter */
    bool    warning;
    bool    error_passive;
    bool    bus_off;
} can_ch32_error_t;

/* Non-destructive: is a bus error latched in CAN_ERRSR.LEC? */
bool can_ch32_error_pending(void);

/* Reads and re-arms the LEC latch (LEC = 7, "set by software"). */
bool can_ch32_take_error(can_ch32_error_t *e);

/* Receive FIFO overrun: pending check is non-destructive, take() marks it
 * reported so every lost frame is announced exactly once. */
bool can_ch32_rx_overrun_pending(void);
bool can_ch32_take_rx_overrun(void);


/* Bus state for gs_usb's GET_STATE / error counters. */
typedef struct {
    uint32_t rx_error_count;
    uint32_t tx_error_count;
    bool     bus_off;
    bool     error_passive;
    bool     warning;
} can_ch32_state_t;

void can_ch32_get_state(can_ch32_state_t *st);

/* Called from the CAN interrupt: drains the FIFOs into this driver's ring. */
void can_ch32_irq_handler(void);

#endif /* CAN_CH32_H */
