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
#define CAN_CH32_NUM_RX_FIFO 2U

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
} can_ch32_timing_t;

/* Bring the controller up.  pclk1 is the APB1 clock the CAN cell runs from.
 * data is ignored when fd is false.  Returns false if the timing could not be
 * represented in the hardware (wrong clock or impossible sample point). */
bool can_ch32_init(uint32_t pclk1,
                   const can_ch32_timing_t *nominal,
                   const can_ch32_timing_t *data,
                   bool fd,
                   bool listen_only);

void can_ch32_stop(void);

/* Queue one frame.  Returns false when no mailbox is free (or the frame cannot
 * be represented, e.g. FD requested but the controller is in classic mode). */
bool can_ch32_send(const can_ch32_frame_t *f);

/* Pull one received frame out of the FIFOs.  Returns false when both are
 * empty. */
bool can_ch32_receive(can_ch32_frame_t *f);

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
