/****************************************************************************
 * arch/arm/src/stm32h7/stm32_fdcan.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/can.h>
#include <nuttx/wdog.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/can.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32h7_config.h"
#include "hardware/stm32_fdcan.h"
#include "hardware/stm32_pinmap.h"
#include "stm32_fdcan.h"

#include <arch/board/board.h>

#ifdef CONFIG_NET_CMSG
#include <sys/time.h>
#endif

#ifdef CONFIG_STM32H7_FDCAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#define CANWORK LPWORK

/* CONFIG_STM32_FLEXCAN_NETHIFS determines the number of physical
 * interfaces that will be supported.
 */

#define MASKSTDID                   0x000007ff
#define MASKEXTID                   0x1fffffff
#define FLAGEFF                     (1 << 31) /* Extended frame format */
#define FLAGRTR                     (1 << 30) /* Remote transmission request */

#define RXMBCOUNT                   5
#define TXMBCOUNT                   2
#define TOTALMBCOUNT                RXMBCOUNT + TXMBCOUNT

#define IFLAG1_RX                   ((1 << RXMBCOUNT)-1)
#define IFLAG1_TX                   (((1 << TXMBCOUNT)-1) << RXMBCOUNT)

#define CAN_FIFO_NE                 (1 << 5)
#define CAN_FIFO_OV                 (1 << 6)
#define CAN_FIFO_WARN               (1 << 7)
#define CAN_EFF_FLAG                0x80000000 /* EFF/SFF is set in the MSB */

#define POOL_SIZE                   1

#ifdef CONFIG_NET_CMSG
#define MSG_DATA                    sizeof(struct timeval)
#else
#define MSG_DATA                    0
#endif

/* CAN bit timing values  */
#define CLK_FREQ                    STM32_FDCANCLK
#define PRESDIV_MAX                 256
/// TODO: All of these min/max vals...
#define SEG_MAX                     8
#define SEG_MIN                     1
#define TSEG_MIN                    2
#define TSEG1_MAX                   17
#define TSEG2_MAX                   9
#define NUMTQ_MAX                   26

#define SEG_FD_MAX                  32
#define SEG_FD_MIN                  1
#define TSEG_FD_MIN                 2
#define TSEG1_FD_MAX                39
#define TSEG2_FD_MAX                9
#define NUMTQ_FD_MAX                49

#define WORD_LENGTH                 4U
#define FIFO_ELEMENT_SIZE           4U // size (in Words) of a FIFO element in message RAM

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE

#  if !defined(CONFIG_SCHED_WORKQUEUE)
#    error Work queue support is required
#  endif

#define TX_TIMEOUT_WQ
#endif

/* Interrupt flags for RX fifo */
#define IFLAG1_RXFIFO               (CAN_FIFO_NE | CAN_FIFO_WARN | CAN_FIFO_OV)

static int peak_tx_mailbox_index_ = 0;

/****************************************************************************
 * Private Types
 ****************************************************************************/

union cs_e
{
  volatile uint32_t cs;
  struct
  {
    volatile uint32_t time_stamp : 16;
    volatile uint32_t dlc : 4;
    volatile uint32_t rtr : 1;
    volatile uint32_t ide : 1;
    volatile uint32_t srr : 1;
    volatile uint32_t res : 1;
    volatile uint32_t code : 4;
    volatile uint32_t res2 : 1;
    volatile uint32_t esi : 1;
    volatile uint32_t brs : 1;
    volatile uint32_t edl : 1;
  };
};

union id_e
{
  volatile uint32_t w;
  struct
  {
    volatile uint32_t ext : 29;
    volatile uint32_t resex : 3;
  };
  struct
  {
    volatile uint32_t res : 18;
    volatile uint32_t std : 11;
    volatile uint32_t resstd : 3;
  };
};

union data_e
{
  volatile uint32_t w00;
  struct
  {
    volatile uint32_t b03 : 8;
    volatile uint32_t b02 : 8;
    volatile uint32_t b01 : 8;
    volatile uint32_t b00 : 8;
  };
};

struct mb_s
{
  union cs_e cs;
  union id_e id;
#ifdef CONFIG_NET_CAN_CANFD
  union data_e data[16];
#else
  union data_e data[2];
#endif
};

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
#define TX_ABORT -1
#define TX_FREE 0
#define TX_BUSY 1

struct txmbstats
{
  struct timeval deadline;
  uint32_t pending; /* -1 = abort, 0 = free, 1 = busy  */
};
#endif

/* FDCAN Device hardware configuration */

struct fdcan_config_s
{
  uint32_t tx_pin;           /* GPIO configuration for TX */
  uint32_t rx_pin;           /* GPIO configuration for RX */
  uint32_t enable_pin;       /* Optional enable pin */
  uint32_t enable_high;      /* Optional enable high/low */
  uint32_t mb_irq[2];        /* FDCAN Interrupt 0, 1 (Rx, Tx) */
};

struct fdcan_timeseg
{
  uint32_t bitrate;
  int32_t samplep;
  uint8_t propseg;
  uint8_t pseg1;
  uint8_t pseg2;
  uint8_t presdiv;
  /// JACOB TODO: Replace with values needed by FDCAN controller
};

/* FDCAN device structures */

#ifdef CONFIG_STM32_FDCAN1
static const struct fdcan_config_s stm32_fdcan0_config =
{
  .tx_pin      = GPIO_CAN1_TX,
  .rx_pin      = GPIO_CAN1_RX,
#ifdef GPIO_CAN1_ENABLE
  .enable_pin  = GPIO_CAN1_ENABLE,
  .enable_high = CAN1_ENABLE_OUT,
#else
  .enable_pin  = 0,
  .enable_high = 0,
#endif
  .mb_irq      =
  {
    STM32_IRQ_FDCAN1_0,
    STM32_IRQ_FDCAN1_1,
  },
};
#endif

#ifdef CONFIG_STM32_FDCAN2
static const struct fdcan_config_s stm32_fdcan1_config =
{
  .tx_pin      = PIN_CAN1_TX,
  .rx_pin      = PIN_CAN1_RX,
#ifdef GPIO_CAN2_ENABLE
  .enable_pin  = GPIO_CAN2_ENABLE,
  .enable_high = CAN2_ENABLE_OUT,
#else
  .enable_pin  = 0,
  .enable_high = 0,
#endif
  .mb_irq      =
  {
    STM32_IRQ_FDCAN2_0,
    STM32_IRQ_FDCAN2_1 ,
  },
};
#endif

/* The stm32_driver_s encapsulates all state information for a single
 * hardware interface
 */

struct stm32_driver_s
{
  uint32_t base;                /* FDCAN base address */
  uint8_t iface_idx;            /* FDCAN interface index (0 or 1) */
  bool bifup;                   /* true:ifup false:ifdown */
#ifdef TX_TIMEOUT_WQ
  WDOG_ID txtimeout[TXMBCOUNT]; /* TX timeout timer */
#endif
  struct work_s irqwork;        /* For deferring interrupt work to the wq */
  struct work_s pollwork;       /* For deferring poll work to the work wq */
#ifdef CONFIG_NET_CAN_CANFD
  struct canfd_frame *txdesc;   /* A pointer to the list of TX descriptor */
  struct canfd_frame *rxdesc;   /* A pointer to the list of RX descriptors */
#else
  struct can_frame *txdesc;     /* A pointer to the list of TX descriptor */
  struct can_frame *rxdesc;     /* A pointer to the list of RX descriptors */
#endif

  /* This holds the information visible to the NuttX network */

  struct net_driver_s dev;      /* Interface understood by the network */

  struct mb_s *rx;
  struct mb_s *tx;

  struct fdcan_timeseg arbi_timing; /* Timing for arbitration phase */
#ifdef CONFIG_NET_CAN_CANFD
  struct fdcan_timeseg data_timing; /* Timing for data phase */
#endif

  const struct fdcan_config_s *config;

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  struct txmbstats txmb[TXMBCOUNT];
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_STM32_FDCAN1
static struct stm32_driver_s g_fdcan0;
#endif

#ifdef CONFIG_STM32_FDCAN2
static struct stm32_driver_s g_fdcan1;
#endif

#ifdef CONFIG_NET_CAN_CANFD
static uint8_t g_tx_pool[(sizeof(struct canfd_frame)+MSG_DATA)*POOL_SIZE];
static uint8_t g_rx_pool[(sizeof(struct canfd_frame)+MSG_DATA)*POOL_SIZE];
#else
static uint8_t g_tx_pool[sizeof(struct can_frame)*POOL_SIZE];
static uint8_t g_rx_pool[sizeof(struct can_frame)*POOL_SIZE];
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: arm_lsb
 *
 * Description:
 *   Calculate position of lsb that's equal to 1
 *
 * Input Parameters:
 *   value - The value to perform the operation on
 *
 * Returned Value:
 *   location of lsb which is equal to 1, returns 32 when value is 0
 *
 ****************************************************************************/

static inline uint32_t arm_lsb(unsigned int value)
{
  uint32_t ret;
  volatile uint32_t rvalue = value;
  __asm__ __volatile__ ("rbit %1,%0" : "=r" (rvalue) : "r" (rvalue));
  __asm__ __volatile__ ("clz %0, %1" : "=r"(ret) : "r"(rvalue));
  return ret;
}

/****************************************************************************
 * Name: stm32_bitratetotimeseg
 *
 * Description:
 *   Convert bitrate to timeseg
 *
 * Input Parameters:
 *   timeseg - structure to store bit timing
 *   sp_tolerance - allowed difference in sample point from calculated
 *                  bit timings (recommended value: 1)
 *   can_fd - if set to calculate CAN FD bit timings, otherwise calculate
 *            classical can timings
 *
 * Returned Value:
 *   return 1 on succes, return 0 on failure
 *
 ****************************************************************************/
/// JACOB TODO: Replace with STM32 FDCAN-compatible bit timing calculator
uint32_t stm32_bitratetotimeseg(struct fdcan_timeseg *timeseg,
                                                int32_t sp_tolerance,
                                                uint32_t can_fd)
{
  int32_t tmppresdiv;
  int32_t numtq;
  int32_t tmpsample;
  int32_t tseg1;
  int32_t tseg2;
  int32_t tmppseg1;
  int32_t tmppseg2;
  int32_t tmppropseg;

  const int32_t TSEG1MAX = (can_fd ? TSEG1_FD_MAX : TSEG1_MAX);
  const int32_t TSEG2MAX = (can_fd ? TSEG2_FD_MAX : TSEG2_MAX);
  const int32_t SEGMAX = (can_fd ? SEG_FD_MAX : SEG_MAX);
  const int32_t NUMTQMAX = (can_fd ? NUMTQ_FD_MAX : NUMTQMAX);

  for (tmppresdiv = 0; tmppresdiv < PRESDIV_MAX; tmppresdiv++)
    {
      numtq = (CLK_FREQ / ((tmppresdiv + 1) * timeseg->bitrate));

      if (numtq == 0)
        {
          continue;
        }

      /* The number of time quanta in 1 bit time must be lower than the one supported */

      if ((CLK_FREQ / ((tmppresdiv + 1) * numtq) == timeseg->bitrate)
          && (numtq >= 8) && (numtq < NUMTQMAX))
        {
          /* Compute time segments based on the value of the sampling point */

          tseg1 = (numtq * timeseg->samplep / 100) - 1;
          tseg2 = numtq - 1 - tseg1;

          /* Adjust time segment 1 and time segment 2 */

          while (tseg1 >= TSEG1MAX || tseg2 < TSEG_MIN)
            {
              tseg2++;
              tseg1--;
            }

          tmppseg2 = tseg2 - 1;

          /* Start from pseg1 = pseg2 and adjust until propseg is valid */

          tmppseg1 = tmppseg2;
          tmppropseg = tseg1 - tmppseg1 - 2;

          while (tmppropseg <= 0)
            {
              tmppropseg++;
              tmppseg1--;
            }

          while (tmppropseg >= SEGMAX)
            {
              tmppropseg--;
              tmppseg1++;
            }

          if (((tseg1 >= TSEG1MAX) || (tseg2 >= TSEG2MAX) ||
              (tseg2 < TSEG_MIN) || (tseg1 < TSEG_MIN)) ||
              ((tmppropseg >= SEGMAX) || (tmppseg1 >= SEGMAX) ||
                  (tmppseg2 < SEG_MIN) || (tmppseg2 >= SEGMAX)))
            {
              continue;
            }

          tmpsample = ((tseg1 + 1) * 100) / numtq;

          if ((tmpsample - timeseg->samplep) <= sp_tolerance &&
              (timeseg->samplep - tmpsample) <= sp_tolerance)
            {
              if (can_fd == 1)
                {
                  timeseg->propseg = tmppropseg + 1;
                }
              else
                {
                  timeseg->propseg = tmppropseg;
                }
              timeseg->pseg1 = tmppseg1;
              timeseg->pseg2 = tmppseg2;
              timeseg->presdiv = tmppresdiv;
              timeseg->samplep = tmpsample;
              return 1;
            }
        }
    }

  return 0;
}

/* Common TX logic */

static bool stm32_txringfull(FAR struct stm32_driver_s *priv);
static int  stm32_transmit(FAR struct stm32_driver_s *priv);
static int  stm32_txpoll(struct net_driver_s *dev);

/* Helper functions */

static void stm32_setinit(uint32_t base, uint32_t init);
static void stm32_setenable(uint32_t base, uint32_t enable);
static void stm32_setfreeze(uint32_t base, uint32_t freeze);
static uint32_t stm32_waitccr_change(uint32_t base,
                                       uint32_t mask,
                                       uint32_t target_state);

/* Interrupt handling */

static void stm32_receive(FAR struct stm32_driver_s *priv);
static void stm32_txdone(FAR struct stm32_driver_s *priv);

static int  stm32_fdcan_interrupt(int irq, FAR void *context,
                                      FAR void *arg);

/* Watchdog timer expirations */
#ifdef TX_TIMEOUT_WQ
static void stm32_txtimeout_work(FAR void *arg);
static void stm32_txtimeout_expiry(int argc, uint32_t arg, ...);
#endif

/* NuttX callback functions */

static int  stm32_ifup(struct net_driver_s *dev);
static int  stm32_ifdown(struct net_driver_s *dev);

static void stm32_txavail_work(FAR void *arg);
static int  stm32_txavail(struct net_driver_s *dev);

#ifdef CONFIG_NETDEV_IOCTL
static int  stm32_ioctl(struct net_driver_s *dev, int cmd,
                          unsigned long arg);
#endif

/* Initialization */

static int  stm32_initialize(struct stm32_driver_s *priv);
static void stm32_reset(struct stm32_driver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: stm32_txringfull
 *
 * Description:
 *   Check if all of the TX descriptors are in use.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   true is the TX ring is full; false if there are free slots at the
 *   head index.
 *
 ****************************************************************************/

static bool stm32_txringfull(FAR struct stm32_driver_s *priv)
{
  // Check that we even _have_ a Tx FIFO allocated
  /// JACOB TODO: Decide if this needs to be checked every time, or just during init
	if ((getreg32(priv->base + STM32_FDCAN_TXBC_OFFSET) & FDCAN_TXBC_TFQS) == 0) {
		// Your queue size is 0, you did something wrong
		return true;
	}

	// Check if the Tx queue is full
	if ((getreg32(priv->base + STM32_FDCAN_TXFQS_OFFSET) & FDCAN_TXFQS_TFQF) == FDCAN_TXFQS_TFQF) {
		// Sorry, out of room, try back later
		return true;
	}

	return false;
}

/****************************************************************************
 * Function: stm32_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int stm32_transmit(FAR struct stm32_driver_s *priv)
{
  /** JACOB TODO ---
    0. Assumptions: dev.d_buf contains a frame waiting to be transmitted.
    1. Find the next free Tx mailbox index
    2. Copy the frame from dev.d_buf (points to txdesc array) into CAN msg RAM
    3. Request hardware transmission of the mailbox item
    4. Notify the NuttX network layer of transmission
    5. Setup Tx timout watchdog if enabled
  */

  /* From UAVCAN uc_stm32h7_can.cpp: */
	/*
	 * Normally we should perform the same check as in @ref canAcceptNewTxFrame(), because
	 * it is possible that the highest-priority frame between select() and send() could have been
	 * replaced with a lower priority one due to TX timeout. But we don't do this check because:
	 *
	 *  - It is a highly unlikely scenario.
	 *
	 *  - Frames do not timeout on a properly functioning bus. Since frames do not timeout, the new
	 *    frame can only have higher priority, which doesn't break the logic.
	 *
	 *  - If high-priority frames are timing out in the TX queue, there's probably a lot of other
	 *    issues to take care of before this one becomes relevant.
	 *
	 *  - It takes CPU time. Not just CPU time, but critical section time, which is expensive.
	 */
	// CriticalSectionLocker lock;

	// First, check if there are any slots available in the queue
	if ((getreg32(priv->base + STM32_FDCAN_TXFQS_OFFSET) & FDCAN_TXFQS_TFQF) == FDCAN_TXFQS_TFQF) {
		// Tx FIFO / Queue is full
		return -EBUSY; /// JACOB TODO: why does the NXP Flexcan driver always return OK?
	}

	// Next, get the next available queue index from the controller
	const uint8_t mbi = (getreg32(priv->base + STM32_FDCAN_TXFQS_OFFSET) & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_SHIFT;

	// Now, we can copy the CAN frame to the queue (in message RAM)
  /// JACOB TODO: Ensure priv->tx points to message RAM TX Queue
  struct mb_s *mb = &priv->tx[mbi];
	//uint32_t *txbuf  = (uint32_t *)(message_ram_.TxQueueSA + (index * FIFO_ELEMENT_SIZE * WORD_LENGTH));

  /* Attempt to write frame */

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  int32_t timeout = 0;
  struct timespec ts;
  clock_systimespec(&ts);

  if (priv->dev.d_sndlen > priv->dev.d_len)
    {
      struct timeval *tv =
             (struct timeval *)(priv->dev.d_buf + priv->dev.d_len);
      priv->txmb[mbi].deadline = *tv;
      timeout  = (tv->tv_sec - ts.tv_sec)*CLK_TCK
                 + ((tv->tv_usec - ts.tv_nsec / 1000)*CLK_TCK) / 1000000;
      if (timeout < 0)
        {
          return 0;  /* No transmission for you! */
        }
    }
  else
    {
      /* Default TX deadline defined in NET_CAN_RAW_DEFAULT_TX_DEADLINE */

      if (CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE > 0)
        {
          timeout = ((CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE / 1000000)
              *CLK_TCK);
          priv->txmb[mbi].deadline.tv_sec = ts.tv_sec +
              CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE / 1000000;
          priv->txmb[mbi].deadline.tv_usec = (ts.tv_nsec / 1000) +
              CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE % 1000000;
        }
      else
        {
          priv->txmb[mbi].deadline.tv_sec = 0;
          priv->txmb[mbi].deadline.tv_usec = 0;
        }
    }
#endif

  /* For statistics purposes, keep track of highest waterline in TX mailbox */
  peak_tx_mailbox_index_ =
    (peak_tx_mailbox_index_ > mbi ? peak_tx_mailbox_index_ : mbi);

  union cs_e cs;
  cs.code = CAN_TXMB_DATAORREMOTE;
  struct mb_s *mb = &priv->tx[mbi];
  mb->cs.code = CAN_TXMB_INACTIVE;

  if (priv->dev.d_len == sizeof(struct can_frame))
    {
      struct can_frame *frame = (struct can_frame *)priv->dev.d_buf;

      if (frame->can_id & CAN_EFF_FLAG)
        {
          cs.ide = 1;
          mb->id.ext = frame->can_id & MASKEXTID;
        }
      else
        {
          mb->id.std = frame->can_id & MASKSTDID;
        }

      cs.rtr = frame->can_id & FLAGRTR ? 1 : 0;
      cs.dlc = frame->can_dlc;

      mb->data[0].w00 = __builtin_bswap32(*(uint32_t *)&frame->data[0]);
      mb->data[1].w00 = __builtin_bswap32(*(uint32_t *)&frame->data[4]);
    }
#ifdef CONFIG_NET_CAN_CANFD
  else /* CAN FD frame */
    {
      struct canfd_frame *frame = (struct canfd_frame *)priv->dev.d_buf;

      cs.edl = 1; /* CAN FD Frame */

      if (frame->can_id & CAN_EFF_FLAG)
        {
          cs.ide = 1;
          mb->id.ext = frame->can_id & MASKEXTID;
        }
      else
        {
          mb->id.std = frame->can_id & MASKSTDID;
        }

      cs.rtr = frame->can_id & FLAGRTR ? 1 : 0;

      cs.dlc = len_to_can_dlc[frame->len];

      uint32_t *frame_data_word = (uint32_t *)&frame->data[0];

      for (int i = 0; i < (frame->len + 4 - 1) / 4; i++)
        {
          mb->data[i].w00 = __builtin_bswap32(frame_data_word[i]);
        }
    }
#endif

  mb->cs = cs; /* Go. */

  // Submit the transmission request for this element
  putreg32(1 << mbi, priv->base + STM32_FDCAN_TXBAR_OFFSET);

  /* Increment statistics */

  NETDEV_TXPACKETS(&priv->dev);

#ifdef TX_TIMEOUT_WQ
  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  if (timeout > 0)
    {
      wd_start(priv->txtimeout[mbi], timeout + 1, stm32_txtimeout_expiry,
                1, (wdparm_t)priv);
    }
#endif

  return OK;
}

/****************************************************************************
 * Function: stm32_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int stm32_txpoll(struct net_driver_s *dev)
{
  FAR struct stm32_driver_s *priv =
    (FAR struct stm32_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      if (!devif_loopback(&priv->dev))
        {
          /* Send the packet */

          stm32_transmit(priv);

          /* Check if there is room in the device to hold another packet. If
           * not, return a non-zero value to terminate the poll.
           */

          if (stm32_txringfull(priv))
            {
              return -EBUSY;
            }
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: stm32_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void stm32_receive(FAR struct stm32_driver_s *priv)
{
  uint32_t ir_flags;
  uint32_t mb_index;
  uint8_t fifo_id;
  struct mb_s *rf;

  uint32_t regval = getreg32(priv->base + STM32_FDCAN_IR_OFFSET);

  const uint32_t ir_fifo0 = FDCAN_IR_RF0N | FDCAN_IR_RF0F;
  const uint32_t ir_fifo1 = FDCAN_IR_RF1N | FDCAN_IR_RF1F;

  if (regval & ir_fifo0)
    {
      regval = ir_fifo0;
      fifo_id = 0;
    }
  else if (regval & ir_fifo1)
    {
      regval = ir_fifo1;
      fifo_id = 1;
    }
  else
    {
      nerr("ERROR: Bad RX IR flags");
      return;
    }

  /* Write the corresponding interrupt bits to reset them */
      
  putreg32(regval, priv->base + STM32_FDCAN_IR_OFFSET);

	// Bitwise register definitions are the same for FIFO 0/1
	const uint32_t FDCAN_RXFnC_FnS      = FDCAN_RXF0C_F0S;  // Rx FIFO Size
	const uint32_t FDCAN_RXFnS_RFnL     = FDCAN_RXF0S_RF0L; // Rx Message Lost
	const uint32_t FDCAN_RXFnS_FnFL     = FDCAN_RXF0S_F0FL; // Rx FIFO Fill Level
	const uint32_t FDCAN_RXFnS_FnGI     = FDCAN_RXF0S_F0GI; // Rx FIFO Get Index
	const uint32_t FDCAN_RXFnS_FnGI_Pos = FDCAN_RXF0S_F0GI_Pos;
	//const uint32_t FDCAN_RXFnS_FnPI     = FDCAN_RXF0S_F0PI; // Rx FIFO Put Index
	//const uint32_t FDCAN_RXFnS_FnPI_Pos = FDCAN_RXF0S_F0PI_Pos;
	//const uint32_t FDCAN_RXFnS_FnF      = FDCAN_RXF0S_F0F; // Rx FIFO Full

  uint32_t offset_rxfnc = (fifo_id == 0) ? STM32_FDCAN_RXF0C_OFFSET : STM32_FDCAN_RXF1C_OFFSET;
  uint32_t offset_rxfns = (fifo_id == 0) ? STM32_FDCAN_RXF0S_OFFSET : STM32_FDCAN_RXF1S_OFFSET;
  uint32_t offset_rxfna = (fifo_id == 0) ? STM32_FDCAN_RXF0A_OFFSET : STM32_FDCAN_RXF1A_OFFSET;

	volatile uint32_t *const RXFnC = priv->base + offset_rxfnc;
	volatile uint32_t *const RXFnS = priv->base + offset_rxfns;
	volatile uint32_t *const RXFnA = priv->base + offset_rxfna;

	bool had_error = false;

	// Check number of elements in message RAM allocated to this FIFO
	if ((*RXFnC & FDCAN_RXFnC_FnS) == 0) {
		nerr("ERROR: No RX FIFO elements allocated");
		return;
	}

	// Check for message lost; count an error
	if ((*RXFnS & FDCAN_RXFnS_RFnL) != 0) {
    /// JACOB: TODO: does NuttX store this kind of frame-drop count somewhere?
		// error_cnt_++;  
	}

	// Check number of elements available (fill level)
	const uint8_t n_elem = (*RXFnS & FDCAN_RXFnS_FnFL);

	if (n_elem == 0) {
		// No elements available?
		nerr("RX interrupt but 0 frames available");
		return;
	}

	while ((*RXFnS & FDCAN_RXFnS_FnFL) > 0)
    {
      // Copy the frame from message RAM

      const uint8_t index = (*RXFnS & FDCAN_RXFnS_FnGI) >> FDCAN_RXFnS_FnGI_Pos;

      rf = &priv->rx[index];

      /* Read the frame contents */

#ifdef CONFIG_NET_CAN_CANFD
      if (rf->cs.edl) /* CAN FD frame */
        {
        struct canfd_frame *frame = (struct canfd_frame *)priv->rxdesc;

          if (rf->cs.ide)
            {
              frame->can_id = MASKEXTID & rf->id.ext;
              frame->can_id |= FLAGEFF;
            }
          else
            {
              frame->can_id = MASKSTDID & rf->id.std;
            }

          if (rf->cs.rtr)
            {
              frame->can_id |= FLAGRTR;
            }

          frame->len = can_dlc_to_len[rf->cs.dlc];

          uint32_t *frame_data_word = (uint32_t *)&frame->data[0];

          for (int i = 0; i < (frame->len + 4 - 1) / 4; i++)
            {
              frame_data_word[i] = __builtin_bswap32(rf->data[i].w00);
            }

          /* Acknowledge receipt of this FIFO element */

          *RXFnA = index;

          /* Copy the buffer pointer to priv->dev..  Set amount of data
          * in priv->dev.d_len
          */

          priv->dev.d_len = sizeof(struct canfd_frame);
          priv->dev.d_buf = (uint8_t *)frame;
        }
      else /* CAN 2.0 Frame */
#endif
        {
        struct can_frame *frame = (struct can_frame *)priv->rxdesc;

          if (rf->cs.ide)
            {
              frame->can_id = MASKEXTID & rf->id.ext;
              frame->can_id |= FLAGEFF;
            }
          else
            {
              frame->can_id = MASKSTDID & rf->id.std;
            }

          if (rf->cs.rtr)
            {
              frame->can_id |= FLAGRTR;
            }

          frame->can_dlc = rf->cs.dlc;

          *(uint32_t *)&frame->data[0] = __builtin_bswap32(rf->data[0].w00);
          *(uint32_t *)&frame->data[4] = __builtin_bswap32(rf->data[1].w00);


          /* Acknowledge receipt of this FIFO element */

          *RXFnA = index;

          /* Copy the buffer pointer to priv->dev..  Set amount of data
          * in priv->dev.d_len
          */

          priv->dev.d_len = sizeof(struct can_frame);
          priv->dev.d_buf = (uint8_t *)frame;
        }

      /* Send to socket interface */

      NETDEV_RXPACKETS(&priv->dev);

      can_input(&priv->dev);

      /* Point the packet buffer back to the next Tx buffer that will be
      * used during the next write.  If the write queue is full, then
      * this will point at an active buffer, which must not be written
      * to.  This is OK because devif_poll won't be called unless the
      * queue is not full.
      */

      priv->dev.d_buf = (uint8_t *)priv->txdesc;
    }

	// if (had_error) {
	// 	error_cnt_++;
	// }

	// update_event_.signalFromInterrupt();

	// pollErrorFlagsFromISR();
}

/****************************************************************************
 * Function: stm32_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *   The network is locked.
 *
 ****************************************************************************/

static void stm32_txdone(FAR struct stm32_driver_s *priv)
{
  #warning Missing logic

  /* FIXME First Process Error aborts */

  /* Process TX completions */

  uint32_t mb_bit = 1 << RXMBCOUNT;
  for (uint32_t mbi = 0; flags && mbi < TXMBCOUNT; mbi++)
    {
      if (flags & mb_bit)
        {
          putreg32(mb_bit, priv->base + STM32_CAN_IFLAG1_OFFSET);
          flags &= ~mb_bit;
          NETDEV_TXDONE(&priv->dev);
#ifdef TX_TIMEOUT_WQ
          /* We are here because a transmission completed, so the
           * corresponding watchdog can be canceled.
           */

          wd_cancel(priv->txtimeout[mbi]);
#endif
        }

      mb_bit <<= 1;
    }

  /* There should be space for a new TX in any event.  Poll the network for
   * new XMIT data
   */

  devif_poll(&priv->dev, stm32_txpoll);
}

/****************************************************************************
 * Function: stm32_fdcan_interrupt
 *
 * Description:
 *   Three interrupt sources will vector this this function:
 *   1. CAN MB transmit interrupt handler
 *   2. CAN MB receive interrupt handler
 *   3.
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_fdcan_interrupt(int irq, FAR void *context,
                                     FAR void *arg)
{
  FAR struct stm32_driver_s *priv = (struct stm32_driver_s *)arg;

  if (irq == priv->config->mb_irq[0])
    {
      /* Rx Interrupt */

      stm32_receive(priv);

    }
  else if (irq == priv->config->mb_irq[1])
    {
      /* Tx Complete Interrupt */

      stm32_txdone(priv);
    }

  return OK;

}

/****************************************************************************
 * Function: stm32_txtimeout_work
 *
 * Description:
 *   Perform TX timeout related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/
#ifdef TX_TIMEOUT_WQ

static void stm32_txtimeout_work(FAR void *arg)
{
  FAR struct stm32_driver_s *priv = (FAR struct stm32_driver_s *)arg;

  struct timespec ts;
  struct timeval *now = (struct timeval *)&ts;
  clock_systimespec(&ts);
  now->tv_usec = ts.tv_nsec / 1000; /* timespec to timeval conversion */

  /* The watchdog timed out, yet we still check mailboxes in case the
   * transmit function transmitted a new frame
   */

  for (int mbi = 0; mbi < TXMBCOUNT; mbi++)
    {
      if (priv->txmb[mbi].deadline.tv_sec != 0
          && (now->tv_sec > priv->txmb[mbi].deadline.tv_sec
          || now->tv_usec > priv->txmb[mbi].deadline.tv_usec))
        {
          NETDEV_TXTIMEOUTS(&priv->dev);
          struct mb_s *mb = &priv->tx[mbi];
          mb->cs.code = CAN_TXMB_ABORT;
          priv->txmb[mbi].pending = TX_ABORT;
        }
    }
}

/****************************************************************************
 * Function: stm32_txtimeout_expiry
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Input Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void stm32_txtimeout_expiry(int argc, uint32_t arg, ...)
{
  FAR struct stm32_driver_s *priv = (FAR struct stm32_driver_s *)arg;

  /* Schedule to perform the TX timeout processing on the worker thread
   */

  work_queue(CANWORK, &priv->irqwork, stm32_txtimeout_work, priv, 0);
}

#endif


static void stm32_setinit(uint32_t base, uint32_t init)
{
  uint32_t regval;

  if (init)
    {
      /* Enter hardware initialization mode */
      
      modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_INIT);
      stm32_waitccr_change(base, FDCAN_CCCR_INIT, FDCAN_CCCR_INIT);
    }
  else
    {
      /* Exit hardware initialization mode */

      modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_INIT, 0);
      stm32_waitccr_change(base, FDCAN_CCCR_INIT, 0);
    }
}

static void stm32_setenable(uint32_t base, uint32_t enable)
{
  uint32_t regval;
  
  if (enable)
    {
      /* Clear CSR bit */

      modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_CSR, 0);
      stm32_waitccr_change(base, FDCAN_CCCR_CSA, 0);
    }
  else
    {
      /* Set CSR bit */

      modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_CSR);
      /// JACOB TODO: different target value to wait on here?
      //stm32_waitccr_change(base, FDCAN_CCCR_CSA, 0);
    }
}

static void stm32_setconfig(uint32_t base, uint32_t config_enable)
{
  uint32_t regval;
  if (config_enable)
    {
      /* Configuration Changes Enabled (CCE) mode */

      modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_CCE);
    }
  else
    {
      /* Exit CCE mode */

      modifyreg32(base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_CCE, 0);
    }
}

static uint32_t stm32_waitccr_change(uint32_t base, uint32_t mask,
                                       uint32_t target_state)
{
  const unsigned timeout = 1000;
  for (unsigned wait_ack = 0; wait_ack < timeout; wait_ack++)
    {
      const bool state = (getreg32(base + STM32_FDCAN_CCCR_OFFSET) & mask)
          != 0;
      if (state == target_state)
        {
          return true;
        }

      up_udelay(10);
    }

  return false;
}

/****************************************************************************
 * Function: stm32_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_ifup(struct net_driver_s *dev)
{
  FAR struct stm32_driver_s *priv =
    (FAR struct stm32_driver_s *)dev->d_private;

  if (!stm32_initialize(priv))
    {
      nerr("initialize failed");
      return -1;
    }

  priv->bifup = true;

#ifdef CONFIG_NET_CAN_CANFD
  priv->txdesc = (struct canfd_frame *)&g_tx_pool;
  priv->rxdesc = (struct canfd_frame *)&g_rx_pool;
#else
  priv->txdesc = (struct can_frame *)&g_tx_pool;
  priv->rxdesc = (struct can_frame *)&g_rx_pool;
#endif

  priv->dev.d_buf = (uint8_t *)priv->txdesc;

  /* Set interrupts */

  up_enable_irq(priv->config->mb_irq[0]);

  return OK;
}

/****************************************************************************
 * Function: stm32_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int stm32_ifdown(struct net_driver_s *dev)
{
  FAR struct stm32_driver_s *priv =
    (FAR struct stm32_driver_s *)dev->d_private;

  stm32_reset(priv);

  priv->bifup = false;
  return OK;
}

/****************************************************************************
 * Function: stm32_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Input Parameters:
 *   arg - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void stm32_txavail_work(FAR void *arg)
{
  FAR struct stm32_driver_s *priv = (FAR struct stm32_driver_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing
       * packet.
       */

      if (!stm32_txringfull(priv))
        {
          /* No, there is space for another transfer.  Poll the network for
           * new XMIT data.
           */

          devif_poll(&priv->dev, stm32_txpoll);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Function: stm32_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int stm32_txavail(struct net_driver_s *dev)
{
  FAR struct stm32_driver_s *priv =
    (FAR struct stm32_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      stm32_txavail_work(priv);
    }

  return OK;
}

/****************************************************************************
 * Function: stm32_ioctl
 *
 * Description:
 *   PHY ioctl command handler
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   cmd  - ioctl command
 *   arg  - Argument accompanying the command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_CAN_BITRATE_IOCTL
static int stm32_ioctl(struct net_driver_s *dev, int cmd,
                         unsigned long arg)
{
  FAR struct stm32_driver_s *priv =
      (FAR struct stm32_driver_s *)dev->d_private;

  int ret;

  switch (cmd)
    {
      case SIOCGCANBITRATE: /* Get bitrate from a CAN controller */
        {
          struct can_ioctl_data_s *req =
              (struct can_ioctl_data_s *)((uintptr_t)arg);
          req->arbi_bitrate = priv->arbi_timing.bitrate / 1000; /* kbit/s */
          req->arbi_samplep = priv->arbi_timing.samplep;
#ifdef CONFIG_NET_CAN_CANFD
          req->data_bitrate = priv->data_timing.bitrate / 1000; /* kbit/s */
          req->data_samplep = priv->data_timing.samplep;
#else
          req->data_bitrate = 0;
          req->data_samplep = 0;
#endif
          ret = OK;
        }
        break;

      case SIOCSCANBITRATE: /* Set bitrate of a CAN controller */
        {
          struct can_ioctl_data_s *req =
              (struct can_ioctl_data_s *)((uintptr_t)arg);

          struct fdcan_timeseg arbi_timing;
          arbi_timing.bitrate = req->arbi_bitrate * 1000;
          arbi_timing.samplep = req->arbi_samplep;

          if (stm32_bitratetotimeseg(&arbi_timing, 10, 0))
            {
              ret = OK;
            }
          else
            {
              ret = -EINVAL;
            }

#ifdef CONFIG_NET_CAN_CANFD
          struct fdcan_timeseg data_timing;
          data_timing.bitrate = req->data_bitrate * 1000;
          data_timing.samplep = req->data_samplep;

          if (ret == OK && stm32_bitratetotimeseg(&data_timing, 10, 1))
            {
              ret = OK;
            }
          else
            {
              ret = -EINVAL;
            }
#endif

          if (ret == OK)
            {
              /* Reset CAN controller and start with new timings */

              priv->arbi_timing = arbi_timing;
#ifdef CONFIG_NET_CAN_CANFD
              priv->data_timing = data_timing;
#endif
              stm32_ifup(dev);
            }
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}
#endif /* CONFIG_NETDEV_IOCTL */

/****************************************************************************
 * Function: stm32_initialize
 *
 * Description:
 *   Initialize FDCAN device
 *
 * Input Parameters:
 *   priv - Reference to the private FDCAN driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

int stm32_initialize(struct stm32_driver_s *priv)
{
  uint32_t regval;
  
  /// NOTE: modifyreg32(addr, clearbits, setbits)
	
  /*
	 * Wake up the device and enable configuration changes
	 */
  // Exit Power-down / Sleep mode, then wait for acknowledgement
  stm32_setenable(priv->base, 1);

  // Request Init mode, then wait for completion
  stm32_setinit(priv->base, 1);

  // Configuration Changes Enable.  Can only be set during Init mode;
  // cleared when INIT bit is cleared.
  stm32_setconfig(priv->base, 1);

  // Disable interrupts while we configure the hardware
  putreg32(0, priv->base + STM32_FDCAN_IE_OFFSET);

	/*
	 * CAN timings for this bitrate
	 */

  /// JACOB: TODO: Relocate...?  Check this is done properly
  struct fdcan_timeseg arbi_timing;
# ifdef CONFIG_NET_CAN_CANFD
  #error "CAN FD not yet supported for STM32H7"
  priv->arbi_timing.bitrate = CONFIG_FDCAN1_ARBI_BITRATE;
  priv->arbi_timing.samplep = CONFIG_FDCAN1_ARBI_SAMPLEP;
  priv->data_timing.bitrate = CONFIG_FDCAN1_DATA_BITRATE;
  priv->data_timing.samplep = CONFIG_FDCAN1_DATA_SAMPLEP;
# else
  arbi_timing.bitrate = CONFIG_FDCAN1_BITRATE;
  arbi_timing.samplep = CONFIG_FDCAN1_SAMPLEP;
# endif
  /// TODO: shoud this be here, or elsewhere?
  stm32_bitratetotimeseg(&arbi_timing, 1, 0); // default 1-tseg tolerance; non-FD

	if (timings_res < 0) {
		stm32_setinit(priv->base, 0);
		return timings_res;
	}

	// ninfo("Timings: presc=%u sjw=%u bs1=%u bs2=%u\r\n",
	// 		   unsigned(timings.prescaler), unsigned(timings.sjw), unsigned(timings.bs1), unsigned(timings.bs2));

	/*
	 * Set bit timings and prescalers (Nominal and Data bitrates)
	 */

	//  We're not using CAN-FD (yet), so set same timings for both
  regval = ((atbi_timing.sjw << FDCAN_NBTP_NSJW_SHIFT)  |
		        (arbi_timing.pseg1 << FDCAN_NBTP_NTSEG1_SHIFT) |
		        (arbi_timing.pseg2 << FDCAN_NBTP_TSEG2_SHIFT)  |
		        (arbi_timing.prescaler << FDCAN_NBTP_NBRP_SHIFT),
  putreg32(regval, priv->base + STM32_FDCAN_NBTP_OFFSET);
	putreg32(regval, priv->base + STM32_FDCAN_DBTP_OFFSET);

	/*
	 * Operation Configuration
	 */

	// Disable CAN-FD communications ("classic" CAN only)
  /// JACOB: TODO: Enable CAN FD based on CONFIG
  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_FDOE, 0);

	// Disable Time Triggered (TT) operation -- TODO (must use TTCAN_TypeDef)
	//ttcan_->TTOCF &= ~FDCAN_TTOCF_OM

	/*
	 * Configure Interrupts
	 */

	// Clear all interrupt flags
	// Note: A flag is cleared by writing a 1 to the corresponding bit position
  putreg32(0xFFFFFFFF, priv->base + STM32_FDCAN_IR_OFFSET);

	// Enable relevant interrupts
  regval = FDCAN_IE_TCE     // Transmit Complete
		     | FDCAN_IE_RF0NE   // Rx FIFO 0 new message
		     | FDCAN_IE_RF0FE   // Rx FIFO 0 FIFO full
		     | FDCAN_IE_RF1NE   // Rx FIFO 1 new message
		     | FDCAN_IE_RF1FE;  // Rx FIFO 1 FIFO full
  putreg32(regval, priv->base + STM32_FDCAN_IE_OFFSET);

	// Keep Rx interrupts on Line 0; move Tx to Line 1
  // TC (Tx Complete) interrupt on line 1
  putreg32(FDCAN_ILS_TCL, priv->base + STM32_FDCAN_ILS_OFFSET);

	// Enable Tx buffer transmission interrupt
  putreg32(FDCAN_TXBTIE_TIE, priv->base + STM32_FDCAN_TXBTIE_OFFSET);

	// Enable both interrupt lines
  putreg32(FDCAN_ILE_EINT0 | FDCAN_ILE_EINT1, priv->base + STM32_FDCAN_ILE_OFFSET);

	/*
	 * Configure Message RAM
	 *
	 * The available 2560 words (10 kiB) of RAM are shared between both FDCAN
	 * interfaces. It is up to us to ensure each interface has its own non-
	 * overlapping region of RAM assigned to it by properly assignin the start and
	 * end addresses for all regions of RAM.
	 *
	 * We will give each interface half of the available RAM.
	 *
	 * Rx buffers are only used in conjunction with acceptance filters; we don't
	 * have any specific need for this, so we will only use Rx FIFOs.
	 *
	 * Each FIFO can hold up to 64 elements, where each element (for a classic CAN
	 * 2.0B frame) is up to 4 words long (8 bytes data + header bits)
	 *
	 * Let's make use of the full 64 FIFO elements for FIFO0.  We have no need to
	 * separate messages between FIFO0 and FIFO1, so ignore FIFO1 for simplicity.
	 *
	 * Note that the start addresses given to FDCAN are in terms of _words_, not
	 * bytes, so when we go to read/write to/from the message RAM, there will be a
	 * factor of 4 necessary in the address relative to the SA register values.
	 */

	// Location of this interface's message RAM - address in CPU memory address
	// and relative address (in words) used for configuration
	const uint32_t iface_ram_base = (2560 / 2) * priv->iface_idx;
	const uint32_t gl_ram_base = STM32_CANRAM_BASE;
	uint32_t ram_offset = iface_ram_base;

	// Standard ID Filters: Allow space for 128 filters (128 words)
	const uint8_t n_stdid = 128;
	message_ram_.StdIdFilterSA = gl_ram_base + ram_offset * WORD_LENGTH;

  regval  = n_stdid << FDCAN_SIDFC_LSS_SHIFT;
	regval |= ram_offset << FDCAN_SIDFC_FLSSA_SHIFT;
  putreg32(regval, priv->base + STM32_FDCAN_SIDFC_OFFSET);
	ram_offset += n_stdid;

	// Extended ID Filters: Allow space for 128 filters (128 words)
	const uint8_t n_extid = 128;
	message_ram_.ExtIdFilterSA = gl_ram_base + ram_offset * WORD_LENGTH;
  regval = n_extid << FDCAN_XIDFC_LSE_SHIFT;
	regval |= ram_offset << FDCAN_XIDFC_FLESA_SHIFT;
  putreg32(regval, priv->base + STM32_FDCAN_XIDFC_OFFSET);
	ram_offset += n_extid;

	// Set size of each element in the Rx/Tx buffers and FIFOs
  putreg32(0, priv->base + STM32_FDCAN_RXESC_OFFSET);  // 8 byte space for every element (Rx buffer, FIFO1, FIFO0)
  putreg32(0, priv->base + STM32_FDCAN_TXESC_OFFSET);  // 8 byte space for every element (Tx buffer)

  /// JACOB: TODO: Correct place for setting the tx, rx pointers?
  /// JACOB: TODO: Figure out how to setup all this for CAN FD frames
	const uint8_t n_rxfifo0 = 64;  // 64 elements max for RX FIFO0
  const uint8_t n_txqueue = 32;  // 32 elements max for TX queue

	// Set Rx FIFO0 size
  priv->rx = (struct mb_s *)(gl_ram_base + ram_offset * WORD_LENGTH);
  putreg32(ram_offset << FDCAN_RXF0C_F0SA_SHIFT, priv->base + STM32_FDCAN_RF0C_OFFSET);

  regval = ram_offset << FDCAN_RXF0C_F0SA_SHIFT;
  regval |= n_rxfifo0 << FDCAN_RXF0C_F0S_SHIFT;
  putreg32(regval, priv->base + STM32_FDCAN_RF0C_OFFSET);
	ram_offset += n_rxfifo0 * FIFO_ELEMENT_SIZE;

	// Set Tx queue size
  priv->tx = (struct mb_s *)(gl_ram_base + ram_offset * WORD_LENGTH);
  regval = n_txqueue << FDCAN_TXBC_TFQS_SHIFT
  regval |= FDCAN_TXBC_TFQM;  // Queue mode (vs. FIFO)
  regval |= ram_offset << FDCAN_TXBC_TBSA_SHIFT;
  putreg32(regval, priv->base + STM32_FDCAN_TXBC_OFFSET);

	/*
	 * Default filter configuration
	 *
	 * Accept all messages into Rx FIFO0 by default
	 */
  regval = getreg32(priv->base + STM32_FDCAN_GFC_OFFSET);
  regval &= ~FDCAN_GFC_ANFS;  // Accept non-matching stdid frames into FIFO0
  regval &= ~FDCAN_GFC_ANFE;  // Accept non-matching extid frames into FIFO0
  putreg32(regval, priv->base + STM32_FDCAN_GFC_OFFSET);

	/*
	 * Exit Initialization mode
	 */
   stm32_setinit(priv->base, 0);

	return 0;
}

/****************************************************************************
 * Function: stm32_reset
 *
 * Description:
 *   Put the EMAC in the non-operational, reset state
 *
 * Input Parameters:
 *   priv - Reference to the private FDCAN driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void stm32_reset(struct stm32_driver_s *priv)
{
  uint32_t regval;
  uint32_t i;

  /// JACOB: TODO: put hardware into reset / sleep mode

  /// JACOB: TODO: Configure all Rx / Tx message RAM here?
  /// NOTE: in NXP FlexCAN they call this from _initialize() after setting 
  /// clock src and enabling the hardware

  /* Initialize all MB rx and tx */

  for (i = 0; i < TOTALMBCOUNT; i++)
    {
      ninfo("MB %i %p\r\n", i, &priv->rx[i]);
      ninfo("MB %i %p\r\n", i, &priv->rx[i].id.w);
      priv->rx[i].cs.cs = 0x0;
      priv->rx[i].id.w = 0x0;
      priv->rx[i].data[0].w00 = 0x0;
      priv->rx[i].data[1].w00 = 0x0;
    }


  // Clear Tx mailbox entries in message RAM

  /* Filtering catchall */

  /// TODO: set default filters
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: stm32_netinitialize
 *
 * Description:
 *   Initialize the network controller and driver
 *
 * Input Parameters:
 *   intf - In the case where there are multiple transceivers, this value
 *          identifies which transceiver is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int stm32_netinitialize(int intf)
{
  struct stm32_driver_s *priv;
  int ret;

  switch (intf)
    {
#ifdef CONFIG_STM32_FDCAN1
    case 0:
      priv             = &g_fdcan0;
      memset(priv, 0, sizeof(struct stm32_driver_s));
      priv->base       = STM32_FDCAN1_BASE;
      priv->iface_idx  = 0;
      priv->config     = &stm32_fdcan0_config;

      /* Default bitrate configuration */

#  ifdef CONFIG_NET_CAN_CANFD
      priv->arbi_timing.bitrate = CONFIG_FDCAN1_ARBI_BITRATE;
      priv->arbi_timing.samplep = CONFIG_FDCAN1_ARBI_SAMPLEP;
      priv->data_timing.bitrate = CONFIG_FDCAN1_DATA_BITRATE;
      priv->data_timing.samplep = CONFIG_FDCAN1_DATA_SAMPLEP;
#  else
      priv->arbi_timing.bitrate = CONFIG_FDCAN1_BITRATE;
      priv->arbi_timing.samplep = CONFIG_FDCAN1_SAMPLEP;
#  endif
      break;
#endif

#ifdef CONFIG_STM32_FDCAN2
    case 1:
      priv             = &g_fdcan1;
      memset(priv, 0, sizeof(struct stm32_driver_s));
      priv->base       = STM32_FDCAN2_BASE;
      priv->iface_idx  = 1;
      priv->config     = &stm32_fdcan1_config;

      /* Default bitrate configuration */

#  ifdef CONFIG_NET_CAN_CANFD
      priv->arbi_timing.bitrate = CONFIG_FDCAN2_ARBI_BITRATE;
      priv->arbi_timing.samplep = CONFIG_FDCAN2_ARBI_SAMPLEP;
      priv->data_timing.bitrate = CONFIG_FDCAN2_DATA_BITRATE;
      priv->data_timing.samplep = CONFIG_FDCAN2_DATA_SAMPLEP;
#  else
      priv->arbi_timing.bitrate = CONFIG_FDCAN2_BITRATE;
      priv->arbi_timing.samplep = CONFIG_FDCAN2_SAMPLEP;
#  endif
      break;
#endif

    default:
      return -ENODEV;
    }

  if (!stm32_bitratetotimeseg(&priv->arbi_timing, 1, 0))
    {
      nerr("ERROR: Invalid CAN timings: please try another sample point "
           "or refer to the reference manual\n");
      return -1;
    }

#ifdef CONFIG_NET_CAN_CANFD
  if (!stm32_bitratetotimeseg(&priv->data_timing, 1, 1))
    {
      nerr("ERROR: Invalid CAN data phase timings: please try another "
           "sample point or refer to the reference manual\n");
      return -1;
    }
#endif

  stm32_pinconfig(priv->config->tx_pin);
  stm32_pinconfig(priv->config->rx_pin);
  if (priv->config->enable_pin > 0)
    {
      stm32_pinconfig(priv->config->enable_pin);
      stm32_gpiowrite(priv->config->enable_pin, priv->config->enable_high);
    }

  /* Attach the fdcan interrupt handler */

  if (irq_attach(priv->config->mb_irq[0], stm32_fdcan_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach CAN RX IRQ\n");
      return -EAGAIN;
    }


  if (irq_attach(priv->config->mb_irq[1], stm32_fdcan_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      nerr("ERROR: Failed to attach CAN TX IRQ\n");
      return -EAGAIN;
    }

  /* Initialize the driver structure */

  priv->dev.d_ifup    = stm32_ifup;      /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = stm32_ifdown;    /* I/F down callback */
  priv->dev.d_txavail = stm32_txavail;   /* New TX data callback */
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = stm32_ioctl;     /* Support CAN ioctl() calls */
#endif
  priv->dev.d_private = (void *)priv;      /* Used to recover private state from dev */

#ifdef TX_TIMEOUT_WQ
  for (int i = 0; i < TXMBCOUNT; i++)
    {
      priv->txtimeout[i] = wd_create();    /* Create TX timeout timer */
    }

#endif
  /// JACOB TODO: I set these later when I configure the message RAM for the interface.
  ///             Is that okay?
  priv->rx = NULL;
  priv->tx = NULL;

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling stm32_ifdown().
   */

  ninfo("callbacks done\r\n");

  stm32_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->dev, NET_LL_CAN);

  UNUSED(ret);
  return OK;
}

/****************************************************************************
 * Name: up_netinitialize
 *
 * Description:
 *   Initialize the first network interface.  If there are more than one
 *   interface in the chip, then board-specific logic will have to provide
 *   this function to determine which, if any, Ethernet controllers should
 *   be initialized.
 *
 ****************************************************************************/

#if !defined(CONFIG_NETDEV_LATEINIT)
void up_netinitialize(void)
{
#ifdef CONFIG_STM32H7_FDCAN1
  stm32_netinitialize(0);
#endif

#ifdef CONFIG_STM32H7_FDCAN2
  stm32_netinitialize(1);
#endif
}
#endif

#endif /* CONFIG_STM32H7_FDCAN */
