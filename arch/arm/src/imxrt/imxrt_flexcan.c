/****************************************************************************
 * arch/arm/src/imxrt/imxrt_flexcan.c
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

#include <barriers.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/can.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/spinlock.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/can.h>

#include "arm_internal.h"
#include "chip.h"
#include "imxrt_config.h"
#include "imxrt_flexcan.h"
#include "imxrt_periphclks.h"
#include "hardware/imxrt_flexcan.h"
#include "hardware/imxrt_pinmux.h"
#include "hardware/imxrt_ccm.h"

#include <arch/board/board.h>

#include <sys/time.h>

#ifdef CONFIG_IMXRT_FLEXCAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If processing is not done at the interrupt level, then work queue support
 * is required.
 */

#define CANWORK    LPWORK
#define CANRCVWORK HPWORK

/* Special address description flags for the CAN_ID */

#define CAN_EFF_FLAG 0x80000000  /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000  /* Remote transmission request */
#define CAN_ERR_FLAG 0x20000000  /* Error message frame */
#define CAN_EVT_FLAG 0x10000000  /* Lower_half use this flags to report state switch event */

/* Valid bits in CAN ID for frame formats */

#define CAN_SFF_MASK 0x000007ff  /* Standard frame format (SFF) */
#define CAN_EFF_MASK 0x1fffffff  /* Extended frame format (EFF) */
#define CAN_ERR_MASK 0x1fffffff  /* Omit EFF, RTR, ERR flags */

/* CAN MB TX & RX codes */

#define CAN_RXMB_INACTIVE          0x0        /* MB is inactive */
#define CAN_RXMB_FULL              0x2        /* MB is full */
#define CAN_RXMB_EMPTY             0x4        /* MB is empty */
#define CAN_RXMB_OVERRUN           0x6        /* overrun */
#define CAN_RXMB_BUSY_BIT          0x1        /* BUSY, orred with any of the above */

/* CAN MB TX codes */

#define CAN_TXMB_INACTIVE          0x8        /* MB is not active. */
#define CAN_TXMB_ABORT             0x9        /* MB is aborted. */
#define CAN_TXMB_DATAORREMOTE      0xC        /* MB is a TX Data Frame(when MB RTR = 0) or */
                                              /* MB is a TX Remote Request Frame (when MB RTR = 1). */
#define CAN_TXMB_TANSWER           0xE        /* MB is a TX Response Request Frame from */
                                              /* an incoming Remote Request Frame. */

/* CAN MB CS fields (1st 32-bit word of header) */

#define CAN_MB_CS_TIMESTAMP_SHIFT  0          /* Free-Running Counter Timestamp */
#define CAN_MB_CS_TIMESTAMP_MASK   (0xffff << CAN_MB_CS_TIMESTAMP_SHIFT)
#define CAN_MB_CS_TIMESTAMP(x)     (((x) & CAN_MB_CS_TIMESTAMP_MASK) >> CAN_MB_CS_TIMESTAMP_SHIFT)
#define CAN_MB_CS_DLC_SHIFT 16                /* Length of Data in Bytes */
#define CAN_MB_CS_DLC_MASK         (0xf << CAN_MB_CS_DLC_SHIFT)
#define CAN_MB_CS_DLC(x)           (((x) & CAN_MB_CS_DLC_MASK) >> CAN_MB_CS_DLC_SHIFT)
#define CAN_MB_CS_RTR              (1 << 20)  /* Remote Transmission Request */
#define CAN_MB_CS_IDE              (1 << 21)  /* ID Extended Bit */
#define CAN_MB_CS_SSR              (1 << 22)  /* Substitute Remote Request */
                                              /* Bit 23: Reserved */
#define CAN_MB_CS_CODE_SHIFT       24         /* Message buffer code */
#define CAN_MB_CS_CODE_MASK        (0xf << CAN_MB_CS_CODE_SHIFT)
                                              /* Bit 28: Reserved */
#define CAN_MB_CS_CODE(x)          (((x) & CAN_MB_CS_CODE_MASK) >> CAN_MB_CS_CODE_SHIFT)
#define CAN_MB_CS_ESI              (1 << 29)  /* Error State Indicator */
#define CAN_MB_CS_BRS              (1 << 30)  /* Bit Rate Switch */
#define CAN_MB_CS_EDL              (1 << 31)  /* Extended Data Length */

/* CAN MB PRIO and ID fields (2nd 32-bit word of header) */

#define CAN_MB_ID_ID_SHIFT         0
#define CAN_MB_ID_ID_MASK          (0x1fffffff << CAN_MB_ID_ID_SHIFT)
#define CAN_MB_ID_ID_STD_SHIFT     18
#define CAN_MB_ID_ID_STD_MASK      (0x7ff << CAN_MB_ID_ID_STD_SHIFT)
#define CAN_MB_ID_PRIO_SHIFT       29
#define CAN_MB_ID_PRIO_MASK        (0x7 << CAN_MB_ID_PRIO_SHIFT)

#define RXMBCOUNT                   CONFIG_IMXRT_FLEXCAN_RXMB
#define TXMBCOUNT                   CONFIG_IMXRT_FLEXCAN_TXMB

#define MB_BIT(mbi)             (1u << (mbi))
#define MB_RANGE_MASK(first, count) \
  ((count) == 0 ? 0u : ((((uint32_t)1u << (count)) - 1u) << (first)))
#define ALIGN_UP(v, a)   (((v) + ((a) - 1)) & ~((a) - 1))

/* Only i.MXRT10XX are affected by ERR005829 */

#ifndef CONFIG_ARCH_FAMILY_IMXRT117x
#define FLEXCAN_ERR005829
#endif

#ifdef FLEXCAN_ERR005829
/* ERR005829 workaround:
 *
 * Reserve the first valid mailbox as INACTIVE. With RX FIFO disabled,
 * the errata requires this to be MB0.
 *
 * Layout with workaround enabled:
 *   MB0                  : reserved/inactive
 *   MB1 .. MB(RXMBCOUNT) : RX mailboxes
 *   remaining            : TX mailboxes
 */

#  define FLEXCAN_ERR005829_MB0_RESERVED 0
#  define RXMB_FIRST              (FLEXCAN_ERR005829_MB0_RESERVED + 1)
#  define RXMB_END                (RXMB_FIRST + RXMBCOUNT)   /* exclusive */
#  define TXMB_FIRST              RXMB_END
#  define TXMB_END                (TXMB_FIRST + TXMBCOUNT)   /* exclusive */
#  define TOTALMBCOUNT            TXMB_END
#  define IFLAG1_RX               MB_RANGE_MASK(RXMB_FIRST, RXMBCOUNT)
#  define IFLAG1_TX               MB_RANGE_MASK(TXMB_FIRST, TXMBCOUNT)
#if (CONFIG_IMXRT_FLEXCAN_RXMB + CONFIG_IMXRT_FLEXCAN_TXMB) > 13
# error Only 13 MB are allowed to be used
#endif

#else
/* Original layout without workaround:
 *   MB0 .. MB(RXMBCOUNT - 1) : RX mailboxes
 *   remaining                : TX mailboxes
 */
#  define RXMB_FIRST              0
#  define RXMB_END                RXMBCOUNT                   /* exclusive */
#  define TXMB_FIRST              RXMBCOUNT
#  define TXMB_END                (RXMBCOUNT + TXMBCOUNT)     /* exclusive */
#  define TOTALMBCOUNT            TXMB_END
#  define IFLAG1_RX               MB_RANGE_MASK(RXMB_FIRST, RXMBCOUNT)
#  define IFLAG1_TX               MB_RANGE_MASK(TXMB_FIRST, TXMBCOUNT)
#if (CONFIG_IMXRT_FLEXCAN_RXMB + CONFIG_IMXRT_FLEXCAN_TXMB) > 14
# error Only 14 MB are allowed to be used
#endif
#endif

#define MSG_DATA                    sizeof(struct timeval)

#define CLK_FREQ                    80000000

#define TSEG_MIN                    2

/* Classical can CTRL1 bit timing */

#define TSEG1_MAX                   16
#define TSEG2_MAX                   8
#define NUMTQ_MIN                   8
#define NUMTQ_MAX                   25

/* CAN FD CBT and FDCBT bit timing */

#define TSEG1_FD_MAX                96
#define TSEG2_FD_MAX                32
#define NUMTQ_FD_MIN                8
#define NUMTQ_FD_MAX                129

#define TSEG1_FD_DATAMAX            39
#define TSEG2_FD_DATAMAX            8
#define NUMTQ_FD_DATAMIN            5
#define NUMTQ_FD_DATAMAX            48

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE

#  if !defined(CONFIG_SCHED_WORKQUEUE)
#    error Work queue support is required
#  endif

#define TX_TIMEOUT_WQ
#endif

#ifdef CONFIG_NET_CAN_CANFD
#  define FRAME_SIZE   ALIGN_UP(sizeof(struct canfd_frame) + MSG_DATA, 4)
#  define TX_POOL_SIZE (FRAME_SIZE)
#  define RX_POOL_SIZE (FRAME_SIZE * RXMBCOUNT)
#else
#  define FRAME_SIZE   ALIGN_UP(sizeof(struct can_frame) + MSG_DATA, 4)
#  define TX_POOL_SIZE (FRAME_SIZE)
#  define RX_POOL_SIZE (FRAME_SIZE * RXMBCOUNT)
#endif

#define FRAME_SIZE_WORDS (FRAME_SIZE / 4)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mb_s
{
  volatile uint32_t cs;
  volatile uint32_t id;
  volatile uint32_t data[];
};

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
#define TX_ABORT -1
#define TX_FREE 0
#define TX_BUSY 1

struct txmbstats
{
  struct timeval deadline;
  bool active;
};
#endif

/* FlexCAN Device hardware configuration */

struct flexcan_timeseg
{
  uint32_t bitrate;
  int32_t samplep;
  uint8_t propseg;
  uint8_t pseg1;
  uint8_t pseg2;
  uint8_t presdiv;
};

/* The imxrt_driver_s encapsulates all state information for a single
 * hardware interface
 */

struct imxrt_driver_s
{
  struct net_driver_s dev;            /* Interface understood by the network */

  const uintptr_t base;               /* FLEXCAN base address */
  const int irq;                      /* irq number */
  const bool canfd_capable;
  const uint32_t *txdesc;             /* A pointer to the list of TX descriptor */
  const uint32_t *rxdesc;             /* A pointer to the list of RX descriptors */
  const bool srxdis;                  /* Self reception disable */

  uint32_t clk_freq;                  /* Peripheral clock frequency */
  bool bifup;                         /* true:ifup false:ifdown */
#ifdef TX_TIMEOUT_WQ
  struct wdog_s txtimeout[TXMBCOUNT]; /* TX timeout timer */
#endif
  struct work_s rcvwork;              /* For deferring recv work to the wq */
  struct work_s irqwork;              /* For deferring interrupt work to the wq */
  struct work_s timeoutwork;          /* For deferring timeout work to the wq */
  struct flexcan_timeseg arbi_timing; /* Timing for arbitration phase */
  struct flexcan_timeseg data_timing; /* Timing for data phase */

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  struct txmbstats txmb[TXMBCOUNT];
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_IMXRT_FLEXCAN1

static uint32_t g_tx_pool_can1[(TX_POOL_SIZE) / 4];
static uint32_t g_rx_pool_can1[(RX_POOL_SIZE) / 4];

static struct imxrt_driver_s g_flexcan1 =
  {
    .base              = IMXRT_CAN1_BASE,
    .irq               = IMXRT_IRQ_CAN1,
#  if defined(CONFIG_NET_CAN_CANFD) && defined(CONFIG_IMXRT_FLEXCAN1_FD)
    .canfd_capable     = true,
#  else
    .canfd_capable     = false,
#  endif

    /* Default bitrate configuration */

#  if defined(CONFIG_NET_CAN_CANFD) && defined(CONFIG_IMXRT_FLEXCAN1_FD)
    .arbi_timing =
      {
        .bitrate = CONFIG_FLEXCAN1_ARBI_BITRATE,
        .samplep = CONFIG_FLEXCAN1_ARBI_SAMPLEP,
      },
    .data_timing =
      {
        .bitrate = CONFIG_FLEXCAN1_DATA_BITRATE,
        .samplep = CONFIG_FLEXCAN1_DATA_SAMPLEP,
      },
#  else
    .arbi_timing =
      {
        .bitrate = CONFIG_FLEXCAN1_BITRATE,
        .samplep = CONFIG_FLEXCAN1_SAMPLEP,
      },
#  endif

    .txdesc = g_tx_pool_can1,
    .rxdesc = g_rx_pool_can1,

    .srxdis = true,
    .clk_freq = CLK_FREQ,
  };
#endif

#ifdef CONFIG_IMXRT_FLEXCAN2

static uint32_t g_tx_pool_can2[(TX_POOL_SIZE) / 4];
static uint32_t g_rx_pool_can2[(RX_POOL_SIZE) / 4];

static struct imxrt_driver_s g_flexcan2 =
  {
    .base              = IMXRT_CAN2_BASE,
    .irq               = IMXRT_IRQ_CAN2,
#  if defined(CONFIG_NET_CAN_CANFD) && defined(CONFIG_IMXRT_FLEXCAN2_FD)
    .canfd_capable     = true,
#  else
    .canfd_capable     = false,
#  endif

    /* Default bitrate configuration */

#  if defined(CONFIG_NET_CAN_CANFD) && defined(CONFIG_IMXRT_FLEXCAN2_FD)
    .arbi_timing =
      {
        .bitrate = CONFIG_FLEXCAN2_ARBI_BITRATE,
        .samplep = CONFIG_FLEXCAN2_ARBI_SAMPLEP,
      },
    .data_timing =
      {
        .bitrate = CONFIG_FLEXCAN2_DATA_BITRATE,
        .samplep = CONFIG_FLEXCAN2_DATA_SAMPLEP,
      },
#  else
    .arbi_timing =
      {
        .bitrate = CONFIG_FLEXCAN2_BITRATE,
        .samplep = CONFIG_FLEXCAN2_SAMPLEP,
      },
#  endif

    .txdesc = g_tx_pool_can2,
    .rxdesc = g_rx_pool_can2,

    .srxdis = true,
    .clk_freq = CLK_FREQ,
  };
#endif

#ifdef CONFIG_IMXRT_FLEXCAN3

static uint32_t g_tx_pool_can3[(TX_POOL_SIZE) / 4];
static uint32_t g_rx_pool_can3[(RX_POOL_SIZE) / 4];

static struct imxrt_driver_s g_flexcan3 =
  {
    .base              = IMXRT_CAN3_BASE,
    .irq               = IMXRT_IRQ_CAN3,
#  if defined(CONFIG_NET_CAN_CANFD)
    .canfd_capable     = true,
#  else
    .canfd_capable     = false,
#  endif

    /* Default bitrate configuration */

#  if defined(CONFIG_NET_CAN_CANFD)
    .arbi_timing =
      {
        .bitrate = CONFIG_FLEXCAN3_ARBI_BITRATE,
        .samplep = CONFIG_FLEXCAN3_ARBI_SAMPLEP,
      },
    .data_timing =
      {
        .bitrate = CONFIG_FLEXCAN3_DATA_BITRATE,
        .samplep = CONFIG_FLEXCAN3_DATA_SAMPLEP,
      },
#  else
    .arbi_timing =
      {
        .bitrate = CONFIG_FLEXCAN3_BITRATE,
        .samplep = CONFIG_FLEXCAN3_SAMPLEP,
      },
#  endif

    .txdesc = g_tx_pool_can3,
    .rxdesc = g_rx_pool_can3,

    .srxdis = true,
    .clk_freq = CLK_FREQ,
  };
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
 * Name: imxrt_bitratetotimeseg
 *
 * Description:
 *   Convert bitrate to timeseg
 *
 * Input Parameters:
 *   timeseg - structure to store bit timing
 *                  bit timings (recommended value: 1)
 *   can_fd_data - if set to calculate CAN FD data bit timings,
 *                  otherwise calculate classical or arbitration can
 *                  timings
 *
 * Returned Value:
 *   return OK on success, negated error number on failure
 *
 ****************************************************************************/

static uint32_t imxrt_bitratetotimeseg(struct imxrt_driver_s *priv,
                                      struct flexcan_timeseg *timeseg,
                                      bool can_fd_data)
{
#if defined(CONFIG_NET_CAN_CANFD)
  /* Max SEG1 & SEG2 values in TQ */

  const int32_t TSEG1MAX = can_fd_data ? TSEG1_FD_DATAMAX : TSEG1_FD_MAX;
  const int32_t TSEG2MAX = can_fd_data ? TSEG2_FD_DATAMAX : TSEG2_FD_MAX;

  /* Min and max bit length in TQ */

  const int32_t NUMTQMIN = can_fd_data ? NUMTQ_FD_DATAMIN : NUMTQ_FD_MIN;
  const int32_t NUMTQMAX = can_fd_data ? NUMTQ_FD_DATAMAX : NUMTQ_FD_MAX;

  /* Max register field values */

  /* Max register field value for presdiv */

  const uint32_t PRESDIVMAX = can_fd_data ?
    CAN_FDCBT_FPRESDIV_MASK >> CAN_FDCBT_FPRESDIV_SHIFT :
    CAN_CBT_EPRESDIV_MASK >> CAN_CBT_EPRESDIV_SHIFT;

  /* Max register field values from PSEG and PROPSEG */

  const int32_t PSEGMAX = can_fd_data ?
    CAN_FDCBT_FPSEG1_MASK >> CAN_FDCBT_FPSEG1_SHIFT :
    CAN_CBT_EPSEG1_MASK >> CAN_CBT_EPSEG1_SHIFT;
  const int32_t PROPSEGMAX = can_fd_data ?
    (CAN_FDCBT_FPROPSEG_MASK >> CAN_FDCBT_FPROPSEG_SHIFT) - 1 :
    CAN_CBT_EPROPSEG_MASK >> CAN_CBT_EPROPSEG_SHIFT;
#else
  /* Max SEG1 & SEG2 values in TQ */

  const int32_t TSEG1MAX = TSEG1_MAX;
  const int32_t TSEG2MAX = TSEG2_MAX;

  /* Min and max bit length in TQ */

  const int32_t NUMTQMIN = NUMTQ_MIN;
  const int32_t NUMTQMAX = NUMTQ_MAX;

  /* Max register field values */

  /* Max register field value for presdiv */

  const uint32_t PRESDIVMAX =
    CAN_CTRL1_PRESDIV_MASK >> CAN_CTRL1_PRESDIV_SHIFT;

  /* Max register field values from PSEG and PROPSEG */

  const int32_t PSEGMAX = CAN_CTRL1_PSEG1_MASK >> CAN_CTRL1_PSEG1_SHIFT;
  const int32_t PROPSEGMAX =
    CAN_CTRL1_PROPSEG_MASK >> CAN_CTRL1_PROPSEG_SHIFT;
#endif

  int32_t presdiv = PRESDIVMAX;
  int32_t tmppresdiv;
  int32_t numtq;
  int32_t tmpnumtq;
  int32_t tmpsample;
  int32_t tseg1;
  int32_t tseg2;
  int32_t tmppseg1;
  int32_t tmppseg2;
  int32_t tmppropseg;
  int32_t bitrate = 0;

  int32_t bitrate_tmp = 0;
  int32_t bitrate_err = INT32_MAX;

  for (tmppresdiv = 0; tmppresdiv < PRESDIVMAX; tmppresdiv++)
    {
      tmpnumtq = (priv->clk_freq / ((tmppresdiv + 1) * timeseg->bitrate));

      /* if number of time quanta per bit is too high, continue */

      if (tmpnumtq > NUMTQMAX)
        {
          continue;
        }

      /* if number of time quanta per bit is too small, break out */

      if (tmpnumtq < NUMTQMIN)
        {
          break;
        }

      bitrate_tmp = priv->clk_freq / ((tmppresdiv + 1) * tmpnumtq);
      if (abs(bitrate - bitrate_tmp) < bitrate_err)
        {
          bitrate_err = abs(bitrate - bitrate_tmp);
          bitrate = bitrate_tmp;
          numtq = tmpnumtq;
          presdiv = tmppresdiv;
        }
    }

  if (bitrate != timeseg->bitrate)
    {
      canwarn("bitrate set to %" PRId32 " instead of %" PRId32 "\n",
              bitrate, timeseg->bitrate);
    }

  /* Compute time segments based on the value of the sampling point */

  tseg1 = (numtq * timeseg->samplep / 100) - 1;
  tseg2 = numtq - 1 - tseg1;

  /* Adjust time segment 1 and time segment 2 */

  while (tseg1 >= TSEG1MAX || tseg2 < TSEG_MIN)
    {
      tseg2++;
      tseg1--;
    }

  if (tseg1 > TSEG1MAX || tseg2 > TSEG2MAX ||
      tseg2 < TSEG_MIN || tseg1 < TSEG_MIN)
    {
      canerr("tseg1 %" PRId32 ", max %" PRId32 "\n", tseg1, TSEG1MAX);
      canerr("tseg2 %" PRId32 ", max %" PRId32 "\n", tseg2, TSEG2MAX);
      return -EINVAL;
    }

  DEBUGASSERT(1 + tseg1 + tseg2 == numtq);

  tmppseg2 = tseg2 - 1;

  /* Start from pseg1 = pseg2 and adjust until propseg is valid */

  tmppseg1 = tmppseg2;
  tmppropseg = tseg1 - tmppseg1 - 2;

  while (tmppropseg <= 0)
    {
      tmppropseg++;
      tmppseg1--;
    }

  while (tmppropseg >= PROPSEGMAX)
    {
      tmppropseg--;
      tmppseg1++;
    }

  if (tmppseg1 > PSEGMAX || tmppseg2 > PSEGMAX)
    {
      canerr("tmppseg1 %" PRId32 ", max %" PRId32 "\n", tmppseg1, PSEGMAX);
      canerr("tmppseg2 %" PRId32 ", max %" PRId32 "\n", tmppseg2, PSEGMAX);
      return -EINVAL;
    }

  tmpsample = (1 + tseg1) * 100 / numtq;

  /* Allow 10% tolerance in sample point */

  if (abs(tmpsample - timeseg->samplep) <= 10)
    {
      if (can_fd_data)
        {
          timeseg->propseg = tmppropseg + 1;
        }
      else
        {
          timeseg->propseg = tmppropseg;
        }

      timeseg->pseg1 = tmppseg1;
      timeseg->pseg2 = tmppseg2;
      timeseg->presdiv = presdiv;
      timeseg->samplep = tmpsample;

      return OK;
    }

  canerr("sample point %" PRId32 ", configured %" PRId32 "\n",
         tmpsample, timeseg->samplep);

  return -EINVAL;
}

/* Common TX logic */

static bool imxrt_txringfull(struct imxrt_driver_s *priv);
static int  imxrt_transmit(struct imxrt_driver_s *priv);
static int  imxrt_txpoll(struct net_driver_s *dev);

/* Helper functions */

static bool imxrt_setenable(uint32_t base, bool enable);
static bool imxrt_setfreeze(uint32_t base, bool freeze);
static bool imxrt_waitmcr_change(uint32_t base,
                                uint32_t mask,
                                bool target_state);
static volatile struct mb_s *flexcan_get_mb(struct imxrt_driver_s *priv,
                                            int mbi);
static inline bool imxrt_txmb_active(volatile struct mb_s *mb);
static inline unsigned int imxrt_txmb_index(unsigned int mbi);
static int  imxrt_next_tx_mbi(struct imxrt_driver_s *priv);
#ifdef TX_TIMEOUT_WQ
static void imxrt_deadline_clear(struct txmbstats *txmb);
static bool imxrt_deadline_expired(const struct txmbstats *txmb,
                                   const struct timeval *now);
#endif

/* Interrupt handling */

static void imxrt_rxdone_work(void *arg);
static void imxrt_receive(struct imxrt_driver_s *priv);
static void imxrt_txdone_work(void *arg);
static void imxrt_txdone(struct imxrt_driver_s *priv, uint32_t flags);

static int  imxrt_flexcan_interrupt(int irq, void *context,
                                   void *arg);

/* Watchdog timer expirations */
#ifdef TX_TIMEOUT_WQ
static void imxrt_txtimeout_work(void *arg);
static void imxrt_txtimeout_expiry(wdparm_t arg);
#endif

/* NuttX callback functions */

static int  imxrt_ifup(struct net_driver_s *dev);
static int  imxrt_ifdown(struct net_driver_s *dev);

static int  imxrt_txavail(struct net_driver_s *dev);

#ifdef CONFIG_NETDEV_IOCTL
static int  imxrt_ioctl(struct net_driver_s *dev, int cmd,
                       unsigned long arg);
#endif

/* CAN ID filtering */

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
static uint32_t imxrt_add_filter(struct imxrt_driver_s *priv,
                                 uint8_t filter_type,
                                 bool ext_id,
                                 uint32_t filter_id1,
                                 uint32_t filter_id2);
static uint8_t imxrt_reset_filter(struct imxrt_driver_s *priv);
#endif

/* Initialization */

static int  imxrt_initialize(struct imxrt_driver_s *priv);
static void imxrt_reset(struct imxrt_driver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: imxrt_txringfull
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

static inline bool imxrt_txmb_active(volatile struct mb_s *mb)
{
  uint32_t code = CAN_MB_CS_CODE(mb->cs);

  return code == CAN_TXMB_DATAORREMOTE || code == CAN_TXMB_TANSWER;
}

static inline unsigned int imxrt_txmb_index(unsigned int mbi)
{
  DEBUGASSERT(mbi >= TXMB_FIRST && mbi < TXMB_END);
  return mbi - TXMB_FIRST;
}

static int imxrt_next_tx_mbi(struct imxrt_driver_s *priv)
{
  int mbi;

  /* With CTRL1[LBUF]=1, the lowest-numbered active TX mailbox wins the
   * internal FlexCAN arbitration.  To preserve software FIFO order, new
   * frames must therefore always be inserted strictly behind the newest
   * outstanding frame instead of reusing a lower-numbered hole.
   */

  for (mbi = TXMB_END - 1; mbi >= TXMB_FIRST; mbi--)
    {
      if (imxrt_txmb_active(flexcan_get_mb(priv, mbi)))
        {
          return (mbi + 1 < TXMB_END) ? (mbi + 1) : -1;
        }
    }

  return TXMB_FIRST;
}

#ifdef TX_TIMEOUT_WQ
static void imxrt_deadline_clear(struct txmbstats *txmb)
{
  txmb->deadline.tv_sec  = 0;
  txmb->deadline.tv_usec = 0;
  txmb->active           = false;
}

static bool imxrt_deadline_expired(const struct txmbstats *txmb,
                                   const struct timeval *now)
{
  if (!txmb->active)
    {
      return false;
    }

  if (now->tv_sec > txmb->deadline.tv_sec)
    {
      return true;
    }

  if (now->tv_sec < txmb->deadline.tv_sec)
    {
      return false;
    }

  return now->tv_usec >= txmb->deadline.tv_usec;
}
#endif

static bool imxrt_txringfull(struct imxrt_driver_s *priv)
{
  return imxrt_next_tx_mbi(priv) < 0;
}

#ifdef FLEXCAN_ERR005829
/* ERR005829 workaround: reserve MB0 as permanently inactive and write the
 * INACTIVE code twice. This helper is called during initialization and after
 * arming each TX mailbox.
 */

static inline void imxrt_err005829_kick(struct imxrt_driver_s *priv)
{
  volatile struct mb_s *mb = flexcan_get_mb(priv,
                             FLEXCAN_ERR005829_MB0_RESERVED);
  mb->cs = CAN_TXMB_INACTIVE << CAN_MB_CS_CODE_SHIFT;
  mb->cs = CAN_TXMB_INACTIVE << CAN_MB_CS_CODE_SHIFT;
}
#endif

/****************************************************************************
 * Function: imxrt_transmit
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

static int imxrt_transmit(struct imxrt_driver_s *priv)
{
  volatile struct mb_s *mb;
  int mbi;
  uint32_t *frame_data_word;
  uint32_t i;
#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  int32_t timeout;
  unsigned int txmb;
#endif
  uint32_t cs = 0;
  canid_t can_id;
  uint32_t can_dlc;
  uint8_t len;

  mbi = imxrt_next_tx_mbi(priv);
  if (mbi < 0)
    {
      nwarn("No TX MB available\n");
      NETDEV_TXERRORS(&priv->dev);
      return -EBUSY;
    }

  mb = flexcan_get_mb(priv, mbi);

#ifdef CONFIG_NET_CAN_RAW_TX_DEADLINE
  txmb = imxrt_txmb_index((unsigned int)mbi);

  struct timespec ts;
  clock_systime_timespec(&ts);

  if (priv->dev.d_sndlen > priv->dev.d_len)
    {
      struct timeval *tv =
             (struct timeval *)(priv->dev.d_buf + priv->dev.d_len);

      priv->txmb[txmb].deadline = *tv;
      priv->txmb[txmb].active   = true;
      timeout  = (tv->tv_sec - ts.tv_sec)*CLK_TCK
                 + ((tv->tv_usec - ts.tv_nsec / 1000)*CLK_TCK) / 1000000;
      if (timeout < -1)
        {
          imxrt_deadline_clear(&priv->txmb[txmb]);
          return -ETIMEDOUT;
        }
    }
  else
    {
      /* Default TX deadline defined in NET_CAN_RAW_DEFAULT_TX_DEADLINE */

      if (CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE > 0)
        {
          timeout = ((CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE / 1000000)
              *CLK_TCK);
          priv->txmb[txmb].deadline.tv_sec = ts.tv_sec +
              CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE / 1000000;
          priv->txmb[txmb].deadline.tv_usec = (ts.tv_nsec / 1000) +
              CONFIG_NET_CAN_RAW_DEFAULT_TX_DEADLINE % 1000000;
        }
      else
        {
          imxrt_deadline_clear(&priv->txmb[txmb]);
          timeout = -1;
        }
    }
#endif

  if (priv->dev.d_len == sizeof(struct can_frame))
    {
      struct can_frame *frame = (struct can_frame *)priv->dev.d_buf;
      can_id = frame->can_id;
      len = 8;
      can_dlc = frame->can_dlc;
      frame_data_word = (uint32_t *)&frame->data[0];
    }
#ifdef CONFIG_NET_CAN_CANFD
  else
    {
      struct canfd_frame *frame = (struct canfd_frame *)priv->dev.d_buf;
      cs |= CAN_MB_CS_EDL;
      cs |= frame->flags & CANFD_BRS ? CAN_MB_CS_BRS : 0;
      can_id = frame->can_id;
      len = frame->len;
      can_dlc = len_to_can_dlc[len];
      frame_data_word = (uint32_t *)&frame->data[0];
    }
#endif

  if (can_id & CAN_EFF_FLAG)
    {
      cs |= CAN_MB_CS_IDE;
      mb->id = can_id & CAN_MB_ID_ID_MASK;
    }
  else
    {
      mb->id = ((can_id & CAN_SFF_MASK) << CAN_MB_ID_ID_STD_SHIFT) &
        CAN_MB_ID_ID_STD_MASK;
    }

  cs |= (can_id & CAN_RTR_FLAG) ? CAN_MB_CS_RTR : 0;
  cs |= (can_dlc << CAN_MB_CS_DLC_SHIFT) & CAN_MB_CS_DLC_MASK;

  for (i = 0; i < (len + 4 - 1) / 4; i++)
    {
      mb->data[i] = __builtin_bswap32(frame_data_word[i]);
    }

  /* Go */

  cs |= CAN_TXMB_DATAORREMOTE << CAN_MB_CS_CODE_SHIFT;
  mb->cs = cs;

#ifdef FLEXCAN_ERR005829
  /* ERR005829: write INACTIVE twice to the reserved first valid mailbox
   * after arming the TX mailbox.
   */

  imxrt_err005829_kick(priv);
#endif

  /* Increment statistics */

  NETDEV_TXPACKETS(&priv->dev);

#ifdef TX_TIMEOUT_WQ
  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  if (timeout > 0)
    {
      wd_start(&priv->txtimeout[txmb], timeout + 1,
               imxrt_txtimeout_expiry, (wdparm_t)priv);
    }
#endif

  modifyreg32(priv->base + IMXRT_CAN_IMASK1_OFFSET, 0, MB_BIT(mbi));

  return OK;
}

/****************************************************************************
 * Function: imxrt_txpoll
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

static int imxrt_txpoll(struct net_driver_s *dev)
{
  struct imxrt_driver_s *priv = (struct imxrt_driver_s *)dev;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (dev->d_len > 0)
    {
      /* Send the packet */

      if (imxrt_transmit(priv) != OK)
        {
          return -EINVAL;
        }

      /* Check if there is room in the device to hold another packet. If
       * not, return a non-zero value to terminate the poll.
       */

      if (imxrt_txringfull(priv))
        {
          return -EBUSY;
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: imxrt_get_oldest_mbi
 *
 * Description:
 *   Find the oldest MB in the message buffers, based on timestamp
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *   flags - Bitmask of MBs which should be checked
 *
 * Returned Value:
 *   index of the MB with oldest received data
 *
 * Assumptions:
 *   Always called with at least one RX buffer to be checked
 *
 ****************************************************************************/

static inline bool flexcan_ts_is_older(uint16_t a, uint16_t b)
{
  /* True if a is older than b in 16-bit wrapping time.
   * Assumes the timestamps differ by less than 0x8000.
   */

  return (int16_t)(a - b) < 0;
}

static int imxrt_get_oldest_mbi(struct imxrt_driver_s *priv, uint32_t flags)
{
  int mbi;
  int oldest = -1;
  bool first = true;
  uint16_t t = 0;
  uint16_t t_oldest = 0;
  volatile struct mb_s *mb;

  DEBUGASSERT((flags & IFLAG1_RX) != 0);

  for (mbi = RXMB_FIRST; mbi < RXMB_END; mbi++)
    {
      if (flags & (1u << mbi))
        {
          mb = flexcan_get_mb(priv, mbi);
          t = CAN_MB_CS_TIMESTAMP(mb->cs);

          if (first || flexcan_ts_is_older(t, t_oldest))
            {
              first = false;
              t_oldest = t;
              oldest = mbi;
            }
        }
    }

  return oldest;
}

/****************************************************************************
 * Function: imxrt_receive
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

static void imxrt_receive(struct imxrt_driver_s *priv)
{
  int mbi;
  volatile struct mb_s *rf;
#ifdef CONFIG_NET_CAN_CANFD
  uint32_t *frame_data_word;
  uint32_t i;
#endif
  uint32_t cs;
  uint32_t packet_no = 0;
  uint8_t frame_len[RXMBCOUNT];

  uint32_t flags = getreg32(priv->base + IMXRT_CAN_IFLAG1_OFFSET);
  flags &= IFLAG1_RX;

  while (flags != 0)
    {
      mbi = imxrt_get_oldest_mbi(priv, flags);

      /* Make sure the MB is locked */

      rf = flexcan_get_mb(priv, mbi);

      do
        {
          cs = rf->cs;
        }
      while ((CAN_MB_CS_CODE(cs) & CAN_RXMB_BUSY_BIT) != 0);

      DEBUGASSERT(CAN_MB_CS_CODE(cs) != CAN_RXMB_EMPTY);

      if (CAN_MB_CS_CODE(cs) == CAN_RXMB_OVERRUN)
        {
          canwarn("RX overrun\n");
          NETDEV_RXERRORS(dev);
        }

      /* Read the frame contents */

#ifdef CONFIG_NET_CAN_CANFD
      if (cs & CAN_MB_CS_EDL)
        {
          /* CAN FD frame */

          struct canfd_frame *frame = (struct canfd_frame *)&priv->rxdesc
                                      [packet_no * FRAME_SIZE_WORDS];

          if (cs & CAN_MB_CS_IDE)
            {
              frame->can_id = ((rf->id & CAN_MB_ID_ID_MASK) >>
                               CAN_MB_ID_ID_SHIFT);
              frame->can_id |= CAN_EFF_FLAG;
            }
          else
            {
              frame->can_id = ((rf->id & CAN_MB_ID_ID_STD_MASK) >>
                               CAN_MB_ID_ID_STD_SHIFT);
            }

          if (cs & CAN_MB_CS_RTR)
            {
              frame->can_id |= CAN_RTR_FLAG;
            }

          /* Set bitrate switch by default if frame is CANFD */

          frame->flags = CANFD_BRS;
          if (cs & CAN_MB_CS_ESI)
            {
              frame->flags |= CANFD_ESI;
            }

          frame->len = can_dlc_to_len[CAN_MB_CS_DLC(cs)];

          frame_data_word = (uint32_t *)&frame->data[0];

          for (i = 0; i < (frame->len + 4 - 1) / 4; i++)
            {
              frame_data_word[i] = __builtin_bswap32(rf->data[i]);
            }

          frame_len[packet_no] = sizeof(struct canfd_frame);
        }
      else
#endif
        {
          /* CAN 2.0 Frame */

          struct can_frame *frame = (struct can_frame *)&priv->rxdesc
                                    [packet_no * FRAME_SIZE_WORDS];

          if (cs & CAN_MB_CS_IDE)
            {
              frame->can_id = ((rf->id & CAN_MB_ID_ID_MASK) >>
                               CAN_MB_ID_ID_SHIFT);
              frame->can_id |= CAN_EFF_FLAG;
            }
          else
            {
              frame->can_id = ((rf->id & CAN_MB_ID_ID_STD_MASK) >>
                               CAN_MB_ID_ID_STD_SHIFT);
            }

          if (cs & CAN_MB_CS_RTR)
            {
              frame->can_id |= CAN_RTR_FLAG;
            }

          frame->can_dlc = CAN_MB_CS_DLC(cs);

          *(uint32_t *)&frame->data[0] = __builtin_bswap32(rf->data[0]);
          *(uint32_t *)&frame->data[4] = __builtin_bswap32(rf->data[1]);

          frame_len[packet_no] = sizeof(struct can_frame);
        }

      /* Clear MB interrupt flag */

      putreg32(1 << mbi, priv->base + IMXRT_CAN_IFLAG1_OFFSET);

      /* Read TIMER to unlock MB */

      (void)getreg32(priv->base + IMXRT_CAN_TIMER_OFFSET);

      /* Re-activate the buffer */

      rf->cs = (CAN_RXMB_EMPTY << CAN_MB_CS_CODE_SHIFT) | CAN_MB_CS_IDE;

      flags &= ~(1 << mbi);

      packet_no++;
    }

  net_lock();

  /* Dispatch processed packets to network stack */

  for (int i = 0; i < packet_no; i++)
    {
      /* Copy the buffer pointer to priv->dev..  Set amount of data
       * in priv->dev.d_len
       */

      priv->dev.d_buf = (uint8_t *)&priv->rxdesc[i * FRAME_SIZE_WORDS];
      priv->dev.d_len = frame_len[i];

      /* Send to socket interface */

      can_input(&priv->dev);
    }

  net_unlock();

  /* Re-enable RX interrupts */

  modifyreg32(priv->base + IMXRT_CAN_IMASK1_OFFSET, 0, IFLAG1_RX);
}

/****************************************************************************
 * Function: imxrt_txdone
 *
 * Description:
 *   Check transmit interrupt flags and clear them
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void imxrt_txdone(struct imxrt_driver_s *priv, uint32_t flags)
{
  volatile struct mb_s *mb;
  uint32_t mbi;
  uint32_t mb_bit;
#ifdef TX_TIMEOUT_WQ
  unsigned int txmb;
#endif
  int code;

  /* Process TX completions */

  for (mbi = TXMB_FIRST; mbi < TXMB_END; mbi++)
    {
      mb_bit = MB_BIT(mbi);
      if (flags & mb_bit)
        {
          mb = flexcan_get_mb(priv, mbi);
          code = CAN_MB_CS_CODE(mb->cs);

          /* Clear interrupt */

          putreg32(mb_bit, priv->base + IMXRT_CAN_IFLAG1_OFFSET);

          /* After RTR transmission, the MB transitions into RX MB.
           * Check for RX empty w. busy bit or RX full - this should
           * not happen.
           */

          if (((code & ~CAN_RXMB_BUSY_BIT) == CAN_RXMB_EMPTY &&
               (code & CAN_RXMB_BUSY_BIT) != 0) ||
              code == CAN_RXMB_FULL)
            {
              /* Received something in this buffer?
               * This should only happen if we sent RTR and then did
               * run out of RX MBs (which are at lower indices).
               * Or perhaps this shouldn't happen at all when AEN=1. This
               * is unclear in the RM.
               */

              NETDEV_RXDROPPED(priv->dev);
              canerr("RCV in TX MB, code %x\n", code);
            }

          /* Only possible TX codes after transmission are ABORT or
           * INACTIVE. If it transitioned to RX MB after RTR sent,
           * deactivate it.
           */

          if (code != CAN_TXMB_ABORT && code != CAN_TXMB_INACTIVE)
            {
              mb->cs = CAN_TXMB_INACTIVE << CAN_MB_CS_CODE_SHIFT;
            }

          if (code == CAN_TXMB_ABORT)
            {
              NETDEV_TXERRORS(dev);
            }
          else
            {
              NETDEV_TXDONE(dev);
            }

#ifdef TX_TIMEOUT_WQ
          txmb = imxrt_txmb_index(mbi);

          /* We are here because a transmission completed, so the
           * corresponding watchdog can be canceled
           */

          wd_cancel(&priv->txtimeout[txmb]);
          imxrt_deadline_clear(&priv->txmb[txmb]);
#endif
        }
    }

  /* Schedule worker to poll for more data */

  work_queue(CANWORK, &priv->irqwork, imxrt_txdone_work, priv, 0);
}

/****************************************************************************
 * Function: imxrt_txdone_work
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
 *   We are not in an interrupt context so that we can lock the network.
 *
 ****************************************************************************/

static void imxrt_txdone_work(void *arg)
{
  struct imxrt_driver_s *priv = (struct imxrt_driver_s *)arg;

  /* There should be space for a new TX in any event.  Poll the network for
   * new XMIT data
   */

  net_lock();
  devif_poll(&priv->dev, imxrt_txpoll);
  net_unlock();
}

/****************************************************************************
 * Function: imxrt_rxdone_work
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

static void imxrt_rxdone_work(void *arg)
{
  struct imxrt_driver_s *priv = (struct imxrt_driver_s *)arg;

  imxrt_receive(priv);
}

/****************************************************************************
 * Function: imxrt_flexcan_interrupt
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

static int imxrt_flexcan_interrupt(int irq, void *context,
                                     void *arg)
{
  struct imxrt_driver_s *priv = (struct imxrt_driver_s *)arg;

  if (irq == priv->irq)
    {
      uint32_t flags = getreg32(priv->base + IMXRT_CAN_IFLAG1_OFFSET);

      if (flags & IFLAG1_RX)
        {
          work_queue(CANRCVWORK, &priv->rcvwork, imxrt_rxdone_work, priv, 0);

          /* Mask RX interrupts until handled in the work queue */

          modifyreg32(priv->base + IMXRT_CAN_IMASK1_OFFSET,
                      IFLAG1_RX, 0);
        }

      if (flags & IFLAG1_TX)
        {
          imxrt_txdone(priv, flags);
        }
    }

  return OK;
}

/****************************************************************************
 * Function: imxrt_txtimeout_work
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

static void imxrt_txtimeout_work(void *arg)
{
  struct imxrt_driver_s *priv = (struct imxrt_driver_s *)arg;
  int mbi;
  uint32_t mb_bit;
  volatile struct mb_s *mb;
  uint32_t iflag1;
  struct timespec ts;
  struct timeval now;

  clock_systime_timespec(&ts);
  now.tv_sec = ts.tv_sec;
  now.tv_usec = ts.tv_nsec / 1000; /* timespec to timeval conversion */

  /* The watchdog timed out, yet we still check mailboxes in case the
   * transmit function transmitted a new frame
   */

  for (mbi = TXMB_FIRST; mbi < TXMB_END; mbi++)
    {
      unsigned int txmbi = imxrt_txmb_index(mbi);
      mb_bit = MB_BIT(mbi);

      /* Disable interrupt for this MB */

      modifyreg32(priv->base + IMXRT_CAN_IMASK1_OFFSET, mb_bit, 0);
      ARM_DSB();

      if (!imxrt_deadline_expired(&priv->txmb[txmbi], &now))
        {
          goto reenable_irq;
        }

      mb = flexcan_get_mb(priv, mbi);
      iflag1 = getreg32(priv->base + IMXRT_CAN_IFLAG1_OFFSET);

      /* If the TX completion flag is already pending, or the mailbox is no
       * longer in an active transmit state, then the transmission has
       * already completed or completion is pending and this is not a
       * real timeout.
       *
       * In that case, just clear the software deadline and let the normal
       * interrupt/completion path run when the interrupt is re-enabled.
       */

      if ((iflag1 & mb_bit) != 0 || !imxrt_txmb_active(mb))
        {
          imxrt_deadline_clear(&priv->txmb[txmbi]);
          goto reenable_irq;
        }

      /* This MB is still actively transmitting and its deadline really
       * expired, so abort it.
       */

      NETDEV_TXTIMEOUTS(&priv->dev);
      mb->cs = CAN_TXMB_ABORT << CAN_MB_CS_CODE_SHIFT;
      imxrt_deadline_clear(&priv->txmb[txmbi]);

reenable_irq:

      /* Re-enable interrupt for this MB */

      modifyreg32(priv->base + IMXRT_CAN_IMASK1_OFFSET, 0, mb_bit);
    }
}

/****************************************************************************
 * Function: imxrt_txtimeout_expiry
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Input Parameters:
 *   arg  - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void imxrt_txtimeout_expiry(wdparm_t arg)
{
  struct imxrt_driver_s *priv = (struct imxrt_driver_s *)arg;

  /* Schedule to perform the TX timeout processing on the worker thread */

  work_queue(CANWORK, &priv->timeoutwork, imxrt_txtimeout_work, priv, 0);
}

#endif

static bool imxrt_setenable(uint32_t base, bool enable)
{
  if (enable)
    {
      modifyreg32(base + IMXRT_CAN_MCR_OFFSET, CAN_MCR_MDIS, 0);
    }
  else
    {
      modifyreg32(base + IMXRT_CAN_MCR_OFFSET, 0, CAN_MCR_MDIS);
    }

  return imxrt_waitmcr_change(base, CAN_MCR_LPMACK, !enable);
}

static bool imxrt_setfreeze(uint32_t base, bool freeze)
{
  if (freeze)
    {
      modifyreg32(base + IMXRT_CAN_MCR_OFFSET, 0,
                  CAN_MCR_HALT | CAN_MCR_FRZ);
    }
  else
    {
      modifyreg32(base + IMXRT_CAN_MCR_OFFSET,
                  CAN_MCR_HALT | CAN_MCR_FRZ, 0);
    }

  return imxrt_waitmcr_change(base, CAN_MCR_FRZACK, freeze);
}

static bool imxrt_waitmcr_change(uint32_t base, uint32_t mask,
                                bool target_state)
{
  const uint32_t timeout = 1000;
  uint32_t wait_ack;
  bool state;

  for (wait_ack = 0; wait_ack < timeout; wait_ack++)
    {
      state = (getreg32(base + IMXRT_CAN_MCR_OFFSET) & mask) != 0;
      if (state == target_state)
        {
          return true;
        }

      up_udelay(10);
    }

  return false;
}

/****************************************************************************
 * Function: flexcan_get_mb
 *
 * Description:
 *   Get message buffer start address by buffer index. Message buffers
 *   are allocated in 512-byte ramblocks.
 *
 * Input Parameters:
 *   priv - Reference to the private FLEXCAN driver state structure
 *   mbi  - Message buffer index
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static volatile struct mb_s *flexcan_get_mb(struct imxrt_driver_s *priv,
                                            int mbi)
{
  uintptr_t mb_offset;
  size_t data_bytes = priv->canfd_capable ? 64 : 8;
  size_t mb_bytes = sizeof(struct mb_s) + data_bytes;
  int mbs_per_block = 512 / mb_bytes;          /* n of buffers in one ramblock */
  int ramblock = mbi / mbs_per_block;          /* ramblock in which the mb resides */
  int mb_off = mbi - ramblock * mbs_per_block; /* idx of the mb within ramblock */

  mb_offset = IMXRT_CAN_MB_OFFSET + (ramblock * 512) + mb_off * mb_bytes;

  DEBUGASSERT(mb_offset < IMXRT_CAN_MB_END);

  return (volatile struct mb_s *)(priv->base + mb_offset);
}

/****************************************************************************
 * Function: imxrt_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   return OK on success, negated error number on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int imxrt_ifup(struct net_driver_s *dev)
{
  struct imxrt_driver_s *priv = (struct imxrt_driver_s *)dev;

  if (imxrt_initialize(priv) != OK)
    {
      canerr("initialize failed");
      return -EIO;
    }

  priv->bifup = true;
  priv->dev.d_buf = (uint8_t *)priv->txdesc;

  /* Set interrupts */

  up_enable_irq(priv->irq);
  up_enable_irq(priv->irq + 1);

  netdev_carrier_on(dev);

  return OK;
}

/****************************************************************************
 * Function: imxrt_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   return OK on success, negated error number on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int imxrt_ifdown(struct net_driver_s *dev)
{
  struct imxrt_driver_s *priv = (struct imxrt_driver_s *)dev;

  /* Disable interrupts */

  up_disable_irq(priv->irq);
  up_disable_irq(priv->irq + 1);

  imxrt_reset(priv);

  priv->bifup = false;

  netdev_carrier_off(dev);

  return OK;
}

/****************************************************************************
 * Function: imxrt_txavail
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

static int imxrt_txavail(struct net_driver_s *dev)
{
  struct imxrt_driver_s *priv = (struct imxrt_driver_s *)dev;

  net_lock();

  if (priv->bifup && !imxrt_txringfull(priv))
    {
      devif_poll(&priv->dev, imxrt_txpoll);
    }

  net_unlock();
  return OK;
}

/****************************************************************************
 * Function: imxrt_ioctl
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

#ifdef CONFIG_NETDEV_IOCTL
static int imxrt_ioctl(struct net_driver_s *dev, int cmd,
                      unsigned long arg)
{
  int ret;

  switch (cmd)
    {
#ifdef CONFIG_NETDEV_CAN_BITRATE_IOCTL
      case SIOCGCANBITRATE: /* Get bitrate from a CAN controller */
        {
          struct imxrt_driver_s *priv = (struct imxrt_driver_s *)dev;
          struct can_ioctl_data_s *req =
              (struct can_ioctl_data_s *)((uintptr_t)arg);
          req->arbi_bitrate = priv->arbi_timing.bitrate;
          req->arbi_samplep = priv->arbi_timing.samplep;
          if (priv->canfd_capable)
            {
              req->data_bitrate = priv->data_timing.bitrate;
              req->data_samplep = priv->data_timing.samplep;
            }
          else
            {
              req->data_bitrate = 0;
              req->data_samplep = 0;
            }

          ret = OK;
        }
        break;

      case SIOCSCANBITRATE: /* Set bitrate of a CAN controller */
        {
          struct imxrt_driver_s *priv = (struct imxrt_driver_s *)dev;
          struct can_ioctl_data_s *req =
              (struct can_ioctl_data_s *)((uintptr_t)arg);
          struct flexcan_timeseg arbi_timing;
          struct flexcan_timeseg data_timing;

          arbi_timing.bitrate = req->arbi_bitrate;
          arbi_timing.samplep = req->arbi_samplep;
          ret = imxrt_bitratetotimeseg(priv, &arbi_timing, false);
          if (ret == OK && priv->canfd_capable)
            {
              data_timing.bitrate = req->data_bitrate;
              data_timing.samplep = req->data_samplep;
              ret = imxrt_bitratetotimeseg(priv, &data_timing, true);
            }

          if (ret == OK)
            {
              /* Apply the new timings (interface is guaranteed to be down) */

              priv->arbi_timing = arbi_timing;
              if (priv->canfd_capable)
              {
                priv->data_timing = data_timing;
              }
            }
        }
        break;
#endif

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
      case SIOCACANSTDFILTER: /* Set STD ID CAN filter */
        {
          struct imxrt_driver_s *priv = (struct imxrt_driver_s *)dev;
          struct can_ioctl_filter_s *req =
            (struct can_ioctl_filter_s *)((uintptr_t)arg);
          if (!req)
            {
              return -EINVAL;
            }

            ret = imxrt_add_filter(priv, req->ftype, 0,
                                   req->fid1, req->fid2);
        }
        break;

      case SIOCACANEXTFILTER: /* Set EXT ID CAN filter */
        {
          struct imxrt_driver_s *priv = (struct imxrt_driver_s *)dev;
          struct can_ioctl_filter_s *req =
            (struct can_ioctl_filter_s *)((uintptr_t)arg);
          if (!req)
            {
              return -EINVAL;
            }

            ret = imxrt_add_filter(priv, req->ftype, 1,
                                   req->fid1, req->fid2);
        }
        break;

      case SIOCDCANSTDFILTER: /* Reset STD ID CAN filter */
      case SIOCDCANEXTFILTER: /* Reset EXT ID CAN filter */
        {
          struct imxrt_driver_s *priv = (struct imxrt_driver_s *)dev;
          ret = imxrt_reset_filter(priv);
        }
        break;
#endif

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}
#endif /* CONFIG_NETDEV_IOCTL */

/****************************************************************************
 * Name: imxrt_add_filter
 *
 * Description:
 *   Add new MB filter. Currently only support single ID mask filter.
 *   TODO: add support for multiple ID mask filters
 *
 * Input Parameters:
 *   priv          - Pointer to the private CAN driver state structure
 *   filter_type   - The type of the filter: mask, range or dual filter
 *   ext_id        - true: extended id, false: standard id
 *   filter_id1    - filter id 1 (refer to can_ioctl_filter_s in if.h)
 *   filter_id2    - filter id 2 (refer to can_ioctl_filter_s in if.h)
 *
 * Returned Value:
 *   return OK on success, negated error number on failure
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
static uint32_t imxrt_add_filter(struct imxrt_driver_s *priv,
                               uint8_t filter_type,
                               bool ext_id,
                               uint32_t filter_id1,
                               uint32_t filter_id2)
{
  volatile struct mb_s *mb;
  uint32_t mbi = 0;

  /* Enter freeze mode */

  if (!imxrt_setfreeze(priv->base, true))
    {
      canerr("FLEXCAN: freeze fail\n");
      return -EIO;
    }

  switch (filter_type)
    {
      case CAN_FILTER_MASK:
        {
          if (ext_id)
            {
              for (mbi = 0; mbi < RXMBCOUNT; mbi++)
                {
                  /* Set individual mask register */

                  putreg32(filter_id2 & CAN_MB_ID_ID_MASK,
                           priv->base + IMXRT_CAN_RXIMR_OFFSET(mbi));

                  /* Set the acceptance EXT ID filter in MB */

                  mb = flexcan_get_mb(priv, mbi);
                  mb->id = filter_id1 & CAN_MB_ID_ID_MASK;
                }
            }
          else
            {
              for (mbi = 0; mbi < RXMBCOUNT; mbi++)
                {
                  /* Set individual mask register */

                  putreg32(((filter_id2 & CAN_SFF_MASK)
                           << CAN_MB_ID_ID_STD_SHIFT)
                           & CAN_MB_ID_ID_STD_MASK,
                           priv->base + IMXRT_CAN_RXIMR_OFFSET(mbi));

                  /* Set the acceptance STD ID filter in MB */

                  mb = flexcan_get_mb(priv, mbi);
                  mb->id = ((filter_id1 & CAN_SFF_MASK)
                           << CAN_MB_ID_ID_STD_SHIFT)
                           & CAN_MB_ID_ID_STD_MASK;
                }
            }
        }
        break;

      case CAN_FILTER_RANGE:
        {
          canerr("Range filter type not supported\n");
          return -EINVAL;
        }
        break;

      case CAN_FILTER_DUAL:
        {
          canerr("Dual filter type not supported\n");
          return -EINVAL;
        }
        break;

      default:
        {
          canerr("FLEXCAN: invalid filter type\n");
          return -EINVAL;
        }
        break;
    }

  /* Exit freeze mode */

  if (!imxrt_setfreeze(priv->base, false))
    {
      canerr("FLEXCAN: unfreeze fail\n");
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: imxrt_reset_filter
 *
 * Description:
 *   Clear all ID filters in the MBs
 *
 * Input Parameters:
 *   priv - Reference to the private FLEXCAN driver state structure
 *
 * Returned Value:
 *   return OK on success, negated error number on failure
 *
 ****************************************************************************/

static uint8_t imxrt_reset_filter(struct imxrt_driver_s *priv)
{
  volatile struct mb_s *mb;
  uint32_t mbi = 0;

  /* Enter freeze mode */

  if (!imxrt_setfreeze(priv->base, true))
    {
      canerr("FLEXCAN: freeze fail\n");
      return -EIO;
    }

  for (mbi = 0; mbi < RXMBCOUNT; mbi++)
    {
      /* Clear individual mask register */

      putreg32(0, priv->base + IMXRT_CAN_RXIMR_OFFSET(mbi));

      /* clear the acceptance ID filter */

      mb = flexcan_get_mb(priv, mbi);
      mb->id = 0;
    }

  /* Exit freeze mode */

  if (!imxrt_setfreeze(priv->base, false))
    {
      canerr("FLEXCAN: unfreeze fail\n");
      return -EIO;
    }

  return OK;
}
#endif /* CONFIG_NETDEV_CAN_FILTER_IOCTL */

#ifdef CONFIG_IMXRT_FLEXCAN_ECC

/****************************************************************************
 * Function: imxrt_init_eccram
 *
 * Description:
 *   Initialize FLEXCAN ECC RAM
 *
 * Input Parameters:
 *   priv - Reference to the private FLEXCAN driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int imxrt_init_eccram(struct imxrt_driver_s *priv)
{
  uint32_t i;
  uint32_t regval;
  irqstate_t flags;

  flags = enter_critical_section();

  regval = getreg32(priv->base + IMXRT_CAN_CTRL2_OFFSET);

  /* Set WRMFRZ bit in CTRL2 Register to grant write access to memory */

  regval |= CAN_CTRL2_WRMFRZ;

  putreg32(regval, priv->base + IMXRT_CAN_CTRL2_OFFSET);

  for (i = IMXRT_CAN_MB_OFFSET; i < IMXRT_CAN_MB_END; i += 4)
    {
      putreg32(0, priv->base + i);
    }

  for (i = IMXRT_CAN_MB2_OFFSET; i < IMXRT_CAN_MB2_END; i += 4)
    {
      putreg32(0, priv->base + i);
    }

  regval = getreg32(priv->base + IMXRT_CAN_CTRL2_OFFSET);

  /* Clear WRMFRZ bit in CTRL2 Register */

  regval &= ~CAN_CTRL2_WRMFRZ;

  leave_critical_section(flags);

  return 0;
}

#endif

/****************************************************************************
 * Function: imxrt_initalize
 *
 * Description:
 *   Initialize FLEXCAN device
 *
 * Input Parameters:
 *   priv - Reference to the private FLEXCAN driver state structure
 *
 * Returned Value:
 *   return OK on success, negated error number on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int imxrt_initialize(struct imxrt_driver_s *priv)
{
  uint32_t tdcoff;
  int i;
  volatile struct mb_s *mb;

  /* Enable module */

  if (!imxrt_setenable(priv->base, true))
    {
      canerr("FLEXCAN: enable fail\n");
      return -EIO;
    }

  /* Enter freeze mode */

  if (!imxrt_setfreeze(priv->base, true))
    {
      canerr("FLEXCAN: freeze fail\n");
      return -EIO;
    }

  /* Initialize memory buffers */

#ifdef CONFIG_IMXRT_FLEXCAN_ECC
  imxrt_init_eccram(priv);
#endif

  /* Configure RX*MASKs at 0xaa0-> */

  putreg32(0x3fffffff, priv->base + IMXRT_CAN_RX14MASK_OFFSET);
  putreg32(0x3fffffff, priv->base + IMXRT_CAN_RX15MASK_OFFSET);
  putreg32(0x3fffffff, priv->base + IMXRT_CAN_RXMGMASK_OFFSET);
  putreg32(0x0, priv->base + IMXRT_CAN_RXFGMASK_OFFSET);

  /* Configure MCR */

  modifyreg32(priv->base + IMXRT_CAN_MCR_OFFSET,
              CAN_MCR_MAXMB_MASK | CAN_MCR_LPRIOEN,
              CAN_MCR_SLFWAK | CAN_MCR_WRNEN | CAN_MCR_WAKSRC |
              CAN_MCR_IRMQ | CAN_MCR_AEN |
              (((TOTALMBCOUNT - 1) << CAN_MCR_MAXMB_SHIFT) &
               CAN_MCR_MAXMB_MASK));

  if (priv->srxdis)
    {
      modifyreg32(priv->base + IMXRT_CAN_MCR_OFFSET, 0, CAN_MCR_SRXDIS);
    }

  if (!priv->canfd_capable)
    {
      modifyreg32(priv->base + IMXRT_CAN_CTRL1_OFFSET,
                  CAN_CTRL1_PRESDIV_MASK | CAN_CTRL1_PROPSEG_MASK |
                  CAN_CTRL1_PSEG1_MASK | CAN_CTRL1_PSEG2_MASK |
                  CAN_CTRL1_RJW_MASK,
                  CAN_CTRL1_PRESDIV(priv->arbi_timing.presdiv) | /* Prescaler divisor factor */
                  CAN_CTRL1_PROPSEG(priv->arbi_timing.propseg) | /* Propagation segment */
                  CAN_CTRL1_PSEG1(priv->arbi_timing.pseg1) |     /* Phase buffer segment 1 */
                  CAN_CTRL1_PSEG2(priv->arbi_timing.pseg2) |     /* Phase buffer segment 2 */
                  CAN_CTRL1_RJW(1));                             /* Resynchronization jump width */
    }
  else
    {
      modifyreg32(priv->base + IMXRT_CAN_CBT_OFFSET,
                  CAN_CBT_EPRESDIV_MASK | CAN_CBT_EPROPSEG_MASK |
                  CAN_CBT_EPSEG1_MASK | CAN_CBT_EPSEG2_MASK |
                  CAN_CBT_ERJW_MASK,
                  CAN_CBT_BTF |                                 /* Enable extended bit timing
                                                                 * configurations for CAN-FD for setting up
                                                                 * separately nominal and data phase */
                  CAN_CBT_EPRESDIV(priv->arbi_timing.presdiv) | /* Prescaler divisor factor */
                  CAN_CBT_EPROPSEG(priv->arbi_timing.propseg) | /* Propagation segment */
                  CAN_CBT_EPSEG1(priv->arbi_timing.pseg1) |     /* Phase buffer segment 1 */
                  CAN_CBT_EPSEG2(priv->arbi_timing.pseg2) |     /* Phase buffer segment 2 */
                  CAN_CBT_ERJW(1));                             /* Resynchronization jump width */

      /* Enable CAN FD feature */

      modifyreg32(priv->base + IMXRT_CAN_MCR_OFFSET, 0, CAN_MCR_FDEN);

      modifyreg32(priv->base + IMXRT_CAN_FDCBT_OFFSET,
                  CAN_FDCBT_FPRESDIV_MASK | CAN_FDCBT_FPROPSEG_MASK |
                  CAN_FDCBT_FPSEG1_MASK | CAN_FDCBT_FPSEG2_MASK |
                  CAN_FDCBT_FRJW_MASK,
                  CAN_FDCBT_FPRESDIV(priv->data_timing.presdiv) |  /* Prescaler divisor factor of 1 */
                  CAN_FDCBT_FPROPSEG(priv->data_timing.propseg) |  /* Propagation
                                                                    * segment (only register that doesn't add 1) */
                  CAN_FDCBT_FPSEG1(priv->data_timing.pseg1) |      /* Phase buffer segment 1 */
                  CAN_FDCBT_FPSEG2(priv->data_timing.pseg2) |      /* Phase buffer segment 2 */
                  CAN_FDCBT_FRJW(priv->data_timing.pseg2));        /* Resynchorinzation jump width same as PSEG2 */

      /* Additional CAN-FD configurations */

      tdcoff = (priv->data_timing.pseg1 + priv->data_timing.pseg2 + 2) *
        (priv->data_timing.presdiv + 1);

      modifyreg32(priv->base + IMXRT_CAN_FDCTRL_OFFSET, 0,
                  CAN_FDCTRL_FDRATE |          /* Enable bit rate switch in data phase of frame */
                  CAN_FDCTRL_TDCEN |           /* Enable transceiver delay compensation */
                  CAN_FDCTRL_TDCOFF(tdcoff) |  /* Setup 5 cycles for data phase sampling delay */
                  CAN_FDCTRL_MBDSR0(3) |       /* Setup 64 bytes per MB 0-6 */
                  CAN_FDCTRL_MBDSR1(3));       /* Setup 64 bytes per MB 7-13 */

      modifyreg32(priv->base + IMXRT_CAN_CTRL2_OFFSET, 0,
                  CAN_CTRL2_ISOCANFDEN);
    }

  /* Exit supervisor mode */

  modifyreg32(priv->base + IMXRT_CAN_MCR_OFFSET, CAN_MCR_SUPV, 0);

  /* Always compare also IDE and RTR bits to mask in RX */

  modifyreg32(priv->base + IMXRT_CAN_CTRL2_OFFSET, 0,
              CAN_CTRL2_RRS | CAN_CTRL2_EACEN);

  /* Lowest-numbered active TX mailbox first.  Combined with the software
   * mailbox placement logic this prevents local TX re-ordering.
   */

  modifyreg32(priv->base + IMXRT_CAN_CTRL1_OFFSET, 0, CAN_CTRL1_LBUF);

  /* Clear MB interrupts */

  putreg32(IFLAG1_TX | IFLAG1_RX, priv->base + IMXRT_CAN_IFLAG1_OFFSET);

  /* Enable MB interrupts */

  putreg32(IFLAG1_TX | IFLAG1_RX, priv->base + IMXRT_CAN_IMASK1_OFFSET);

#ifdef FLEXCAN_ERR005829
  /* ERR005829: reserve the first valid mailbox (MB0) as permanently
   * inactive. RX FIFO is disabled in this driver, so the errata
   * requires MB0 to be the reserved mailbox.
   */

  imxrt_err005829_kick(priv);
#endif

  /* Set RX buffers to receive */

  for (i = RXMB_FIRST; i < RXMB_END; i++)
    {
      mb = flexcan_get_mb(priv, i);
      mb->cs = (CAN_RXMB_EMPTY << CAN_MB_CS_CODE_SHIFT) | CAN_MB_CS_IDE;
    }

  for (i = TXMB_FIRST; i < TXMB_END; i++)
    {
      mb = flexcan_get_mb(priv, i);
      mb->cs = CAN_TXMB_INACTIVE << CAN_MB_CS_CODE_SHIFT;
#ifdef TX_TIMEOUT_WQ
      imxrt_deadline_clear(&priv->txmb[imxrt_txmb_index(i)]);
      wd_cancel(&priv->txtimeout[imxrt_txmb_index(i)]);
#endif
    }

  putreg32(IFLAG1_TX | IFLAG1_RX, priv->base + IMXRT_CAN_IFLAG1_OFFSET);
  putreg32(IFLAG1_TX | IFLAG1_RX, priv->base + IMXRT_CAN_IMASK1_OFFSET);

  /* Exit freeze mode */

  if (!imxrt_setfreeze(priv->base, false))
    {
      canerr("FLEXCAN: unfreeze fail\n");
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Function: imxrt_reset
 *
 * Description:
 *   Reset the flexcan and put it into disabled state
 *
 * Input Parameters:
 *   priv - Reference to the private FLEXCAN driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void imxrt_reset(struct imxrt_driver_s *priv)
{
  /* Make sure module is enabled */

  if (!imxrt_setenable(priv->base, true))
    {
      canerr("Enable fail\n");
      return;
    }

  modifyreg32(priv->base + IMXRT_CAN_MCR_OFFSET, 0, CAN_MCR_SOFTRST);

  if (!imxrt_waitmcr_change(priv->base, CAN_MCR_SOFTRST, false))
    {
      canerr("Reset failed");
      return;
    }

  /* Disable module */

  if (!imxrt_setenable(priv->base, false))
    {
      canerr("Disable fail\n");
      return;
    }
}

/****************************************************************************
 * Function: imxrt_canpinmux
 *
 * Description:
 *   Mux the pins used for CAN RX&TX
 *§
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void imxrt_canpinmux(void)
{
#ifdef CONFIG_IMXRT_FLEXCAN1
  imxrt_config_gpio(GPIO_FLEXCAN1_TX);
  imxrt_config_gpio(GPIO_FLEXCAN1_RX);
#endif

#ifdef CONFIG_IMXRT_FLEXCAN2
  imxrt_config_gpio(GPIO_FLEXCAN2_TX);
  imxrt_config_gpio(GPIO_FLEXCAN2_RX);
#endif

#ifdef CONFIG_IMXRT_FLEXCAN3
  imxrt_config_gpio(GPIO_FLEXCAN3_TX);
  imxrt_config_gpio(GPIO_FLEXCAN3_RX);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: imxrt_caninitialize
 *
 * Description:
 *   Initialize the CAN controller and driver
 *
 * Input Parameters:
 *   intf - In the case where there are multiple CAN, this value
 *          identifies which CAN is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int imxrt_caninitialize(int intf)
{
  struct imxrt_driver_s *priv;
  int ret;

  switch (intf)
    {
#ifdef CONFIG_IMXRT_FLEXCAN1
    case 1:
      imxrt_clockall_can1();
      imxrt_clockall_can1_serial();
      priv = &g_flexcan1;
      break;
#endif

#ifdef CONFIG_IMXRT_FLEXCAN2
    case 2:
      imxrt_clockall_can2();
      imxrt_clockall_can2_serial();
      priv = &g_flexcan2;
      break;
#endif

#ifdef CONFIG_IMXRT_FLEXCAN3
    case 3:
      imxrt_clockall_can3();
      imxrt_clockall_can3_serial();
      priv = &g_flexcan3;
      break;
#endif

    default:
      return -ENODEV;
    }

  if (imxrt_bitratetotimeseg(priv, &priv->arbi_timing, false) != OK)
    {
      canerr("ERROR: Invalid CAN timings please try another sample point "
             "or refer to the reference manual\n");
      return -1;
    }

  if (priv->canfd_capable)
    {
      if (imxrt_bitratetotimeseg(priv, &priv->data_timing, true) != OK)
        {
          canerr("ERROR: Invalid CAN data phase timings please try another "
                 "sample point or refer to the reference manual\n");
          return -1;
        }
    }

  /* Mux the can RX&TX pins */

  imxrt_canpinmux();

  if (irq_attach(priv->irq, imxrt_flexcan_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      canerr("ERROR: Failed to attach CAN bus IRQ\n");
      return -EAGAIN;
    }

  /* Disable */

  imxrt_setenable(priv->base, false);

  /* Initialize the driver structure */

  priv->dev.d_ifup    = imxrt_ifup;      /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = imxrt_ifdown;    /* I/F down callback */
  priv->dev.d_txavail = imxrt_txavail;   /* New TX data callback */
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = imxrt_ioctl;     /* Support CAN ioctl() calls */
#endif

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling imxrt_ifdown().
   */

  ninfo("callbacks done\n");

  imxrt_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->dev, NET_LL_CAN);

  UNUSED(ret);
  return OK;
}

/****************************************************************************
 * Name: arm_caninitialize
 *
 * Description:
 *   Initialize the can network interfaces.
 *
 ****************************************************************************/

#if !defined(CONFIG_NETDEV_LATEINIT)
void arm_netinitialize(void)
{
#ifdef CONFIG_IMXRT_FLEXCAN1
  imxrt_caninitialize(1);
#endif

#ifdef CONFIG_IMXRT_FLEXCAN2
  imxrt_caninitialize(2);
#endif

#ifdef CONFIG_IMXRT_FLEXCAN3
  imxrt_caninitialize(3);
#endif
}
#endif

#endif /* CONFIG_IMXRT_FLEXCAN */
