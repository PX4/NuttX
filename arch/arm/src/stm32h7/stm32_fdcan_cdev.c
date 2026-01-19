/****************************************************************************
 * arch/arm/src/stm32h7/stm32_fdcan_cdev.c
 *
 * FDCAN Character Device Driver for STM32H7
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/can/can.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32.h"
#include "stm32_rcc.h"
#include "stm32_gpio.h"
#include "hardware/stm32_fdcan.h"
#include "stm32_fdcan_cdev.h"

#if defined(CONFIG_CAN) && (defined(CONFIG_STM32H7_CAN1) || \
    defined(CONFIG_STM32H7_CAN2) || defined(CONFIG_STM32H7_CAN3))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bit timing ***************************************************************/

#define CAN_BIT_QUANTA (CONFIG_STM32H7_CAN_TSEG1 + CONFIG_STM32H7_CAN_TSEG2 + 1)

/* FDCAN peripheral clock comes from HSE (High Speed External oscillator)
 * This is configured via RCC_D2CCIP1R_FDCANSEL in board.h
 */
#define STM32_FDCANCLK  STM32_HSE_FREQUENCY
#define CLK_FREQ        STM32_FDCANCLK

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_fdcan_s
{
  uint8_t  port;             /* CAN port number (1, 2, or 3) */
  uint8_t  canrx[2];         /* RX IRQ numbers */
  uint8_t  cantx;            /* TX IRQ number */
  uint32_t base;             /* Base address of FDCAN registers */
  uint32_t baud;             /* Configured baud rate */
  uint32_t rxmsgram_offset;  /* RX message RAM offset */
  uint32_t txmsgram_offset;  /* TX message RAM offset */
};

struct fdcan_bitseg
{
  uint32_t bitrate;
  uint8_t  sjw;
  uint8_t  bs1;
  uint8_t  bs2;
  uint16_t prescaler;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* CAN driver methods */

static void stm32_fdcan_reset(struct can_dev_s *dev);
static int  stm32_fdcan_setup(struct can_dev_s *dev);
static void stm32_fdcan_shutdown(struct can_dev_s *dev);
static void stm32_fdcan_rxint(struct can_dev_s *dev, bool enable);
static void stm32_fdcan_txint(struct can_dev_s *dev, bool enable);
static int  stm32_fdcan_ioctl(struct can_dev_s *dev, int cmd, unsigned long arg);
static int  stm32_fdcan_remoterequest(struct can_dev_s *dev, uint16_t id);
static int  stm32_fdcan_send(struct can_dev_s *dev, struct can_msg_s *msg);
static bool stm32_fdcan_txready(struct can_dev_s *dev);
static bool stm32_fdcan_txempty(struct can_dev_s *dev);

/* Helper functions */

static int fdcan_bittiming(struct fdcan_bitseg *timing);
static void stm32_fdcan_configure_message_ram(struct stm32_fdcan_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct can_ops_s g_fdcan_ops =
{
  .co_reset         = stm32_fdcan_reset,
  .co_setup         = stm32_fdcan_setup,
  .co_shutdown      = stm32_fdcan_shutdown,
  .co_rxint         = stm32_fdcan_rxint,
  .co_txint         = stm32_fdcan_txint,
  .co_ioctl         = stm32_fdcan_ioctl,
  .co_remoterequest = stm32_fdcan_remoterequest,
  .co_send          = stm32_fdcan_send,
  .co_txready       = stm32_fdcan_txready,
  .co_txempty       = stm32_fdcan_txempty,
};

#ifdef CONFIG_STM32H7_CAN1
static struct stm32_fdcan_s g_fdcan1priv =
{
  .port             = 1,
  .canrx            = { STM32_IRQ_FDCAN1_0, STM32_IRQ_FDCAN1_1 },
  .cantx            = STM32_IRQ_FDCAN1_0,
  .base             = STM32_FDCAN1_BASE,
  .baud             = CONFIG_STM32H7_CAN1_BAUD,
  .rxmsgram_offset  = 0,
  .txmsgram_offset  = 0,
};

static struct can_dev_s g_fdcan1dev =
{
  .cd_ops           = &g_fdcan_ops,
  .cd_priv          = &g_fdcan1priv,
};
#endif

#ifdef CONFIG_STM32H7_CAN2
static struct stm32_fdcan_s g_fdcan2priv =
{
  .port             = 2,
  .canrx            = { STM32_IRQ_FDCAN2_0, STM32_IRQ_FDCAN2_1 },
  .cantx            = STM32_IRQ_FDCAN2_0,
  .base             = STM32_FDCAN2_BASE,
  .baud             = CONFIG_STM32H7_CAN2_BAUD,
  .rxmsgram_offset  = 0,
  .txmsgram_offset  = 0,
};

static struct can_dev_s g_fdcan2dev =
{
  .cd_ops           = &g_fdcan_ops,
  .cd_priv          = &g_fdcan2priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdcan_bittiming
 *
 * Description:
 *   Convert desired bitrate to FDCAN bit segment values
 *   The computed values apply to both data and arbitration phases
 *
 * Input Parameters:
 *   timing - structure to store bit timing
 *
 * Returned Value:
 *   OK on success; >0 on failure.
 *
 ****************************************************************************/

static int fdcan_configure_bittiming(struct stm32_fdcan_s *priv)
{
  /* Implementation ported from PX4's uavcan_drivers/stm32[h7]
   *
   * Ref. "Automatic Baudrate Detection in CANopen Networks", U. Koppe
   *  MicroControl GmbH & Co. KG
   *  CAN in Automation, 2003
   *
   * According to the source, optimal quanta per bit are:
   *   Bitrate        Optimal Maximum
   *   1000 kbps      8       10
   *   500  kbps      16      17
   *   250  kbps      16      17
   *   125  kbps      16      17
   */

  const uint32_t target_bitrate    = priv->baud;
  static const int32_t max_bs1     = 16;
  static const int32_t max_bs2     = 8;
  const uint8_t max_quanta_per_bit = (priv->baud >= 1000000) ? 10 : 17;
  static const int max_sp_location = 900;

  /* Computing (prescaler * BS):
   *   BITRATE = 1 / (PRESCALER * (1 / PCLK) * (1 + BS1 + BS2))
   *   BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))
   * let:
   *   BS = 1 + BS1 + BS2
   *     (BS == total number of time quanta per bit)
   *   PRESCALER_BS = PRESCALER * BS
   * ==>
   *   PRESCALER_BS = PCLK / BITRATE
   */

  const uint32_t prescaler_bs = CLK_FREQ / target_bitrate;

  /* Find prescaler value such that the number of quanta per bit is highest */

  uint8_t bs1_bs2_sum = max_quanta_per_bit - 1;

  while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0)
    {
      if (bs1_bs2_sum <= 2)
        {
          canerr("Target bitrate too high - no solution possible.\n");
          return 1; /* No solution */
        }

      bs1_bs2_sum--;
    }

  const uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);

  if ((prescaler < 1U) || (prescaler > 1024U))
    {
      canerr("Target bitrate invalid - bad prescaler.\n");
      return 2; /* No solution */
    }

  /* Now we have a constraint: (BS1 + BS2) == bs1_bs2_sum.
   * We need to find the values so that the sample point is as close as
   * possible to the optimal value.
   *
   *   Solve[(1 + bs1)/(1 + bs1 + bs2) == 7/8, bs2]
   *     (Where 7/8 is 0.875, the recommended sample point location)
   *   {{bs2 -> (1 + bs1)/7}}
   *
   * Hence:
   *   bs2 = (1 + bs1) / 7
   *   bs1 = (7 * bs1_bs2_sum - 1) / 8
   *
   * Sample point location can be computed as follows:
   *   Sample point location = (1 + bs1) / (1 + bs1 + bs2)
   *
   * Since the optimal solution is so close to the maximum, we prepare two
   * solutions, and then pick the best one:
   *   - With rounding to nearest
   *   - With rounding to zero
   */

  /* First attempt with rounding to nearest */

  uint8_t bs1 = (uint8_t)((7 * bs1_bs2_sum - 1) + 4) / 8;
  uint8_t bs2 = (uint8_t)(bs1_bs2_sum - bs1);
  uint16_t sample_point_permill =
    (uint16_t)(1000 * (1 + bs1) / (1 + bs1 + bs2));

  if (sample_point_permill > max_sp_location)
    {
      /* Second attempt with rounding to zero */

      bs1 = (7 * bs1_bs2_sum - 1) / 8;
      bs2 = bs1_bs2_sum - bs1;
    }

  bool valid = (bs1 >= 1) && (bs1 <= max_bs1) && (bs2 >= 1) &&
    (bs2 <= max_bs2);

  /* Final validation
   * Helpful Python:
   * def sample_point_from_btr(x):
   *     assert 0b0011110010000000111111000000000 & x == 0
   *     ts2,ts1,brp = (x>>20)&7, (x>>16)&15, x&511
   *     return (1+ts1+1)/(1+ts1+1+ts2+1)
   */

  if (target_bitrate != (CLK_FREQ / (prescaler * (1 + bs1 + bs2))) || !valid)
    {
      canerr("Target bitrate invalid - solution does not match.\n");
      return 3; /* Solution not found */
    }

  caninfo("[fdcan] CLK_FREQ %u, target_bitrate %lu, prescaler %lu, bs1 %d"
          ", bs2 %d\n", (unsigned int)CLK_FREQ, target_bitrate, prescaler_bs, bs1 - 1,
          bs2 - 1);

  uint8_t fin_bs1 = (uint8_t)(bs1 - 1);
  uint8_t fin_bs2 = (uint8_t)(bs2 - 1);
  uint16_t fin_prescaler = (uint16_t)(prescaler - 1);
  uint8_t fin_sjw = 0; /* Which means one */

  caninfo("FDCAN%d: Calculated timing - SJW=%d BS1=%d BS2=%d Prescaler=%d\n",
          priv->port, fin_sjw, fin_bs1, fin_bs2, fin_prescaler);

  uint32_t regval = (fin_sjw << FDCAN_NBTP_NSJW_SHIFT) |
           (fin_bs1 << FDCAN_NBTP_NTSEG1_SHIFT) |
           (fin_bs2 << FDCAN_NBTP_TSEG2_SHIFT) |
           (fin_prescaler << FDCAN_NBTP_NBRP_SHIFT);

  putreg32(regval, priv->base + STM32_FDCAN_NBTP_OFFSET);

  /* Configure data phase (even though we're not using CAN-FD), not sure if this is needed */
  putreg32(regval, priv->base + STM32_FDCAN_DBTP_OFFSET);

  return 0;
}

/****************************************************************************
 * Name: stm32_fdcan_configure_message_ram
 *
 * Description:
 *   Configure the FDCAN Message RAM layout including filters, FIFOs, and
 *   TX buffers. This must be called while in INIT mode with CCE enabled.
 *
 * Input Parameters:
 *   priv - Pointer to the private FDCAN driver state structure
 *
 ****************************************************************************/

static void stm32_fdcan_configure_message_ram(struct stm32_fdcan_s *priv)
{
  uint32_t regval;
  uint32_t ram_offset;

  /* Configure Message RAM
   *
   * EXACT layout per user specification (all sections in strict order):
   * Section                  Elements    Words/Element   Total Words
   * ----------------------------------------------------------------
   * SIDFC.FLSSA 11-bit filter   0-128        1            0-128
   * XIDFC.FLESA 29-bit filter   0-64         2            0-128
   * RXF0C.F0SA  Rx FIFO 0       0-64        18            0-1152
   * RXF1C.F1SA  Rx FIFO 1       0-64        18            0-1152
   * RXBC.RBSA   Rx buffer       0-64        18            0-1152
   * TXEFC.EFSA  Tx event FIFO   0-32         2            0-64
   * TXBC.TBSA   Tx buffers      0-32        18            0-576
   * TMC.TMSA    Trigger memory  0-64         2            0-128
   *
   * Note: Element sizes depend on RXESC/TXESC configuration
   * For RXESC=0, TXESC=0 (8-byte data): Element = 4 words (16 bytes)
   *
   * Our configuration (fitting in 1280 words per interface):
   * 1. 11-bit filter: 1 element (128 words allocated)
   * 2. 29-bit filter: 0 elements (0 words)
   * 3. RX FIFO 0: 64 elements @ 4 words = 256 words
   * 4. RX FIFO 1: 0 elements (0 words)
   * 5. RX Buffer: 0 elements (0 words)
   * 6. TX Event FIFO: 0 elements (0 words)
   * 7. TX Buffers: 32 elements @ 4 words = 128 words
   * 8. Trigger Memory: 0 elements (0 words)
   * Total: 512 words
   */
  ram_offset = priv->rxmsgram_offset;

  /* 1. SIDFC.FLSSA: 11-bit filter (Standard ID Filter Configuration) */
  regval = (ram_offset << FDCAN_SIDFC_FLSSA_SHIFT) | (1 << FDCAN_SIDFC_LSS_SHIFT);
  putreg32(regval, priv->base + STM32_FDCAN_SIDFC_OFFSET);
  ram_offset += 128;  /* Allocate full 128 words */

  /* 2. XIDFC.FLESA: 29-bit filter (Extended ID Filter Configuration) */
  regval = (ram_offset << FDCAN_XIDFC_FLESA_SHIFT) | (0 << FDCAN_XIDFC_LSE_SHIFT);
  putreg32(regval, priv->base + STM32_FDCAN_XIDFC_OFFSET);
  /* ram_offset += 0; - 0 elements */

  /* 3. RXF0C.F0SA: Rx FIFO 0 (Receive FIFO 0 Configuration) */
  regval = (ram_offset << FDCAN_RXF0C_F0SA_SHIFT) | (64 << FDCAN_RXF0C_F0S_SHIFT);
  putreg32(regval, priv->base + STM32_FDCAN_RXF0C_OFFSET);
  ram_offset += 64 * 4;  /* 64 elements @ 4 words each = 256 words */

  /* 4. RXF1C.F1SA: Rx FIFO 1 (Receive FIFO 1 Configuration) */
  regval = (ram_offset << FDCAN_RXF1C_F1SA_SHIFT) | (0 << FDCAN_RXF1C_F1S_SHIFT);
  putreg32(regval, priv->base + STM32_FDCAN_RXF1C_OFFSET);
  /* ram_offset += 0; - 0 elements */

  /* 5. RXBC.RBSA: Rx Buffer (Receive Buffer Configuration) */
  regval = (ram_offset << FDCAN_RXBC_RBSA_SHIFT);
  putreg32(regval, priv->base + STM32_FDCAN_RXBC_OFFSET);
  /* ram_offset += 0; - 0 elements */

  /* 6. TXEFC.EFSA: Tx Event FIFO (Transmit Event FIFO Configuration) */
  regval = (ram_offset << FDCAN_TXEFC_EFSA_SHIFT) | (0 << FDCAN_TXEFC_EFS_SHIFT);
  putreg32(regval, priv->base + STM32_FDCAN_TXEFC_OFFSET);
  /* ram_offset += 0; - 0 elements */

  /* 7. TXBC.TBSA: Tx Buffers (Transmit Buffer Configuration)
   * TFQS: Number of Tx FIFO/Queue elements
   * Maximum TFQS = 32 (NDTB + TFQS must be <= 32)
   */
  regval = (ram_offset << FDCAN_TXBC_TBSA_SHIFT) | (32 << FDCAN_TXBC_TFQS_SHIFT);
  putreg32(regval, priv->base + STM32_FDCAN_TXBC_OFFSET);
  ram_offset += 32 * 4;  /* 32 elements @ 4 words each = 128 words */

  /* 8. TMC.TMSA: Trigger Memory - Not used in our configuration */
  /* No register configuration needed if not using trigger memory */

  caninfo("FDCAN%d: Message RAM configured, total used: %lu words (of 1280 available)\n",
          (unsigned long)(priv->port), (unsigned long)(ram_offset - priv->rxmsgram_offset));

  /* Configure element sizes for Rx and Tx buffers
   * IMPORTANT: These registers define the data field size
   * Value 0 = 8 bytes (classic CAN)
   */

  /* Rx Element Size: 8-byte data field for both FIFOs and RX Buffer */
  putreg32(0, priv->base + STM32_FDCAN_RXESC_OFFSET);  /* 0 = 8 bytes */

  /* Tx Element Size: 8-byte data field */
  putreg32(0, priv->base + STM32_FDCAN_TXESC_OFFSET);  /* 0 = 8 bytes */

  caninfo("FDCAN%d: Element sizes configured (RXESC=0, TXESC=0)\n", priv->port);
}

/****************************************************************************
 * Name: stm32_fdcan_configure_message_ram
 *
 * Description:
 *   Configure the FDCAN filters (currently set to accept all messages into)
 *

 ****************************************************************************/

static void stm32_fdcan_configure_filters(struct stm32_fdcan_s *priv)
{
  /* Configure ONE accept-all filter
   * Filter format: [31:30]=SFEC, [29:16]=SFID1, [15:0]=SFID2
   * SFEC=1: Store in RX FIFO 0 if filter matches
   */
  uint32_t *filter_ram = (uint32_t *)(STM32_CANRAM_BASE + (priv->rxmsgram_offset * 4));
  filter_ram[0] = (1 << 30) | (0x7FF << 16) | 0x000;  /* Accept all to FIFO 0 */

  /* Configure global filter to accept all messages into FIFO 0*/
  uint32_t regval;
  regval = (0 << FDCAN_GFC_ANFS_SHIFT) | (0 << FDCAN_GFC_ANFE_SHIFT);
  putreg32(regval, priv->base + STM32_FDCAN_GFC_OFFSET);
  caninfo("FDCAN%d: Global filter: accept all to FIFO 0 (GFC=0x%08lX)\n", priv->port, regval);
}

/****************************************************************************
 * Name: stm32_fdcan_dump_registers
 *
 * Description:
 *   Dump all relevant FDCAN registers for debugging
 *
 ****************************************************************************/

static void stm32_fdcan_dump_registers(struct stm32_fdcan_s *priv, const char *context)
{
  printf("\n=== FDCAN%d Register Dump: %s ===\n", priv->port, context);

  /* RCC Clock */
  uint32_t rcc = getreg32(STM32_RCC_APB1HENR);
  printf("RCC_APB1HENR: 0x%08lx (FDCANEN=%d)\n", rcc, !!(rcc & RCC_APB1HENR_FDCANEN));

  /* Control registers */
  uint32_t cccr = getreg32(priv->base + STM32_FDCAN_CCCR_OFFSET);
  uint32_t test = getreg32(priv->base + STM32_FDCAN_TEST_OFFSET);
  uint32_t nbtp = getreg32(priv->base + STM32_FDCAN_NBTP_OFFSET);
  uint32_t dbtp = getreg32(priv->base + STM32_FDCAN_DBTP_OFFSET);

  printf("Control Registers:\n");
  printf("CCCR: 0x%08lx (INIT=%d CCE=%d TEST=%d MON=%d FDOE=%d)\n",
         cccr, !!(cccr & FDCAN_CCCR_INIT), !!(cccr & FDCAN_CCCR_CCE),
         !!(cccr & FDCAN_CCCR_TEST), !!(cccr & FDCAN_CCCR_MON),
         !!(cccr & FDCAN_CCCR_FDOE));
  printf("TEST: 0x%08lx (LBCK=%d TX=%lu RX=%d)\n",
         test, !!(test & FDCAN_TEST_LBCK), (test >> 5) & 0x3, !!(test & (1<<7)));
  printf("NBTP: 0x%08lx\n", nbtp);
  printf("DBTP: 0x%08lx\n", dbtp);

  /* Status registers */
  uint32_t psr = getreg32(priv->base + STM32_FDCAN_PSR_OFFSET);
  uint32_t ecr = getreg32(priv->base + STM32_FDCAN_ECR_OFFSET);
  uint32_t ir = getreg32(priv->base + STM32_FDCAN_IR_OFFSET);
  uint32_t ie = getreg32(priv->base + STM32_FDCAN_IE_OFFSET);
  uint32_t ils = getreg32(priv->base + STM32_FDCAN_ILS_OFFSET);
  uint32_t ile = getreg32(priv->base + STM32_FDCAN_ILE_OFFSET);

  printf("Status Registers:\n");
  printf("PSR:  0x%08lx (LEC=%lu ACT=%lu)\n", psr, psr & 0x7, (psr >> 3) & 0x3);
  printf("ECR:  0x%08lx (TEC=%lu REC=%lu)\n", ecr, (ecr >> 0) & 0xFF, (ecr >> 8) & 0x7F);
  printf("IR:   0x%08lx (TC=%d RF0N=%d RF0L=%d PEA=%d)\n",
         ir, !!(ir & (1<<9)), !!(ir & (1<<19)), !!(ir & (1<<21)), !!(ir & (1<<27)));
  printf("IE:   0x%08lx\n", ie);
  printf("ILS:  0x%08lx\n", ils);
  printf("ILE:  0x%08lx (EINT0=%d EINT1=%d)\n",
         ile, !!(ile & FDCAN_ILE_EINT0), !!(ile & FDCAN_ILE_EINT1));

  /* Filter configuration */
  uint32_t gfc = getreg32(priv->base + STM32_FDCAN_GFC_OFFSET);
  uint32_t sidfc = getreg32(priv->base + STM32_FDCAN_SIDFC_OFFSET);
  uint32_t xidfc = getreg32(priv->base + STM32_FDCAN_XIDFC_OFFSET);

  printf("Filter Configuration:\n");
  printf("GFC:   0x%08lx (ANFS=%lu ANFE=%lu)\n", gfc, (gfc >> 4) & 0x3, (gfc >> 2) & 0x3);
  printf("SIDFC: 0x%08lx (LSS=%lu)\n", sidfc, (sidfc >> 16) & 0xFF);
  printf("XIDFC: 0x%08lx\n", xidfc);

  /* RX configuration and status */
  uint32_t rxesc = getreg32(priv->base + STM32_FDCAN_RXESC_OFFSET);
  uint32_t rxf0c = getreg32(priv->base + STM32_FDCAN_RXF0C_OFFSET);
  uint32_t rxf0s = getreg32(priv->base + STM32_FDCAN_RXF0S_OFFSET);
  uint32_t rxf1c = getreg32(priv->base + STM32_FDCAN_RXF1C_OFFSET);
  uint32_t rxf1s = getreg32(priv->base + STM32_FDCAN_RXF1S_OFFSET);

  printf("RX Configuration and Status:\n");
  printf("RXESC: 0x%08lx\n", rxesc);
  printf("RXF0C: 0x%08lx (size=%lu)\n", rxf0c, (rxf0c >> 16) & 0x7F);
  printf("RXF0S: 0x%08lx (fill=%lu get=%lu put=%lu full=%d lost=%d)\n",
         rxf0s, rxf0s & 0x7F, (rxf0s >> 8) & 0x3F, (rxf0s >> 16) & 0x3F,
         !!(rxf0s & (1<<24)), !!(rxf0s & (1<<25)));
  printf("RXF1C: 0x%08lx\n", rxf1c);
  printf("RXF1S: 0x%08lx (fill=%lu)\n", rxf1s, rxf1s & 0x7F);

  /* TX configuration and status */
  uint32_t txesc = getreg32(priv->base + STM32_FDCAN_TXESC_OFFSET);
  uint32_t txbc = getreg32(priv->base + STM32_FDCAN_TXBC_OFFSET);
  uint32_t txfqs = getreg32(priv->base + STM32_FDCAN_TXFQS_OFFSET);
  uint32_t txbrp = getreg32(priv->base + STM32_FDCAN_TXBRP_OFFSET);
  uint32_t txbar = getreg32(priv->base + STM32_FDCAN_TXBAR_OFFSET);
  uint32_t txbto = getreg32(priv->base + STM32_FDCAN_TXBTO_OFFSET);

  printf("TX Configuration and Status:\n");
  printf("TXESC: 0x%08lx\n", txesc);
  printf("TXBC:  0x%08lx (size=%lu)\n", txbc, (txbc >> 24) & 0x3F);
  printf("TXFQS: 0x%08lx (free=%lu get=%lu put=%lu full=%d)\n",
         txfqs, txfqs & 0x3F, (txfqs >> 8) & 0x1F, (txfqs >> 16) & 0x1F,
         !!(txfqs & (1<<21)));
  printf("TXBRP: 0x%08lx (pending)\n", txbrp);
  printf("TXBAR: 0x%08lx (add request)\n", txbar);
  printf("TXBTO: 0x%08lx (transmitted)\n", txbto);

  printf("=== End Register Dump ===\n\n");
}

/****************************************************************************
 * Name: stm32_fdcan_interrupt
 *
 * Description:
 *   Common FDCAN interrupt handler
 *
 ****************************************************************************/

static int stm32_fdcan_interrupt(int irq, void *context, void *arg)
{
  struct can_dev_s *dev = (struct can_dev_s *)arg;
  struct stm32_fdcan_s *priv = (struct stm32_fdcan_s *)dev->cd_priv;
  uint32_t ir;
  uint32_t rxf0s;
  uint32_t get_index;
  uint32_t fill_level;
  struct can_msg_s msg;
  uint32_t *buffer;

  static uint32_t isr_count = 0;
  isr_count++;
  if ((isr_count % 100) == 0)
    {
      printf("FDCAN%d: ISR called %lu times\n", priv->port, isr_count);
    }

  /* Read interrupt status */
  ir = getreg32(priv->base + STM32_FDCAN_IR_OFFSET);

  if (ir == 0)
    {
      /* No interrupt flags set - spurious interrupt? */
      return OK;
    }

  /* Log any interrupt */
  if (ir != 0 && (isr_count % 10) == 0)
    {
      printf("FDCAN%d: IR=0x%08lX\n", priv->port, ir);
    }

  /* Handle RX FIFO 0 new message */
  if (ir & FDCAN_IR_RF0N)
    {
      /* Read RX FIFO 0 status */
      rxf0s = getreg32(priv->base + STM32_FDCAN_RXF0S_OFFSET);
      fill_level = (rxf0s >> 0) & 0x7F;  /* F0FL: Fill level */
      get_index = (rxf0s >> 8) & 0x3F;   /* F0GI: Get index */

      caninfo("FDCAN%d: RX interrupt, fill_level=%lu, get_index=%lu\n",
              priv->port, fill_level, get_index);

      if (fill_level > 0)
        {
          /* Calculate address of RX buffer element
           * RX FIFO 0 is at: base + rxmsgram_offset + 128 (filters) + get_index * 16
           */
          buffer = (uint32_t *)(STM32_CANRAM_BASE +
                                (priv->rxmsgram_offset + 128) * 4 +
                                get_index * 16);  /* 16 bytes per element */

          /* DEBUG: Dump entire RX element (4 words = 16 bytes) */
          printf("FDCAN%d: RX Element @ 0x%08lX (get_index=%lu, rxmsgram_offset=%lu):\n",
                 priv->port, (uint32_t)buffer, get_index, (unsigned long)priv->rxmsgram_offset);
          printf("  buffer[0] (R0): 0x%08lX\n", buffer[0]);
          printf("  buffer[1] (R1): 0x%08lX\n", buffer[1]);
          printf("  buffer[2] (R2): 0x%08lX\n", buffer[2]);
          printf("  buffer[3] (R3): 0x%08lX\n", buffer[3]);

          /* Read message from RX FIFO 0 */
          uint32_t word0 = buffer[0];
          uint32_t word1 = buffer[1];

          /* Parse header - RX element format is different from TX!
           * R0: [29]=XTD, [28:18]=ID (standard), [17:16]=RTR
           * R1: [30:24]=DLC, [20:16]=filter index
           */
          if (word0 & (1 << 29))  /* XTD bit - extended ID */
            {
              msg.cm_hdr.ch_id = (word0 & 0x1FFFFFFF);
              msg.cm_hdr.ch_extid = true;
            }
          else  /* Standard ID */
            {
              msg.cm_hdr.ch_id = (word0 >> 18) & 0x7FF;
              msg.cm_hdr.ch_extid = false;
            }

          msg.cm_hdr.ch_rtr = !!(word0 & (1 << 20));  /* RTR bit in R0[20] */
          msg.cm_hdr.ch_dlc = (word1 >> 24) & 0x7F;   /* DLC field in R1[30:24] - 7 bits! */

          /* For classic CAN, DLC > 8 should be treated as 8 */
          if (msg.cm_hdr.ch_dlc > 8)
            {
              msg.cm_hdr.ch_dlc = 8;
            }

          /* Calculate actual data length from DLC */
          uint8_t len = msg.cm_hdr.ch_dlc;

          /* Read data bytes from buffer[2] and buffer[3] (R2 and R3) */
          for (int i = 0; i < len; i++)
            {
              if (i < 4)
                msg.cm_data[i] = (buffer[2] >> (i * 8)) & 0xFF;
              else
                msg.cm_data[i] = (buffer[3] >> ((i - 4) * 8)) & 0xFF;
            }

          /* Acknowledge message by writing get index to RXF0A */
          putreg32(get_index, priv->base + STM32_FDCAN_RXF0A_OFFSET);

          printf("FDCAN%d: RX ID=0x%03lX DLC=%d Data: %02X %02X %02X %02X %02X %02X %02X %02X (word0=0x%08lX word1=0x%08lX)\n",
                 priv->port, (unsigned long)msg.cm_hdr.ch_id, msg.cm_hdr.ch_dlc,
                 msg.cm_data[0], msg.cm_data[1], msg.cm_data[2], msg.cm_data[3],
                 msg.cm_data[4], msg.cm_data[5], msg.cm_data[6], msg.cm_data[7],
                 word0, word1);

          /* Pass message to upper layer - commented out as no readers registered */
          can_receive(dev, &msg.cm_hdr, msg.cm_data);
        }

      /* Clear interrupt flag */
      putreg32(FDCAN_IR_RF0N, priv->base + STM32_FDCAN_IR_OFFSET);
    }

  /* Handle RX FIFO 0 full */
  if (ir & FDCAN_IR_RF0F)
    {
      canwarn("FDCAN%d: RX FIFO 0 full\n", priv->port);
      putreg32(FDCAN_IR_RF0F, priv->base + STM32_FDCAN_IR_OFFSET);
    }

  /* Handle RX FIFO 0 message lost */
  if (ir & FDCAN_IR_RF0L)
    {
      canerr("FDCAN%d: RX FIFO 0 message lost\n", priv->port);
      putreg32(FDCAN_IR_RF0L, priv->base + STM32_FDCAN_IR_OFFSET);
    }

  /* Handle TX complete */
  if (ir & FDCAN_IR_TC)
    {
      caninfo("FDCAN%d: TX complete\n", priv->port);

      /* Notify upper layer that TX is complete */
      can_txdone(dev);

      /* Clear interrupt flag */
      putreg32(FDCAN_IR_TC, priv->base + STM32_FDCAN_IR_OFFSET);
    }

  return OK;
}


/****************************************************************************
 * Name: stm32_fdcan_reset
 *
 * Description:
 *   Reset the FDCAN device.  Called early to initialize the hardware.
 *
 ****************************************************************************/

static void stm32_fdcan_reset(struct can_dev_s *dev)
{
  struct stm32_fdcan_s *priv = (struct stm32_fdcan_s *)dev->cd_priv;
  irqstate_t flags;

  caninfo("FDCAN%d: Reset\n", priv->port);

  flags = enter_critical_section();

  /* Enter initialization mode */
  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_INIT);

  /* Wait for init mode */
  int timeout = 1000;
  while (!(getreg32(priv->base + STM32_FDCAN_CCCR_OFFSET) & FDCAN_CCCR_INIT) && timeout-- > 0)
    {
      up_udelay(10);
    }

  /* Disable all interrupts */
  putreg32(0, priv->base + STM32_FDCAN_IE_OFFSET);

  /* Clear all interrupt flags (bits 0-29) */
  putreg32(0x3FFFFFFF, priv->base + STM32_FDCAN_IR_OFFSET);

  leave_critical_section(flags);

  caninfo("FDCAN%d: Reset complete\n", priv->port);
}

/****************************************************************************
 * Name: stm32_fdcan_setup
 *
 * Description:
 *   Configure the FDCAN. This method is called the first time that the CAN
 *   device is opened.
 *
 ****************************************************************************/

static int stm32_fdcan_setup(struct can_dev_s *dev)
{
	// CHECK STM32H7 reference manual RM0433, page 2433 for explanation of steps
	// Values and offsets are defined in /hardware/stm32_fdcan.h
	// modifyreg32, getreg32, putreg32 are defined in arm_internal.h
  struct stm32_fdcan_s *priv = (struct stm32_fdcan_s *)dev->cd_priv;
  uint32_t regval;
  uint32_t ram_offset;
  irqstate_t flags;
  int ret;

  flags = enter_critical_section();

  /* Step 0: Enable FDCAN peripheral clock and configure GPIO pins */
#ifdef CONFIG_STM32H7_CAN1
  if (priv->port == 1)
    {
      /* Enable FDCAN1 clock */
      modifyreg32(STM32_RCC_APB1HENR, 0, RCC_APB1HENR_FDCANEN);
      /* Configure GPIO pins for FDCAN1 */
      stm32_configgpio(GPIO_CAN1_RX);
      stm32_configgpio(GPIO_CAN1_TX);
    }
#endif

#ifdef CONFIG_STM32H7_CAN2
  if (priv->port == 2)
    {
      /* Enable FDCAN2 clock (same as FDCAN1, shared peripheral) */
      modifyreg32(STM32_RCC_APB1HENR, 0, RCC_APB1HENR_FDCANEN);
      /* Configure GPIO pins for FDCAN2 */
      stm32_configgpio(GPIO_CAN2_RX);
      stm32_configgpio(GPIO_CAN2_TX);
    }
#endif

  /* Step 1: Exit Power-down / Sleep mode */
  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_CSR, 0);

  /* Step 2: Enter Initialization mode */
  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_INIT);

  /* Wait for INIT mode acknowledgment */
  int timeout = 1000;
  while (!(getreg32(priv->base + STM32_FDCAN_CCCR_OFFSET) & FDCAN_CCCR_INIT) && timeout-- > 0)
    {
      up_udelay(10);
    }

  if (timeout <= 0)
    {
      canerr("ERROR: Failed to enter INIT mode\n");
      leave_critical_section(flags);
      return -ETIMEDOUT;
    }

  /* Step 3: Enable Configuration Change */
  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_CCE);

  /* Step 4: Disable interrupts during configuration */
  putreg32(0, priv->base + STM32_FDCAN_IE_OFFSET);

  /* Step 5: Configure bit timing */
  ret = fdcan_configure_bittiming(priv);
  if (ret != 0)
    {
      canerr("ERROR: Failed to calculate bit timing for baud rate %lu\n", priv->baud);
      leave_critical_section(flags);
      return -EINVAL;
    }

  /* Step 6: Disable CAN-FD mode (classic CAN only) */
  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE, 0);

  /* Step 7: Configure Message RAM layout */
  stm32_fdcan_configure_message_ram(priv);

  /* Step 8: Configure filters */
  stm32_fdcan_configure_filters(priv);

  /* Step 9: Clear all interrupt flags (bits 0-29) */
  putreg32(0x3FFFFFFF, priv->base + STM32_FDCAN_IR_OFFSET);

  /* Step 10: Configure interrupt line routing - route all to line 0 */
  putreg32(0, priv->base + STM32_FDCAN_ILS_OFFSET);  /* All interrupts to line 0 */

  /* Step 11: Enable interrupt lines */
  putreg32(FDCAN_ILE_EINT0 | FDCAN_ILE_EINT1, priv->base + STM32_FDCAN_ILE_OFFSET);

  /* Step 12: Enable basic interrupts */
  regval = FDCAN_IE_TCE | FDCAN_IE_RF0NE | FDCAN_IE_RF0FE;
  putreg32(regval, priv->base + STM32_FDCAN_IE_OFFSET);

  /* Step 13: Exit Configuration mode and start operation */
  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_CCE, 0);

  /* Step 13: Exit Initialization mode */
  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_INIT, 0);

  /* Wait for normal operation */
  timeout = 1000;
  while ((getreg32(priv->base + STM32_FDCAN_CCCR_OFFSET) & FDCAN_CCCR_INIT) && timeout-- > 0)
    {
      up_udelay(10);
    }

  if (timeout <= 0)
    {
      canerr("ERROR: Failed to exit INIT mode\n");
      leave_critical_section(flags);
      return -ETIMEDOUT;
    }

  leave_critical_section(flags);

  /* Attach FDCAN interrupt handlers */

  ret = irq_attach(priv->canrx[0], stm32_fdcan_interrupt, dev);
  if (ret < 0)
    {
      canerr("ERROR: Failed to attach FDCAN%d RX0 IRQ\n", priv->port);
      return ret;
    }

  ret = irq_attach(priv->canrx[1], stm32_fdcan_interrupt, dev);
  if (ret < 0)
    {
      canerr("ERROR: Failed to attach FDCAN%d RX1 IRQ\n", priv->port);
      irq_detach(priv->canrx[0]);
      return ret;
    }

  /* Enable the interrupts at the NVIC */
  up_enable_irq(priv->canrx[0]);
  up_enable_irq(priv->canrx[1]);

  caninfo("FDCAN%d: Setup complete\n", priv->port);

  //stm32_fdcan_dump_registers(priv, "Post-Setup");

  //stm32_fdcan_loopback(1, true);
  return OK;
}

/****************************************************************************
 * Name: stm32_fdcan_shutdown
 *
 * Description:
 *   Disable the FDCAN.  This method is called when the CAN device is closed.
 *
 ****************************************************************************/

static void stm32_fdcan_shutdown(struct can_dev_s *dev)
{
	printf("========== stm32_fdcan_shutdown called ==========\n");
  struct stm32_fdcan_s *priv = (struct stm32_fdcan_s *)dev->cd_priv;
  irqstate_t flags;

  caninfo("FDCAN%d: Shutdown\n", priv->port);

  flags = enter_critical_section();

  /* Disable all interrupts */
  putreg32(0, priv->base + STM32_FDCAN_IE_OFFSET);

  /* Clear all interrupt flags (bits 0-29) */
  putreg32(0x3FFFFFFF, priv->base + STM32_FDCAN_IR_OFFSET);

  /* Disable interrupts at the NVIC */
  up_disable_irq(priv->canrx[0]);
  up_disable_irq(priv->canrx[1]);

  /* Detach interrupt handlers */
  irq_detach(priv->canrx[0]);
  irq_detach(priv->canrx[1]);

  /* Enter initialization mode to stop operation */
  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_INIT);

  /* Enter sleep mode to reduce power consumption */
  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_CSR);

  leave_critical_section(flags);

  caninfo("FDCAN%d: Shutdown complete\n", priv->port);
}

/****************************************************************************
 * Name: stm32_fdcan_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 ****************************************************************************/

static void stm32_fdcan_rxint(struct can_dev_s *dev, bool enable)
{
  struct stm32_fdcan_s *priv = (struct stm32_fdcan_s *)dev->cd_priv;
  uint32_t regval;

  caninfo("FDCAN%d: RX interrupt %s\n", priv->port, enable ? "enable" : "disable");

  if (enable)
    {
      /* Enable RX FIFO 0 interrupts:
       * - RF0NE: New message in FIFO 0
       * - RF0FE: FIFO 0 full
       * - RF0LE: FIFO 0 message lost
       */
      regval = getreg32(priv->base + STM32_FDCAN_IE_OFFSET);
      regval |= (FDCAN_IE_RF0NE | FDCAN_IE_RF0FE | FDCAN_IE_RF0LE);
      putreg32(regval, priv->base + STM32_FDCAN_IE_OFFSET);
    }
  else
    {
      /* Disable RX FIFO 0 interrupts */
      regval = getreg32(priv->base + STM32_FDCAN_IE_OFFSET);
      regval &= ~(FDCAN_IE_RF0NE | FDCAN_IE_RF0FE | FDCAN_IE_RF0LE);
      putreg32(regval, priv->base + STM32_FDCAN_IE_OFFSET);
    }
}

/****************************************************************************
 * Name: stm32_fdcan_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts.
 *
 ****************************************************************************/

static void stm32_fdcan_txint(struct can_dev_s *dev, bool enable)
{
  struct stm32_fdcan_s *priv = (struct stm32_fdcan_s *)dev->cd_priv;
  uint32_t regval;

  caninfo("FDCAN%d: TX interrupt %s\n", priv->port, enable ? "enable" : "disable");

  if (enable)
    {
      /* Enable TX complete interrupt */
      regval = getreg32(priv->base + STM32_FDCAN_IE_OFFSET);
      regval |= FDCAN_IE_TCE;
      putreg32(regval, priv->base + STM32_FDCAN_IE_OFFSET);
    }
  else
    {
      /* Disable TX complete interrupt */
      regval = getreg32(priv->base + STM32_FDCAN_IE_OFFSET);
      regval &= ~FDCAN_IE_TCE;
      putreg32(regval, priv->base + STM32_FDCAN_IE_OFFSET);
    }
}

/****************************************************************************
 * Name: stm32_fdcan_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int stm32_fdcan_ioctl(struct can_dev_s *dev, int cmd, unsigned long arg)
{
  caninfo("FDCAN ioctl cmd=%d - TODO\n", cmd);
  /* TODO: */
  return -ENOTTY;
}

/****************************************************************************
 * Name: stm32_fdcan_remoterequest
 *
 * Description:
 *   Send a remote request
 *
 ****************************************************************************/

static int stm32_fdcan_remoterequest(struct can_dev_s *dev, uint16_t id)
{
  caninfo("FDCAN remote request id=%d - TODO\n", id);
  /* TODO: */
  return -ENOSYS;
}

/****************************************************************************
 * Name: stm32_fdcan_send
 *
 * Description:
 *   Send one CAN message.
 *
 ****************************************************************************/

static int stm32_fdcan_send(struct can_dev_s *dev, struct can_msg_s *msg)
{
	//printf("stm32_fdcan_send called\n");
  struct stm32_fdcan_s *priv = (struct stm32_fdcan_s *)dev->cd_priv;

  //stm32_fdcan_dump_registers(priv, "Before Send");

  /* Check if TX FIFO has space */
  uint32_t txfqs;
  txfqs = getreg32(priv->base + STM32_FDCAN_TXFQS_OFFSET);

  if ((txfqs & FDCAN_TXFQS_TFQF) != 0)
    {
      /* TX FIFO is full */
      canerr("ERROR: TX FIFO full\n");
      return -EBUSY;
    }

  if (((txfqs >> 0) & 0x3F) == 0)
    {
      canerr("ERROR: TX FIFO free level is 0! TXBC=0x%08lx\n",
             getreg32(priv->base + STM32_FDCAN_TXBC_OFFSET));
      return -EBUSY;
    }

  /* Get the put index (where to write next message) */
  uint32_t put_index;
  put_index = (txfqs & FDCAN_TXFQS_TFQPI_MASK) >> FDCAN_TXFQS_TFQPI_SHIFT;

  irqstate_t flags;
  flags = enter_critical_section();

  /* Calculate TX buffer address in message RAM
   * Base address + port offset + buffer offset + (put_index * element_size)
   * Element size = 4 words (16 bytes) for classic CAN with 8-byte data
   */
  uint32_t *tx_buffer;
  tx_buffer = (uint32_t *)(STM32_CANRAM_BASE +
                          (priv->txmsgram_offset * 4) +
                          (put_index * 16));

  /* Build word 0: ID and control bits
   * Bits [31:18]: Standard ID (11 bits) for standard frames
   * Bits [31:0]: Extended ID (29 bits) for extended frames
   * Bit [30]: RTR (Remote Transmission Request)
   * Bit [29]: XTD (Extended ID)
   * Bits [19:16]: DLC (Data Length Code)
   */
  uint32_t word0;
  uint32_t word1;
  if (msg->cm_hdr.ch_extid)
    {
      /* Extended ID (29 bits) */
      word0 = (msg->cm_hdr.ch_id & 0x1FFFFFFF);
      word0 |= (1 << 30);  /* Set XTD bit */
    }
  else
    {
      /* Standard ID (11 bits) in bits [28:18] */
      word0 = ((msg->cm_hdr.ch_id & 0x7FF) << 18);
    }

  if (msg->cm_hdr.ch_rtr)
    {
      word0 |= (1 << 29);  /* Set RTR bit */
    }

  /* Add DLC in bits [19:16] */
  word0 |= ((msg->cm_hdr.ch_dlc & 0xF) << 16);

  /* Word 1: Message marker and event FIFO control
   * Bits [31:24]: Message Marker
   * Bit [23]: Event FIFO Control
   * Other bits reserved
   */
  word1 = 0;  /* No marker, no event FIFO */

  /* Write to TX buffer */
  tx_buffer[0] = word0;
  tx_buffer[1] = word1;

  /* Write data bytes (words 2-3 for 8 bytes) */
  if (!msg->cm_hdr.ch_rtr && msg->cm_hdr.ch_dlc > 0)
    {
      /* Copy data bytes, handling endianness */
      tx_buffer[2] = ((uint32_t)msg->cm_data[0] << 0)  |
                     ((uint32_t)msg->cm_data[1] << 8)  |
                     ((uint32_t)msg->cm_data[2] << 16) |
                     ((uint32_t)msg->cm_data[3] << 24);

      if (msg->cm_hdr.ch_dlc > 4)
        {
          tx_buffer[3] = ((uint32_t)msg->cm_data[4] << 0)  |
                         ((uint32_t)msg->cm_data[5] << 8)  |
                         ((uint32_t)msg->cm_data[6] << 16) |
                         ((uint32_t)msg->cm_data[7] << 24);
        }
      else
        {
          tx_buffer[3] = 0;
        }
    }
  else
    {
      tx_buffer[2] = 0;
      tx_buffer[3] = 0;
    }

  /* ARM memory barrier to ensure TX buffer writes complete before triggering */
  //ARM_DMB();

  /* Trigger transmission by setting the corresponding bit in TXBAR */
  uint32_t txbar_value = (1 << put_index);
  putreg32(txbar_value, priv->base + STM32_FDCAN_TXBAR_OFFSET);

  /* Check protocol status for errors */
  static uint32_t tx_count = 0;
  tx_count++;
  if ((tx_count % 10) == 0)
    {
      uint32_t psr = getreg32(priv->base + STM32_FDCAN_PSR_OFFSET);
      uint32_t ecr = getreg32(priv->base + STM32_FDCAN_ECR_OFFSET);
      uint32_t rxf0s = getreg32(priv->base + STM32_FDCAN_RXF0S_OFFSET);
      uint32_t ir = getreg32(priv->base + STM32_FDCAN_IR_OFFSET);
      uint32_t ie = getreg32(priv->base + STM32_FDCAN_IE_OFFSET);

      printf("FDCAN%d: PSR=0x%08lX (LEC=%lu, BO=%lu, EP=%lu) ECR=0x%08lX (TEC=%lu, REC=%lu)\n",
             priv->port, psr, (unsigned long)((psr >> 0) & 0x7), (unsigned long)(!!(psr & (1<<7))), (unsigned long)(!!(psr & (1<<5))),
             ecr, (unsigned long)((ecr >> 0) & 0xFF), (unsigned long)((ecr >> 8) & 0x7F));
      printf("FDCAN%d: RXF0S=0x%08lX (fill=%lu, get=%lu) IR=0x%08lX IE=0x%08lX\n",
             priv->port, rxf0s, rxf0s & 0x7F, (rxf0s >> 8) & 0x3F, ir, ie);
    }

  leave_critical_section(flags);

  //stm32_fdcan_dump_registers(priv, "After Send");
  caninfo("FDCAN%d: Message queued for transmission\n", priv->port);
  printf("FDCAN%d: TX ID=0x%03X DLC=%d Data: %02X %02X %02X %02X %02X %02X %02X %02X (word0=0x%08lX word1=0x%08lX)\n",
         priv->port, msg->cm_hdr.ch_id, msg->cm_hdr.ch_dlc,
         msg->cm_data[0], msg->cm_data[1], msg->cm_data[2], msg->cm_data[3],
         msg->cm_data[4], msg->cm_data[5], msg->cm_data[6], msg->cm_data[7],
         word0, word1);
  return OK;
}

/****************************************************************************
 * Name: stm32_fdcan_txready
 *
 * Description:
 *   Return true if the TX hardware is ready to send another message.
 *
 ****************************************************************************/

static bool stm32_fdcan_txready(struct can_dev_s *dev)
{
  struct stm32_fdcan_s *priv = (struct stm32_fdcan_s *)dev->cd_priv;
  uint32_t txfqs;

  /* Read TX FIFO Queue Status register */
  txfqs = getreg32(priv->base + STM32_FDCAN_TXFQS_OFFSET);

  /* Check if FIFO is full */
  if ((txfqs & FDCAN_TXFQS_TFQF) != 0)
    {
      return false;  /* FIFO is full */
    }

  /* Check free level */
  uint32_t free_level = (txfqs & FDCAN_TXFQS_TFFL_MASK) >> FDCAN_TXFQS_TFFL_SHIFT;

  return (free_level > 0);
}

/****************************************************************************
 * Name: stm32_fdcan_txempty
 *
 * Description:
 *   Return true if all messages have been sent.  If for example, the CAN
 *   hardware implements FIFOs, then this would mean the transmit FIFO is
 *   empty.
 *
 ****************************************************************************/

static bool stm32_fdcan_txempty(struct can_dev_s *dev)
{
  struct stm32_fdcan_s *priv = (struct stm32_fdcan_s *)dev->cd_priv;
  uint32_t txfqs;

  /* Read TX FIFO/Queue Status register */
  txfqs = getreg32(priv->base + STM32_FDCAN_TXFQS_OFFSET);

  /* Check free level - if it equals TX FIFO size (32), then FIFO is empty */
  uint32_t free_level = (txfqs & FDCAN_TXFQS_TFFL_MASK) >> FDCAN_TXFQS_TFFL_SHIFT;

  return (free_level == 32);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_caninitialize
 *
 * Description:
 *   Initialize the selected FDCAN port as a character device
 *
 * Input Parameters:
 *   port - CAN port number (1, 2, or 3)
 *
 * Returned Value:
 *   Valid CAN device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct can_dev_s *stm32_caninitialize(int port)
{
	//printf("stm32_caninitialize called with port %d\n", port);
  struct can_dev_s *dev = NULL;
  struct stm32_fdcan_s *priv = NULL;

  caninfo("FDCAN%d: Initializing\n", port);

  /* Select the requested CAN device */

#ifdef CONFIG_STM32H7_CAN1
  if (port == 1)
    {
      dev = &g_fdcan1dev;
      priv = &g_fdcan1priv;
    }
#endif

#ifdef CONFIG_STM32H7_CAN2
  if (port == 2)
    {
      dev = &g_fdcan2dev;
      priv = &g_fdcan2priv;
    }
#endif

  if (dev == NULL)
    {
      canerr("ERROR: Unsupported port %d\n", port);
      return NULL;
    }

  /* Initialize the device structure */

  priv->port = port;

  /* Calculate message RAM offsets
   * Each FDCAN interface gets 1280 words (5120 bytes) of the total
   * 2560 words (10240 bytes) available message RAM
   *
   * Layout per interface:
   * - Standard ID filters: 128 words (offset 0 for FDCAN1, 1280 for FDCAN2)
   * - Rx FIFO 0: 256 words (64 elements × 4 words)
   * - Tx FIFO: 256 words (64 elements × 4 words)
   */

  priv->rxmsgram_offset = (port - 1) * 1280;
  priv->txmsgram_offset = priv->rxmsgram_offset + 128 + 256;  /* After filters + RX FIFO */

  caninfo("FDCAN%d: Device structure initialized\n", port);
  caninfo("  Base: 0x%08lx\n", priv->base);
  caninfo("  Baud: %lu\n", priv->baud);
  caninfo("  RX RAM offset: %lu words\n", priv->rxmsgram_offset);
  caninfo("  TX RAM offset: %lu words\n", priv->txmsgram_offset);

  return dev;
}

/****************************************************************************
 * Name: stm32_fdcan_loopback
 *
 * Description:
 *   Enable or disable internal loopback mode for testing.
 *   In loopback mode, transmitted messages are internally routed to RX
 *   without going to the physical bus. This allows testing without external
 *   CAN devices.
 *
 * Input Parameters:
 *   port   - The FDCAN port number (1 or 2)
 *   enable - true to enable loopback, false to disable
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int stm32_fdcan_loopback(int port, bool enable)
{
  struct stm32_fdcan_s *priv = NULL;
  irqstate_t flags;
  uint32_t cccr;

#ifdef CONFIG_STM32H7_CAN1
  if (port == 1)
    {
      priv = &g_fdcan1priv;
    }
#endif

#ifdef CONFIG_STM32H7_CAN2
  if (port == 2)
    {
      priv = &g_fdcan2priv;
    }
#endif

  if (priv == NULL)
    {
      canerr("ERROR: Invalid port %d\n", port);
      return -EINVAL;
    }

  caninfo("FDCAN%d: %s loopback mode\n", priv->port, enable ? "Enabling" : "Disabling");

  flags = enter_critical_section();

  /* Must be in INIT mode to change TEST.LBCK */
  cccr = getreg32(priv->base + STM32_FDCAN_CCCR_OFFSET);
  if (!(cccr & FDCAN_CCCR_INIT))
    {
      /* Enter INIT mode */
      modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_INIT);

      /* Wait for INIT mode */
      int timeout = 1000;
      while (!(getreg32(priv->base + STM32_FDCAN_CCCR_OFFSET) & FDCAN_CCCR_INIT) && timeout-- > 0)
        {
          up_udelay(10);
        }

      if (timeout <= 0)
        {
          canerr("Failed to enter INIT mode for loopback configuration\n");
          leave_critical_section(flags);
          return -ETIMEDOUT;
        }
    }

  /* Enable CCE to allow configuration */
  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_CCE);

  /* Set TEST mode bit to access TEST register */
  if (enable)
    {
      modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_TEST);

      /* Enable loopback mode in TEST register */
      modifyreg32(priv->base + STM32_FDCAN_TEST_OFFSET, 0, FDCAN_TEST_LBCK);

      /* DON'T enable MON - try loopback without bus monitoring to get data storage */
      /* modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, 0, FDCAN_CCCR_MON); */
      caninfo("FDCAN%d: Loopback enabled WITHOUT MON mode\n", priv->port);
    }
  else
    {
      /* Disable loopback mode */
      modifyreg32(priv->base + STM32_FDCAN_TEST_OFFSET, FDCAN_TEST_LBCK, 0);

      /* Disable MON and TEST modes */
      modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET,
                  FDCAN_CCCR_TEST | FDCAN_CCCR_MON, 0);
    }

  /* Disable CCE */
  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_CCE, 0);

  /* Exit INIT mode to resume operation */
  modifyreg32(priv->base + STM32_FDCAN_CCCR_OFFSET, FDCAN_CCCR_INIT, 0);

  /* Wait for normal mode */
  int timeout = 1000;
  while ((getreg32(priv->base + STM32_FDCAN_CCCR_OFFSET) & FDCAN_CCCR_INIT) && timeout-- > 0)
    {
      up_udelay(10);
    }

  if (timeout <= 0)
    {
      canerr("Failed to exit INIT mode after loopback configuration\n");
      leave_critical_section(flags);
      return -ETIMEDOUT;
    }

  leave_critical_section(flags);

  uint32_t test = getreg32(priv->base + STM32_FDCAN_TEST_OFFSET);
  uint32_t cccr_final = getreg32(priv->base + STM32_FDCAN_CCCR_OFFSET);

  caninfo("FDCAN%d: Loopback mode %s - TEST=0x%08lx CCCR=0x%08lx (LBCK=%d MON=%d)\n",
          priv->port, enable ? "enabled" : "disabled", test, cccr_final,
          !!(test & FDCAN_TEST_LBCK), !!(cccr_final & FDCAN_CCCR_MON));

  return OK;
}

#endif /* CONFIG_CAN && (CONFIG_STM32H7_CAN1 || CONFIG_STM32H7_CAN2 || CONFIG_STM32H7_CAN3) */
