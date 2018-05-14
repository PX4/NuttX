/****************************************************************************************************
 * arch/arm/src/kinetis/chip/kinetis_dma.h
 *
 *   Copyright (C) 2011, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_CHIP_KINETIS_DMA_H
#define __ARCH_ARM_SRC_KINETIS_CHIP_KINETIS_DMA_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

#define KINETIS_DMA_CR_OFFSET             0x0000 /* Control Register */
#define KINETIS_DMA_ES_OFFSET             0x0004 /* Error Status Register */
#define KINETIS_DMA_ERQ_OFFSET            0x000c /* Enable Request Register */
#define KINETIS_DMA_EEI_OFFSET            0x0014 /* Enable Error Interrupt Register */
#define KINETIS_DMA_CEEI_OFFSET           0x0018 /* Clear Enable Error Interrupt Register */
#define KINETIS_DMA_SEEI_OFFSET           0x0019 /* Set Enable Error Interrupt Register */
#define KINETIS_DMA_CERQ_OFFSET           0x001a /* Clear Enable Request Register */
#define KINETIS_DMA_SERQ_OFFSET           0x001b /* Set Enable Request Register */
#define KINETIS_DMA_CDNE_OFFSET           0x001c /* Clear DONE Status Bit Register */
#define KINETIS_DMA_SSRT_OFFSET           0x001d /* Set START Bit Register */
#define KINETIS_DMA_CERR_OFFSET           0x001e /* Clear Error Register */
#define KINETIS_DMA_CINT_OFFSET           0x001f /* Clear Interrupt Request Register */
#define KINETIS_DMA_INT_OFFSET            0x0024 /* Interrupt Request Register */
#define KINETIS_DMA_ERR_OFFSET            0x002c /* Error Register */
#define KINETIS_DMA_HRS_OFFSET            0x0034 /* Hardware Request Status Register */

#define KINETIS_DMA_DCHPRI_OFFSET(n)      0x0100 + (n - (n % 4)) + (3 - (n % 4)) /* Channel n Priority Register */

#define KINETIS_DMA_TCD_OFFSET(n)       (0x0000+((n) << 5))
#define KINETIS_DMA_TCD_SADDR_OFFSET    0x0000 /* TCD Source Address */
#define KINETIS_DMA_TCD_SOFF_OFFSET     0x0004 /* TCD Signed Source Address Offset */
#define KINETIS_DMA_TCD_ATTR_OFFSET     0x0006 /* TCD Transfer Attributes */
#define KINETIS_DMA_TCD_NBYTES_OFFSET   0x0008 /* TCD Minor Byte Count */
#define KINETIS_DMA_TCD_SLAST_OFFSET    0x000c /* TCD Last Source Address Adjustment */
#define KINETIS_DMA_TCD_DADDR_OFFSET    0x0010 /* TCD Destination Address */
#define KINETIS_DMA_TCD_DOFF_OFFSET     0x0014 /* TCD Signed Destination Address Offset */
#define KINETIS_DMA_TCD_CITER_OFFSET    0x0016 /* TCD Current Minor Loop Link, Major Loop Count */
#define KINETIS_DMA_TCD_DLASTSGA_OFFSET 0x0018 /* TCD Last Destination Address Adjustment/Scatter Gather Address */
#define KINETIS_DMA_TCD_CSR_OFFSET      0x001c /* TCD Control and Status */
#define KINETIS_DMA_TCD_BITER_OFFSET    0x001e /* TCD Beginning Minor Loop Link, Major Loop Count */

/* Register Addresses *******************************************************************************/

#define KINETIS_DMA_CR                    (KINETIS_DMAC_BASE+KINETIS_DMA_CR_OFFSET)
#define KINETIS_DMA_ES                    (KINETIS_DMAC_BASE+KINETIS_DMA_ES_OFFSET)
#define KINETIS_DMA_ERQ                   (KINETIS_DMAC_BASE+KINETIS_DMA_ERQ_OFFSET)
#define KINETIS_DMA_EEI                   (KINETIS_DMAC_BASE+KINETIS_DMA_EEI_OFFSET)
#define KINETIS_DMA_CEEI                  (KINETIS_DMAC_BASE+KINETIS_DMA_CEEI_OFFSET)
#define KINETIS_DMA_SEEI                  (KINETIS_DMAC_BASE+KINETIS_DMA_SEEI_OFFSET)
#define KINETIS_DMA_CERQ                  (KINETIS_DMAC_BASE+KINETIS_DMA_CERQ_OFFSET)
#define KINETIS_DMA_SERQ                  (KINETIS_DMAC_BASE+KINETIS_DMA_SERQ_OFFSET)
#define KINETIS_DMA_CDNE                  (KINETIS_DMAC_BASE+KINETIS_DMA_CDNE_OFFSET)
#define KINETIS_DMA_SSRT                  (KINETIS_DMAC_BASE+KINETIS_DMA_SSRT_OFFSET)
#define KINETIS_DMA_CERR                  (KINETIS_DMAC_BASE+KINETIS_DMA_CERR_OFFSET)
#define KINETIS_DMA_CINT                  (KINETIS_DMAC_BASE+KINETIS_DMA_CINT_OFFSET)
#define KINETIS_DMA_INT                   (KINETIS_DMAC_BASE+KINETIS_DMA_INT_OFFSET)
#define KINETIS_DMA_ERR                   (KINETIS_DMAC_BASE+KINETIS_DMA_ERR_OFFSET)
#define KINETIS_DMA_HRS                   (KINETIS_DMAC_BASE+KINETIS_DMA_HRS_OFFSET)

#define KINETIS_DMA_DCHPRI(n)             (KINETIS_DMAC_BASE+KINETIS_DMA_DCHPRI_OFFSET(n))

#define KINETIS_DMA_TCD_BASE(n)         (KINETIS_DMADESC_BASE+KINETIS_DMA_TCD_OFFSET(n))

#define KINETIS_DMA_TCD_SADDR(n)        (KINETIS_DMA_TCD_BASE(n)+KINETIS_DMA_TCD_SADDR_OFFSET)
#define KINETIS_DMA_TCD_SOFF(n)         (KINETIS_DMA_TCD_BASE(n)+KINETIS_DMA_TCD_SOFF_OFFSET)
#define KINETIS_DMA_TCD_ATTR(n)         (KINETIS_DMA_TCD_BASE(n)+KINETIS_DMA_TCD_ATTR_OFFSET)
#define KINETIS_DMA_TCD_NBYTES(n)       (KINETIS_DMA_TCD_BASE(n)+KINETIS_DMA_TCD_NBYTES_OFFSET)
#define KINETIS_DMA_TCD_SLAST(n)        (KINETIS_DMA_TCD_BASE(n)+KINETIS_DMA_TCD_SLAST_OFFSET)
#define KINETIS_DMA_TCD_DADDR(n)        (KINETIS_DMA_TCD_BASE(n)+KINETIS_DMA_TCD_DADDR_OFFSET)
#define KINETIS_DMA_TCD_DOFF(n)         (KINETIS_DMA_TCD_BASE(n)+KINETIS_DMA_TCD_DOFF_OFFSET)
#define KINETIS_DMA_TCD_CITER(n)        (KINETIS_DMA_TCD_BASE(n)+KINETIS_DMA_TCD_CITER_OFFSET)
#define KINETIS_DMA_TCD_DLASTSGA(n)     (KINETIS_DMA_TCD_BASE(n)+KINETIS_DMA_TCD_DLASTSGA_OFFSET)
#define KINETIS_DMA_TCD_CSR(n)          (KINETIS_DMA_TCD_BASE(n)+KINETIS_DMA_TCD_CSR_OFFSET)
#define KINETIS_DMA_TCD_BITER(n)        (KINETIS_DMA_TCD_BASE(n)+KINETIS_DMA_TCD_BITER_OFFSET)

/* Register Bit Definitions *************************************************************************/

/* Control Register (32-bit) */                                                    /* Bit 0:  Reserved */
#define DMA_CR_EDBG                       (1 << 1)  /* Bit 1:  Enable debug */
#define DMA_CR_ERCA                       (1 << 2)  /* Bit 2:  Enable round robin channel arbitration */
#if defined  KINETIS_DMA_HAS_CR_ERGA
#  define DMA_CR_ERGA                     (1 << 3)  /* Bit 3:  Enable round robin group arbitration */
#endif
#define DMA_CR_HOE                        (1 << 4)  /* Bit 4:  Halt on error */
#define DMA_CR_HALT                       (1 << 5)  /* Bit 5:  Halt DMA operations */
#define DMA_CR_CLM                        (1 << 6)  /* Bit 6:  Continuous link mode */
#define DMA_CR_EMLM                       (1 << 7)  /* Bit 7:  Enable minor loop mapping */
#ifdef KINETIS_DMA_HAS_CR_GRP0PRI
#   define DMA_CR_GRP0PRI                 (1 << 8)  /* Bit 8:  Channel Group 0 Priority */
#endif
                                                    /* Bit 9: Reserved */
#ifdef KINETIS_DMA_HAS_CR_GRP1PRI
#  define DMA_CR_GRP1PRI                  (1 << 10) /* Bit 10: Channel Group 1 Priority */
#endif
                                                    /* Bits 11-15: Reserved */
#define DMA_CR_ECX                        (1 << 16) /* Bit 16: Error cancel transfer */
#define DMA_CR_CX                         (1 << 17) /* Bit 17: Cancel transfer */
                                                    /* Bits 18-31: Reserved */

/* Error Status Register */
#define DMA_ES_DBE                        (1 << 0)  /* Bit 0:  Destination bus error */
#define DMA_ES_SBE                        (1 << 1)  /* Bit 1:  Source bus error */
#define DMA_ES_SGE                        (1 << 2)  /* Bit 2:  Scatter/gather configuration error */
#define DMA_ES_NCE                        (1 << 3)  /* Bit 3:  NBYTES/CITER configuration error */
#define DMA_ES_DOE                        (1 << 4)  /* Bit 4:  Destination offset error */
#define DMA_ES_DAE                        (1 << 5)  /* Bit 5:  Destination address error */
#define DMA_ES_SOE                        (1 << 6)  /* Bit 6:  Source offset error */
#define DMA_ES_SAE                        (1 << 7)  /* Bit 7:  Source address error */
#define DMA_ES_ERRCHN_SHIFT               (8)       /* Bits 8-11/12: Error channel number or cancelled channel number */
#define DMA_ES_ERRCHN_MASK                (((1 << KINETIS_DMA_HAS_ES_ERRCHN_BITS) - 1) << DMA_ES_ERRCHN_SHIFT)
                                                    /* Bits 13: Reserved */
#define DMA_ES_CPE                        (1 << 14) /* Bit 14:  Channel priority error */
#ifdef KINETIS_DMA_HAS_ES_GPE
#define DMA_ES_GPE                        (1 << 15) /* Bit 15:  Group priority error */
#endif
#define DMA_ES_ECX                        (1 << 16) /* Bit 16:  Transfer cancelled */
                                                    /* Bits 17-30: Reserved */
#define DMA_ES_VLD                        (1 << 31) /* Bit 31:  Logical OR of all ERR status bits */

/* Enable Request Register (ERQ), Enable Error Interrupt Register (EEI), Interrupt Request Register (INT),
 * Error Register (ERR), Hardware Request Status Register (HRS) common bit definitions
 */
#define DMA_REQ(n)                        (1 << (n)) /* Bit n: DMA Request n, n=0..<KINETIS_NDMACH */
                                                     /* Bits KINETIS_NDMACH-31: Reserved */

/* Clear Enable Error Interrupt Register (8-bit) */
#define DMA_CEEI_SHIFT                    (0)       /* Bits 0-3/4: Clear enable error interrupt */
#define DMA_CEEI_MASK                     (((1 << KINETIS_DMA_HAS_CEEI_CEEI_BITS) - 1) << DMA_CEEI_SHIFT)
                                                    /* Bits 5: Reserved */
#define DMA_CEEI_CAEE                     (1 << 6)  /* Bit 6:  Clear all enable error interrupts */
#define DMA_CEEI_NOP                      (1 << 7)  /* Bit 7:  No operation */

/* Set Enable Error Interrupt Register (8-bit) */
#define DMA_SEEI_SHIFT                    (0)       /* Bits 0-3/4: Set enable error interrupt */
#define DMA_SEEI_MASK                     (((1 << KINETIS_DMA_HAS_SEEI_SEEI_BITS) - 1) << DMA_SEEI_SHIFT)
                                                    /* Bits 5: Reserved */
#define DMA_SEEI_SAEE                     (1 << 6)  /* Bit 6:  Set all enable error interrupts */
#define DMA_SEEI_NOP                      (1 << 7)  /* Bit 7:  No operation */

/* Clear Enable Request Register (8-bit) */
#define DMA_CERQ_SHIFT                    (0)       /* Bits 0-3: Clear enable request */
#define DMA_CERQ_MASK                     (((1 << KINETIS_DMA_HAS_CERQ_CERQ_BITS) - 1) << DMA_CERQ_SHIFT)
                                                    /* Bits 4-5: Reserved */
#define DMA_CERQ_CAER                     (1 << 6)  /* Bit 6:  Clear all enable requests */
#define DMA_CERQ_NOP                      (1 << 7)  /* Bit 7:  No operation */

/* Set Enable Request Register (8-bit) */
#define DMA_SERQ_SHIFT                    (0)       /* Bits 0-3: Set enable request */
#define DMA_SERQ_MASK                     (((1 << KINETIS_DMA_HAS_SERQ_SERQ_BITS) - 1) << DMA_SERQ_SHIFT)
                                                    /* Bits 4-5: Reserved */
#define DMA_SERQ_SAER                     (1 << 6)  /* Bit 6:  Set all enable requests */
#define DMA_SERQ_NOP                      (1 << 7)  /* Bit 7:  No operation */

/* Clear DONE Status Bit Register (8-bit) */
#define DMA_CDNE_SHIFT                    (0)       /* Bits 0-3: Clear DONE bit */
#define DMA_CDNE_MASK                     (((1 << KINETIS_DMA_HAS_CDNE_CDNE_BITS) - 1) << DMA_CDNE_SHIFT)
                                                    /* Bits 4-5: Reserved */
#define DMA_CDNE_CADN                     (1 << 6)  /* Bit 6:  Clears all DONE bits */
#define DMA_CDNE_NOP                      (1 << 7)  /* Bit 7:  No operation */

/* Set START Bit Register (8-bit) */
#define DMA_SSRT_SHIFT                    (0)       /* Bits 0-3: Set START bit */
#define DMA_SSRT_MASK                     (((1 << KINETIS_DMA_HAS_SSRT_SSRT_BITS) - 1) << DMA_SSRT_SHIFT)
                                                    /* Bits 4-5: Reserved */
#define DMA_SSRT_SAST                     (1 << 6)  /* Bit 6:  Set all START bits (activates all channels) */
#define DMA_SSRT_NOP                      (1 << 7)  /* Bit 7:  No operation */

/* Clear Error Register (8-bit) */
#define DMA_CERR_SHIFT                    (0)       /* Bits 0-3: Clear error indicator */
#define DMA_CERR_MASK                     (((1 << KINETIS_DMA_HAS_CERR_CERR_BITS) - 1) << DMA_CERR_SHIFT)
                                                    /* Bits 4-5: Reserved */
#define DMA_CERR_CAEI                     (1 << 6)  /* Bit 6:  Clear all error indicators */
#define DMA_CERR_NOP                      (1 << 7)  /* Bit 7:  No operation */

/* Clear Interrupt Request Register (8-bit) */
#define DMA_CINT_SHIFT                    (0)       /* Bits 0-3: Clear interrupt request */
#define DMA_CINT_MASK                     (((1 << KINETIS_DMA_HAS_CINT_CINT_BITS) - 1) << DMA_CINT_SHIFT)
                                                    /* Bits 4-5: Reserved */
#define DMA_CINT_CAIR                     (1 << 6)  /* Bit 6:  Clear all interrupt requests */
#define DMA_CINT_NOP                      (1 << 7)  /* Bit 7:  No operation */

/* Channel n Priority Register (8-bit) */
#define DMA_DCHPR_SHIFT                   (0)       /* Bits 0-3: Channel n arbitration priority */
#define DMA_DCHPR_MASK                    (((1 << KINETIS_DMA_HAS_DCHPRI_CHPRI_BITS) - 1) << DMA_DCHPR_SHIFT)
#ifdef KINETIS_DMA_HAS_DCHPRI_GRPPRI
#  define DMA_DCHPR_GRPPRI                  (1 << 4)  /* Bits 4-5: Channel n Current Group Priority */
#endif
#define DMA_DCHPR_DPA                     (1 << 6)  /* Bit 6:  Disable preempt ability */
#define DMA_DCHPR_ECP                     (1 << 7)  /* Bit 7:  Enable channel preemption */

/* Enable Asynchronous Request in Stop Register (32-bit) */
#ifdef KINETIS_DMA_HAS_EARS

#define DMA_EARS(n)                        (1 << (n)) /* Bit n: DMA EARS n, n=0..<KINETIS_NDMACH */

#endif /* KINETIS_DMA_HAS_EARS */

/* TCD Source Address.  32-bit address value. */
/* TCD Signed Source Address Offset.  32-bit offset value. */

/* TCD Transfer Attributes (16-bit) */

#define DMA_TCD_ATTR_DSIZE_SHIFT          (0)       /* Bits 0-2: Destination data transfer size */
#define DMA_TCD_ATTR_DSIZE_MASK           (7 << DMA_TCD_ATTR_DSIZE_SHIFT)
#  define DMA_TCD_ATTR_DSIZE_8BIT         (0 << DMA_TCD_ATTR_DSIZE_SHIFT) /* 8-bit */
#  define DMA_TCD_ATTR_DSIZE_16BIT        (1 << DMA_TCD_ATTR_DSIZE_SHIFT) /* 16-bit */
#  define DMA_TCD_ATTR_DSIZE_32BIT        (2 << DMA_TCD_ATTR_DSIZE_SHIFT) /* 32-bit */
#  define DMA_TCD_ATTR_DSIZE_16BYTE       (4 << DMA_TCD_ATTR_DSIZE_SHIFT) /* 16-byte */
#define DMA_TCD_ATTR_DMOD_SHIFT           (3)       /* Bits 3-7: Destination address modulo */
#define DMA_TCD_ATTR_DMOD_MASK            (31 << DMA_TCD_ATTR_DMOD_SHIFT)
#define DMA_TCD_ATTR_SSIZE_SHIFT          (8)       /* Bits 8-10: Source data transfer size */
#define DMA_TCD_ATTR_SSIZE_MASK           (7 << DMA_TCD_ATTR_SSIZE_SHIFT)
#  define DMA_TCD_ATTR_SSIZE_8BIT         (0 << DMA_TCD_ATTR_SSIZE_SHIFT) /* 8-bit */
#  define DMA_TCD_ATTR_SSIZE_16BIT        (1 << DMA_TCD_ATTR_SSIZE_SHIFT) /* 16-bit */
#  define DMA_TCD_ATTR_SSIZE_32BIT        (2 << DMA_TCD_ATTR_SSIZE_SHIFT) /* 32-bit */
#  define DMA_TCD_ATTR_SSIZE_16BYTE       (4 << DMA_TCD_ATTR_SSIZE_SHIFT) /* 16-byte */
#define DMA_TCD_ATTR_SMOD_SHIFT           (11)      /* Bits 11-15: Source address modulo */
#define DMA_TCD_ATTR_SMOD_MASK            (31 << DMA_TCD_ATTR_SMOD_SHIFT)

/* TCD Minor Byte Count.
 * Case 1: Minor Loop Disabled.  In this case, the register holds a simple 32-bit count value.
 * Case 2: Minor Loop Enabled and Offset Disabled:
 */

#define DMA_TCD_NBYTES2_SHIFT             (0)       /* Bits 0-29: Minor byte transfer count */
#define DMA_TCD_NBYTES2_MASK              (0x3fffffff)
#define DMA_TCD_NBYTES_DMLOE              (1 << 30) /* Bit 30: Destination minor loop offset enable (Case 2&3) */
#define DMA_TCD_NBYTES_SMLOE              (1 << 31) /* Bit 31: Source minor loop offset enable (Case 2&3) */

/* Case 3: (Minor Loop and Offset Enabled): */

#define DMA_TCD_NBYTES3_SHIFT             (0)       /* Bits 0-9: Minor byte transfer count */
#define DMA_TCD_NBYTES3_MASK              (0x3ff << DMA_TCD_NBYTES3_SHIFT)
#define DMA_TCD_NBYTES_MLOFF_SHIFT        (10)      /* Bits 10-29: Sign-extended address offset */
#define DMA_TCD_NBYTES_MLOFF_MASK         (0xfffff << DMA_TCD_NBYTES_MLOFF_SHIFT)
                                                    /* Bit 30: Same as Case 2 */
                                                    /* Bit 31: Same as Case 2 */

/* TCD Last Source Address Adjustment. 32-bit address value. */
/* TCD Destination Address. 32-bit address value. */
/* TCD Signed Destination Address Offset. 32-bit offset value. */

/* TCD Current Minor Loop Link, Major Loop Count. 16-bit.
 * Case 1:  Channel Linking Enabled:
 */

#define DMA_TCD_CITER1_SHIFT              (0)       /* Bits 0-8: Current major iteration count */
#define DMA_TCD_CITER1_MASK               (0x1ff << DMA_TCD_CITER1_SHIFT)
#define DMA_TCD_CITER1_LINKCH_SHIFT       (9)       /* Bits 9-12/13: Link channel number */
#define DMA_TCD_CITER1_LINKCH_MASK        (((1 << KINETIS_DMA_HAS_TCD_CITER1_LINKCH_BITS) - 1) << DMA_TCD_CITER1_LINKCH_SHIFT)
                                                    /* Bits 14: Reserved */
#define DMA_TCD_CITER_ELINK               (1 << 15) /* Bit 15: Enable channel-to-channel linking on minor-loop complete (Case 1&2) */

/* Case 2:  Channel Linking Disabled: */

#define DMA_TCD_CITER2_SHIFT              (0)       /* Bits 0-14: Current major iteration count */
#define DMA_TCD_CITER2_MASK               (0x7fff << DMA_TCD_CITER2_SHIFT)
                                                    /* Bits 15: Same as Case 1 */

/* TCD Last Destination Address Adjustment/Scatter Gather Address. 32-bit address value. */

/* TCD Control and Status (16-bit) */

#define DMA_TCD_CSR_START                 (1 << 0)  /* Bit 0:  Channel start */
#define DMA_TCD_CSR_INTMAJOR              (1 << 1)  /* Bit 1:  Enable an interrupt when major iteration count completes */
#define DMA_TCD_CSR_INTHALF               (1 << 2)  /* Bit 2:  Enable an interrupt when major counter is half complete */
#define DMA_TCD_CSR_DREQ                  (1 << 3)  /* Bit 3:  Disable request */
#define DMA_TCD_CSR_ESG                   (1 << 4)  /* Bit 4:  Enable scatter/gather processing */
#define DMA_TCD_CSR_MAJORELINK            (1 << 5)  /* Bit 5:  Enable channel-to-channel linking on major loop complete */
#define DMA_TCD_CSR_ACTIVE                (1 << 6)  /* Bit 6:  Channel active */
#define DMA_TCD_CSR_DONE                  (1 << 7)  /* Bit 7:  Channel done */
#define DMA_TCD_CSR_MAJORLINKCH_SHIFT     (8)       /* Bits 8-11/12: Link channel number */
#define DMA_TCD_CSR_MAJORLINKCH_MASK      (((1 << KINETIS_DMA_HAS_TCD_CSR_MAJORLINKCH_BITS) - 1) << DMA_TCD_CSR_MAJORLINKCH_SHIFT)
                                                    /* Bits 13: Reserved */
#define DMA_TCD_CSR_BWC_SHIFT             (14)      /* Bits 14-15: Bandwidth control */
#define DMA_TCD_CSR_BWC_MASK              (3 << DMA_TCD_CSR_BWC_SHIFT)
#  define DMA_TCD_CSR_BWC_NOSTALLS        (0 << DMA_TCD_CSR_BWC_SHIFT) /* No eDMA engine stalls */
#  define DMA_TCD_CSR_BWC_4CYCLES         (2 << DMA_TCD_CSR_BWC_SHIFT) /* eDMA engine stalls 4 cycles after each R/W */
#  define DMA_TCD_CSR_BWC_8CYCLES         (3 << DMA_TCD_CSR_BWC_SHIFT) /* eDMA engine stalls 8 cycles after each R/W */

/* TCD Beginning Minor Loop Link, Major Loop Count (16-bit).
 *
 * Case 1: Channel Linking Enabled:
 */

#define DMA_TCD_BITER1_SHIFT              (0)       /* Bits 0-8: Starting major iteration count */
#define DMA_TCD_BITER1_MASK               (0x1ff << DMA_TCD_BITER1_SHIFT)
#define DMA_TCD_BITER1_LINKCH_SHIFT       (9)       /* Bits 9-12: Link channel number */
#define DMA_TCD_BITER1_LINKCH_MASK        (((1 << KINETIS_DMA_HAS_TCD_BITER1_LINKCH_BITS) - 1) << DMA_TCD_BITER1_LINKCH_SHIFT)
                                                    /* Bits 13-14: Reserved */
#define DMA_TCD_BITER_ELINK               (1 << 15) /* Bit 15: Enable channel-to-channel linking on minor-loop complete (Case 1&2) */

/* Case 2: Channel Linking Disabled: */

#define DMA_TCD_BITER2_SHIFT              (0)       /* Bits 0-14: Starting major iteration count */
#define DMA_TCD_BITER2_MASK               (0x7fff << DMA_TCD_CITER2_SHIFT)
                                                    /* Bits 15: Same as Case 1 */

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_CHIP_KINETIS_DMA_H */
