/****************************************************************************
 * arch/arm/src/kinetis/kinetis_dma.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETEIS_KINETEIS_DMA_H
#define __ARCH_ARM_SRC_KINETEIS_KINETEIS_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include "chip/kinetis_dma.h"

/****************************************************************************
 * Pre-processor Declarations
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef FAR void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, void *arg, int result);

/* The following is used for sampling DMA registers when CONFIG DEBUG_DMA is selected */

#ifdef CONFIG_DEBUG_DMA
struct kinetis_dmaglobalregs_s
{
#warning "Missing logic"
  /* Global Registers */
};

struct kinetis_dmachanregs_s
{
#warning "Missing logic"
  /* Channel Registers */
};

struct kinetis_dmaregs_s
{
  /* Global Registers */

  struct kinetis_dmaglobalregs_s gbl;

  /* Channel Registers */

  struct kinetis_dmachanregs_s   ch;
};
#endif

typedef enum _KINETIS_DMA_REQUEST_SRC {
  KINETIS_DMA_REQUEST_SRC_UART0_RX = 2,
  KINETIS_DMA_REQUEST_SRC_UART0_TX = 3,
  KINETIS_DMA_REQUEST_SRC_UART1_RX = 4,
  KINETIS_DMA_REQUEST_SRC_UART1_TX = 5,
  KINETIS_DMA_REQUEST_SRC_UART2_RX = 6,
  KINETIS_DMA_REQUEST_SRC_UART2_TX = 7,
  KINETIS_DMA_REQUEST_SRC_UART3_RX = 8,
  KINETIS_DMA_REQUEST_SRC_UART3_TX = 9,
  KINETIS_DMA_REQUEST_SRC_UART4_RXTX = 10,
  KINETIS_DMA_REQUEST_SRC_I2S0_RX = 12,
  KINETIS_DMA_REQUEST_SRC_I2S0_TX = 13,
  KINETIS_DMA_REQUEST_SRC_SPI0_RX = 14,
  KINETIS_DMA_REQUEST_SRC_SPI0_TX = 15,
  KINETIS_DMA_REQUEST_SRC_SPI1_RX = 16,
  KINETIS_DMA_REQUEST_SRC_SPI1_TX = 17,
  KINETIS_DMA_REQUEST_SRC_I2C0__I2C3 = 18,
  KINETIS_DMA_REQUEST_SRC_I2C1__I2C2 = 19,
  KINETIS_DMA_REQUEST_SRC_FTM0_CH0 = 20,
  KINETIS_DMA_REQUEST_SRC_FTM0_CH1 = 21,
  KINETIS_DMA_REQUEST_SRC_FTM0_CH2 = 22,
  KINETIS_DMA_REQUEST_SRC_FTM0_CH3 = 23,
  KINETIS_DMA_REQUEST_SRC_FTM0_CH4 = 24,
  KINETIS_DMA_REQUEST_SRC_FTM0_CH5 = 25,
  KINETIS_DMA_REQUEST_SRC_FTM0_CH6 = 26,
  KINETIS_DMA_REQUEST_SRC_FTM0_CH7 = 27,
  KINETIS_DMA_REQUEST_SRC_FTM1_TPM1_CH0 = 28,
  KINETIS_DMA_REQUEST_SRC_FTM1_TPM1_CH1 = 29,
  KINETIS_DMA_REQUEST_SRC_FTM2_TPM2_CH0 = 30,
  KINETIS_DMA_REQUEST_SRC_FTM2_TPM2_CH1 = 31,
  KINETIS_DMA_REQUEST_SRC_FTM3_CH0 = 32,
  KINETIS_DMA_REQUEST_SRC_FTM3_CH1 = 33,
  KINETIS_DMA_REQUEST_SRC_FTM3_CH2 = 34,
  KINETIS_DMA_REQUEST_SRC_FTM3_CH3 = 35,
  KINETIS_DMA_REQUEST_SRC_FTM3_CH4 = 36,
  KINETIS_DMA_REQUEST_SRC_FTM3_CH5 = 37,
  KINETIS_DMA_REQUEST_SRC_FTM3_CH6__SPI2_RX = 38,
  KINETIS_DMA_REQUEST_SRC_FTM3_CH7__SPI2_TX = 39,
  KINETIS_DMA_REQUEST_SRC_ADC0 = 40,
  KINETIS_DMA_REQUEST_SRC_ADC1 = 41,
  KINETIS_DMA_REQUEST_SRC_CMP0 = 42,
  KINETIS_DMA_REQUEST_SRC_CMP1 = 43,
  KINETIS_DMA_REQUEST_SRC_CMP2__CMP3 = 44,
  KINETIS_DMA_REQUEST_SRC_DAC0 = 45,
  KINETIS_DMA_REQUEST_SRC_DAC1 = 46,
  KINETIS_DMA_REQUEST_SRC_CMT = 47,
  KINETIS_DMA_REQUEST_SRC_PDB = 48,
  KINETIS_DMA_REQUEST_SRC_PCM_A = 49,
  KINETIS_DMA_REQUEST_SRC_PCM_B = 50,
  KINETIS_DMA_REQUEST_SRC_PCM_C = 51,
  KINETIS_DMA_REQUEST_SRC_PCM_D = 52,
  KINETIS_DMA_REQUEST_SRC_PCM_E = 53,
  KINETIS_DMA_REQUEST_SRC_TIMER0 = 54,
  KINETIS_DMA_REQUEST_SRC_TIMER1 = 55,
  KINETIS_DMA_REQUEST_SRC_TIMER2 = 56,
  KINETIS_DMA_REQUEST_SRC_TIMER3 = 57,
  KINETIS_DMA_REQUEST_SRC_LPUART0_RX = 58,
  KINETIS_DMA_REQUEST_SRC_LPUART0_TX = 59,
} KINETIS_DMA_REQUEST_SRC;

typedef enum _KINETIS_DMA_DIRECTION {
  KINETIS_DMA_DIRECTION_PERIPHERAL_TO_MEMORY,
  KINETIS_DMA_DIRECTION_MEMORY_TO_PERIPHERAL
} KINETIS_DMA_DIRECTION;

/* Kinetis data transfer size */
typedef enum _KINETIS_DMA_DATA_SZ {
  KINETIS_DMA_DATA_SZ_8BIT = 0,
  KINETIS_DMA_DATA_SZ_16BIT = 1,
  KINETIS_DMA_DATA_SZ_32BIT = 2,
} KINETIS_DMA_DATA_SZ;

typedef struct _kinetis_dmachannel_config {
  bool circular;                ///< Circular DMA
  bool halfcomplete_interrupt;  ///< Enables an interrupt to be triggered when half of the bytes have been transferred
} kinetis_dmachannel_config;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif


/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: kinetis_dmainitialize
 *
 * Description:
 *   Initialize the GPDMA subsystem.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void kinetis_dmainitialize(void);

/****************************************************************************
 * Name: kinetis_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and
 *   gives the caller exclusive access to the DMA channel.
 *
 * Input Parameters:
 *   src  - DMA request source
 *   per_addr - Address of the peripheral data
 *   per_data_sz - Peripheral data size (register size). Note that if this does not
 *                 agree with the peripheral register size, DMA transfers will
 *                 silently fail during operation.
 *   dir - transfer direction
 *
 * Returned Value:
 *   One success, this function returns a non-NULL, void* DMA channel
 *   handle.  NULL is returned on any failure.  This function can fail only
 *   if no DMA channel is available.
 *
 ****************************************************************************/

DMA_HANDLE kinetis_dmachannel(KINETIS_DMA_REQUEST_SRC src,
                              uint32_t per_addr,
                              KINETIS_DMA_DATA_SZ per_data_sz,
                              KINETIS_DMA_DIRECTION dir);

/****************************************************************************
 * Name: kinetis_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until kinetis_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void kinetis_dmafree(DMA_HANDLE handle);

/****************************************************************************
 * Name: kinetis_dmasetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 * Input Parameters:
 *   mem_addr  - Memory address
 *   ntransfers - Number of transfers. Must be 0<= ntransfers <= 0x7FFF
 *   config - Channel configuration
 *
 * Returned Value:
 *   result: 0 if ok, negative else
 *
 ****************************************************************************/
int kinetis_dmasetup(DMA_HANDLE handle, uint32_t mem_addr,
                     size_t ntransfers, const kinetis_dmachannel_config *config);


/****************************************************************************
 * Name: kinetis_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int kinetis_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);

/****************************************************************************
 * Name: kinetis_dmastop
 *
 * Description:
 *   Cancel the DMA.  After kinetis_dmastop() is called, the DMA channel is
 *   reset and kinetis_dmasetup() must be called before kinetis_dmastart() can be
 *   called again
 *
 ****************************************************************************/

void kinetis_dmastop(DMA_HANDLE handle);

/****************************************************************************
 * Name: kinetis_dmaresidual
 *
 * Description:
 *   Returns the number of transfers left
 *
 * Returned Value:
 *   Residual transfers
 ****************************************************************************/

size_t kinetis_dmaresidual(DMA_HANDLE handle);

/****************************************************************************
 * Name: kinetis_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void kinetis_dmasample(DMA_HANDLE handle, struct kinetis_dmaregs_s *regs);
#else
#  define kinetis_dmasample(handle,regs)
#endif

/****************************************************************************
 * Name: kinetis_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void kinetis_dmadump(DMA_HANDLE handle, const struct kinetis_dmaregs_s *regs,
                     const char *msg);
#else
#  define kinetis_dmadump(handle,regs,msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_KINETEIS_KINETEIS_DMA_H */
