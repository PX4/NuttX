/****************************************************************************
 *  arch/arm/src/kinetis/kinetis_dma.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#include "kinetis_config.h"
#include "chip.h"
#include "kinetis_dma.h"
#include "chip/kinetis_dmamux.h"
#include "chip/kinetis_sim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DMA_N_CHANNELS 32         // Total number of channels
#define DMA_CHN_PER_GROUP 16      // Number of channels per group

/****************************************************************************
 * Private Types
 ****************************************************************************/
 struct kinetis_dma_ch
 {
    bool                  used;
    uint8_t               ind;
    uint8_t               irq;
    KINETIS_DMA_DIRECTION dir;
    dma_callback_t        callback;
    void                  *arg;

    KINETIS_DMA_REQUEST_SRC src;
 };

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct kinetis_dma_ch channels[DMA_N_CHANNELS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int kinetis_dmainterrupt_int(int irq, void *context, struct kinetis_dma_ch *ch) {

  // Clear bit in the interrupt
  putreg8(ch->ind, KINETIS_DMA_CINT);

  // Invoke the callback
  if (ch->callback)
    {
      int result = 0; // todo
      ch->callback((DMA_HANDLE)&ch, ch->arg, result);
    }

  return OK;
}

static int kinetis_dmainterrupt(int irq, void *context, void *arg)
{
  int irq_int = *(int *)arg;
  uint32_t regval;
  regval = getreg32(KINETIS_DMA_INT);
  // Channel irq_int and irq_int + DMA_CHN_PER_GROUP use the same arg. Check which one requested an interrupt
  if ((regval & (1 << irq_int)) != 0)
    {
      kinetis_dmainterrupt_int(irq, context, &channels[irq_int]);
      // todo handle return value!!!
    }

  if ((regval & (1 << (irq_int + DMA_CHN_PER_GROUP))) != 0)
    {
      kinetis_dmainterrupt_int(irq, context, &channels[irq_int + DMA_CHN_PER_GROUP]);
      // todo handle return value!!!
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

size_t kinetis_dmaresidual(DMA_HANDLE handle)
{
  struct kinetis_dma_ch *ch = (struct kinetis_dma_ch *)handle;
  return getreg16(KINETIS_DMA_TCD_CITER(ch->ind)) & 0x7FFF;
}


/****************************************************************************
 * Name: kinetis_dmainitialize
 *
 * Description:
 *   Initialize the DMA subsystem.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function up_dmainitialize(void)
{
  int i;
  uint32_t regval;

  for (i = 0; i < DMA_N_CHANNELS; i++) {
    channels[i].ind = i;
    channels[i].used = false;
    channels[i].irq = KINETIS_IRQ_FIRST + (i % DMA_CHN_PER_GROUP);

    if (i < DMA_CHN_PER_GROUP) {
      /* Attach DMA interrupt */
      (void)irq_attach(channels[i].irq, kinetis_dmainterrupt, (void *)i);
    }
  }

  // Enable clocking for DMA
  regval  = getreg32(KINETIS_SIM_SCGC7);
  regval |= SIM_SCGC7_DMA;
  putreg32(regval, KINETIS_SIM_SCGC7);

  // Configure DMA for round robin arbitration
  regval = 0;
  regval |= DMA_CR_ERCA | DMA_CR_ERGA;
  putreg32(regval, KINETIS_DMA_CR);

  /* Enable clocking for the DMA mux*/
  regval  = getreg32(KINETIS_SIM_SCGC6);
  regval |= SIM_SCGC6_DMAMUX0;
  putreg32(regval, KINETIS_SIM_SCGC6);

}

/****************************************************************************
 * Name: kinetis_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and
 *   gives the caller exclusive access to the DMA channel.
 *
 * Returned Value:
 *   One success, this function returns a non-NULL, void* DMA channel
 *   handle.  NULL is returned on any failure.  This function can fail only
 *   if no DMA channel is available.
 *
 ****************************************************************************/

DMA_HANDLE kinetis_dmachannel(KINETIS_DMA_REQUEST_SRC src, uint32_t per_addr, KINETIS_DMA_DATA_SZ per_data_sz, KINETIS_DMA_DIRECTION dir)
{
  int i;
  int ch_ind = -1;
  uint8_t regval;
  irqstate_t flags;
  struct kinetis_dma_ch *ch;

  // Find available channel
  flags = enter_critical_section();
  for (i = 0; i < DMA_N_CHANNELS; i++) {
    if (!channels[i].used) {
      ch_ind = i;
      channels[ch_ind].used = true;
      break;
    }
  }
  leave_critical_section(flags);

  if (ch_ind == -1) {
    // No available channel
    return NULL;
  }
  ch = &channels[ch_ind];
  ch->dir = dir;

  /* DMAMUX */
  // Set DMA channel source and disable the mux for this channel
  ch->src = src;
  regval = 0;
  //regval |= (uint8_t)src; todo
  putreg8(regval, KINETIS_DMAMUX_CHCFG(ch_ind));

  /* DMA */
  // Set peripheral address in TCD
  if (ch->dir == KINETIS_DMA_DIRECTION_PERIPHERAL_TO_MEMORY) {
    putreg32(per_addr, KINETIS_DMA_TCD_SADDR(ch->ind));
    putreg16(0, KINETIS_DMA_TCD_SOFF(ch->ind));
    putreg16(((uint16_t)per_data_sz) << DMA_TCD_ATTR_SSIZE_SHIFT, KINETIS_DMA_TCD_ATTR(ch->ind));
  } else if (ch->dir == KINETIS_DMA_DIRECTION_MEMORY_TO_PERIPHERAL) {
    putreg32(per_addr, KINETIS_DMA_TCD_DADDR(ch->ind));
    putreg16(0, KINETIS_DMA_TCD_DOFF(ch->ind));
    putreg16(((uint16_t)per_data_sz) << DMA_TCD_ATTR_DSIZE_SHIFT, KINETIS_DMA_TCD_ATTR(ch->ind));
  } else {
    return NULL;
  }

  return (DMA_HANDLE)ch;
}

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

void kinetis_dmafree(DMA_HANDLE handle)
{
  struct kinetis_dma_ch *ch = (struct kinetis_dma_ch *)handle;
  uint8_t regval;
  irqstate_t flags;

  DEBUGASSERT(handle != NULL);

  // Disable DMA channel in the dmamux
  regval = 0;
  putreg8(regval, KINETIS_DMAMUX_CHCFG(ch->ind));

  flags = enter_critical_section();
  ch->used = false;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: kinetis_dmasetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ****************************************************************************/

int kinetis_dmasetup(DMA_HANDLE handle, uint32_t mem_addr, KINETIS_DMA_DATA_SZ mem_data_sz, size_t nbytes, const kinetis_dmachannel_config *config)
{
  struct kinetis_dma_ch *ch = (struct kinetis_dma_ch *)handle;
  uint16_t regval = 0;

  DEBUGASSERT(handle != NULL);

  if (ch->dir == KINETIS_DMA_DIRECTION_PERIPHERAL_TO_MEMORY) {
    putreg32(mem_addr, KINETIS_DMA_TCD_DADDR(ch->ind));
    putreg16(1, KINETIS_DMA_TCD_DOFF(ch->ind));
    putreg16(((uint16_t)mem_data_sz) << DMA_TCD_ATTR_DSIZE_SHIFT, KINETIS_DMA_TCD_ATTR(ch->ind));
  } else if (ch->dir == KINETIS_DMA_DIRECTION_MEMORY_TO_PERIPHERAL) {
    putreg32(mem_addr, KINETIS_DMA_TCD_SADDR(ch->ind));
    putreg16(1, KINETIS_DMA_TCD_SOFF(ch->ind));
    putreg16(((uint16_t)mem_data_sz) << DMA_TCD_ATTR_SSIZE_SHIFT, KINETIS_DMA_TCD_ATTR(ch->ind));
  } else {
    return -1;
  }

  regval = getreg16(KINETIS_DMA_TCD_CSR(ch->ind));

  regval &= ~DMA_TCD_CSR_MAJORLINKCH_MASK;
  // Link channel with itself
  regval |= config->circular ? (ch->ind | DMA_TCD_CSR_MAJORELINK) : 0;
  // Enable major half complete and major complete interrupts
  regval |= config->halfcomplete_interrupt ? DMA_TCD_CSR_INTHALF : 0;
  regval |= DMA_TCD_CSR_INTMAJOR;

  putreg16(regval, KINETIS_DMA_TCD_CSR(ch->ind));

  regval = nbytes & 0x7FFF;
  // todo: ensure that 0<= nbytes <= 0x7FFF!
  putreg16(regval, KINETIS_DMA_TCD_BITER(ch->ind));
  putreg16(regval, KINETIS_DMA_TCD_CITER(ch->ind));

  // Set minor loop count
  putreg16(1, KINETIS_DMA_TCD_NBYTES(ch->ind));

  return OK;
}

/****************************************************************************
 * Name: kinetis_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int kinetis_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  struct kinetis_dma_ch *ch = (struct kinetis_dma_ch *)handle;
  uint8_t regval;

  DEBUGASSERT(handle != NULL);

  ch->callback = callback;
  ch->arg = arg;

  // todo move
  regval = getreg8(KINETIS_DMAMUX_CHCFG(ch->ind));
  regval |= (uint8_t)ch->src;
  regval |= DMAMUX_CHCFG_ENBL;
  putreg8(regval, KINETIS_DMAMUX_CHCFG(ch->ind));

  // Enable request register for this channel
  putreg8(ch->ind, KINETIS_DMA_SERQ);


  // todo: enble also error interrupt register? (seei)?
  // todo: set ssrt? (24.3.10)?
  return OK;
}


/****************************************************************************
 * Name: kinetis_dmastop
 *
 * Description:
 *   Cancel the DMA.  After kinetis_dmastop() is called, the DMA channel is
 *   reset and kinetis_dmasetup() must be called before kinetis_dmastart() can be
 *   called again
 *
 ****************************************************************************/

void kinetis_dmastop(DMA_HANDLE handle)
{
  struct kinetis_dma_ch *ch = (struct kinetis_dma_ch *)handle;

  DEBUGASSERT(handle != NULL);

  putreg8(ch->ind, KINETIS_DMA_CERQ);
}

/****************************************************************************
 * Name: kinetis_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void kinetis_dmasample(DMA_HANDLE handle, struct kinetis_dmaregs_s *regs)
{

  DEBUGASSERT(handle != NULL);

}
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
                const char *msg)
{
  DEBUGASSERT(handle != NULL);

}
#endif

