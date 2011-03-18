/****************************************************************************
 * drivers/usbhost/rtl8187.h
 *
 * This file is part of NuttX:
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *
 * Contributed by:
 *
 *   Copyright (C) 2011 Rafael Noronha. All rights reserved.
 *   Authors: Rafael Noronha <rafael@pdsolucoes.com.br>
 *
 * Portions of the logic in this file derives from the Linux RTL8187x driver
 * 
 *   Copyright 2007 Michael Wu <flamingice@sourmilk.net>
 *   Copyright 2007 Andrea Merello <andreamrl@tiscali.it>
 *
 *   Based on the r8187 driver, which is:
 *   Copyright 2004-2005 Andrea Merello <andreamrl@tiscali.it>, et al.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 ****************************************************************************/

#ifndef __DRIVERS_NET_RTL818X_H
#define __DRIVERS_NET_RTL818X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Set/clear a specific bit */

#define set_bit(x) (1 << (x))
#define clr_bit(x) (~(set_bit(x)))

/* Refers to "cmd" field of "rtl818x_csr_s" struct */

#define RTL818X_CMD_TX_ENABLE     set_bit(2)
#define RTL818X_CMD_RX_ENABLE     set_bit(3)
#define RTL818X_CMD_RESET         set_bit(4)

/* Refers to "status" field of "rtl818x_csr_s" struct */

#define RTL818X_INT_RX_OK         set_bit(0)
#define RTL818X_INT_RX_ERR        set_bit(1)
#define RTL818X_INT_TXL_OK        set_bit(2)
#define RTL818X_INT_TXL_ERR       set_bit(3)
#define RTL818X_INT_RX_DU         set_bit(4)
#define RTL818X_INT_RX_FO         set_bit(5)
#define RTL818X_INT_TXN_OK        set_bit(6)
#define RTL818X_INT_TXN_ERR       set_bit(7)
#define RTL818X_INT_TXH_OK        set_bit(8)
#define RTL818X_INT_TXH_ERR       set_bit(9)
#define RTL818X_INT_TXB_OK        set_bit(10)
#define RTL818X_INT_TXB_ERR       set_bit(11)
#define RTL818X_INT_ATIM          set_bit(12)
#define RTL818X_INT_BEACON        set_bit(13)
#define RTL818X_INT_TIME_OUT      set_bit(14)
#define RTL818X_INT_TX_FO         set_bit(15)

/* Refers to "tx_conf" field of "rtl818x_csr_s" struct */

#define RTL818X_TX_CONF_LOOPBACK_MAC    set_bit(17)
#define RTL818X_TX_CONF_LOOPBACK_CONT   (3 << 17)
#define RTL818X_TX_CONF_NO_ICV          set_bit(19)
#define RTL818X_TX_CONF_DISCW           set_bit(20)
#define RTL818X_TX_CONF_SAT_HWPLCP      set_bit(24)
#define RTL818X_TX_CONF_R8180_ABCD      (2 << 25)
#define RTL818X_TX_CONF_R8180_F         (3 << 25)
#define RTL818X_TX_CONF_R8185_ABC       (4 << 25)
#define RTL818X_TX_CONF_R8185_D         (5 << 25)
#define RTL818X_TX_CONF_R8187vD         (5 << 25)
#define RTL818X_TX_CONF_R8187vD_B       (6 << 25)
#define RTL818X_TX_CONF_HWVER_MASK      (7 << 25)
#define RTL818X_TX_CONF_DISREQQSIZE     set_bit(28)
#define RTL818X_TX_CONF_PROBE_DTS       set_bit(29)
#define RTL818X_TX_CONF_HW_SEQNUM       set_bit(30)
#define RTL818X_TX_CONF_CW_MIN          set_bit(31)

/* Refers to "rx_conf" field of "rtl818x_csr_s" struct */

#define RTL818X_RX_CONF_MONITOR         set_bit(0)
#define RTL818X_RX_CONF_NICMAC          set_bit(1)
#define RTL818X_RX_CONF_MULTICAST       set_bit(2)
#define RTL818X_RX_CONF_BROADCAST       set_bit(3)
#define RTL818X_RX_CONF_FCS             set_bit(5)
#define RTL818X_RX_CONF_DATA            set_bit(18)
#define RTL818X_RX_CONF_CTRL            set_bit(19)
#define RTL818X_RX_CONF_MGMT            set_bit(20)
#define RTL818X_RX_CONF_ADDR3           set_bit(21)
#define RTL818X_RX_CONF_PM              set_bit(22)
#define RTL818X_RX_CONF_BSSID           set_bit(23)
#define RTL818X_RX_CONF_RX_AUTORESETPHY set_bit(28)
#define RTL818X_RX_CONF_CSDM1           set_bit(29)
#define RTL818X_RX_CONF_CSDM2           set_bit(30)
#define RTL818X_RX_CONF_ONLYERLPKT      set_bit(31)

/* Refers to "eeprom_cmd" field of "rtl818x_csr_s" struct */

#define RTL818X_EEPROM_CMD_READ         set_bit(0)
#define RTL818X_EEPROM_CMD_WRITE        set_bit(1)
#define RTL818X_EEPROM_CMD_CK           set_bit(2)
#define RTL818X_EEPROM_CMD_CS           set_bit(3)
#define RTL818X_EEPROM_CMD_NORMAL       (0 << 6)
#define RTL818X_EEPROM_CMD_LOAD         (1 << 6)
#define RTL818X_EEPROM_CMD_PROGRAM      (2 << 6)
#define RTL818X_EEPROM_CMD_CONFIG       (3 << 6)

/* Refers to "config2" field of "rtl818x_csr_s" struct */

#define RTL818X_CONFIG2_ANTENNA_DIV     set_bit(6)

/* Refers to "msr" field of "rtl818x_csr_s" struct */

#define RTL818X_MSR_NO_LINK       (0 << 2)
#define RTL818X_MSR_ADHOC         (1 << 2)
#define RTL818X_MSR_INFRA         (2 << 2)
#define RTL818X_MSR_MASTER        (3 << 2)
#define RTL818X_MSR_ENEDCA        (4 << 2)

/* Refers to "config3" field of "rtl818x_csr_s" struct */

#define RTL818X_CONFIG3_ANAPARAM_WRITE  set_bit(6)
#define RTL818X_CONFIG3_GNT_SELECT      set_bit(7)

/* Refers to "config4" field of "rtl818x_csr_s" struct */

#define RTL818X_CONFIG4_POWEROFF  set_bit(6)
#define RTL818X_CONFIG4_VCOOFF    set_bit(7)

/* Refers to "tx_agc_ctl" field of "rtl818x_csr_s" struct */

#define RTL818X_TX_AGC_CTL_PERPACKET_GAIN_SHIFT    set_bit(0)
#define RTL818X_TX_AGC_CTL_PERPACKET_ANTSEL_SHIFT  set_bit(1)
#define RTL818X_TX_AGC_CTL_FEEDBACK_ANT            set_bit(2)

/* Refers to "cw_conf" field of "rtl818x_csr_s" struct */

#define RTL818X_CW_CONF_PERPACKET_CW_SHIFT      set_bit(0)
#define RTL818X_CW_CONF_PERPACKET_RETRY_SHIFT   set_bit(1)

/* Refers to "rate_fallback" field of "rtl818x_csr_s" struct */

#define RTL818X_RATE_FALLBACK_ENABLE    set_bit(7)

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

 /* RTL-818x mapping struct */

struct rtl818x_csr_s 
{
  uint8_t   mac[6];
  uint8_t   reserved_0[2];
  uint32_t  mar[2];
  uint8_t   rx_fifo_count;
  uint8_t   reserved_1;
  uint8_t   tx_fifo_count;
  uint8_t   bqreq;
  uint8_t   reserved_2[4];
  uint32_t  tsft[2];
  uint32_t  tlpda;
  uint32_t  tnpda;
  uint32_t  thpda;
  uint16_t  brsr;
  uint8_t   bssid[6];
  uint8_t   resp_rate;
  uint8_t   eifs;
  uint8_t   reserved_3[1];
  uint8_t   cmd;
  uint8_t   reserved_4[4];
  uint16_t  int_mask;
  uint16_t  int_status;
  uint32_t  tx_conf;
  uint32_t  rx_conf;
  uint32_t  int_timeout;
  uint32_t  tbda;
  uint8_t   eeprom_cmd;
  uint8_t   config0;
  uint8_t   config1;
  uint8_t   config2;
  uint32_t  anaparam;
  uint8_t   msr;
  uint8_t   config3;
  uint8_t   config4;
  uint8_t   testr;
  uint8_t   reserved_9[2];
  uint8_t   pgselect;
  uint8_t   security;
  uint32_t  anaparam2;
  uint8_t   reserved_10[12];
  uint16_t  beacon_interval;
  uint16_t  atim_wnd;
  uint16_t  beacon_interval_time;
  uint16_t  atimtr_interval;
  uint8_t   phy_delay;
  uint8_t   carrier_sense_counter;
  uint8_t   reserved_11[2];
  uint8_t   phy[4];
  uint16_t  rfpinsoutput;
  uint16_t  rfpinsenable;
  uint16_t  rfpinsselect;
  uint16_t  rfpinsinput;
  uint32_t  rf_para;
  uint32_t  rf_timing;
  uint8_t   gp_enable;
  uint8_t   gpio0;
  uint8_t   gpio1;
  uint8_t   reserved_12;
  uint32_t  hssi_para;
  uint8_t   reserved_13[4];
  uint8_t   tx_agc_ctl;
  uint8_t   tx_gain_cck;
  uint8_t   tx_gain_ofdm;
  uint8_t   tx_antenna;
  uint8_t   reserved_14[16];
  uint8_t   wpa_conf;
  uint8_t   reserved_15[3];
  uint8_t   sifs;
  uint8_t   difs;
  uint8_t   slot;
  uint8_t   reserved_16[5];
  uint8_t   cw_conf;
  uint8_t   cw_val;
  uint8_t   rate_fallback;
  uint8_t   acm_control;
  uint8_t   reserved_17[24];
  uint8_t   config5;
  uint8_t   tx_dma_polling;
  uint8_t   reserved_18[2];
  uint16_t  cwr;
  uint8_t   retry_ctr;
  uint8_t   reserved_19[3];
  uint16_t  int_mig;
  uint32_t  rdsar;
  uint16_t  tid_ac_map;
  uint8_t   reserved_20[4];
  uint8_t   anaparam3;
  uint8_t   reserved_21[5];
  uint16_t  femr;
  uint8_t   reserved_22[4];
  uint16_t  tally_cnt;
  uint8_t   tally_sel;
} __attribute__ ((packed));

/* Numbers from ioregisters */

enum rtl818x_r8187b_x_e
{ 
  RTL818X_R8187B_B = 0,
  RTL818X_R8187B_D,
  RTL818X_R8187B_E
};

struct ieee80211_conf_s;
struct ieee80211_bssconf_s;
struct rtl818x_rfops_s
{
  char    *name;
  void    (*init)    (struct ieee80211_hw_s *);
  void    (*stop)    (struct ieee80211_hw_s *);
  void    (*setchan) (struct ieee80211_hw_s *, struct ieee80211_conf_s *);
  void    (*conferp) (struct ieee80211_hw_s *, struct ieee80211_bssconf_s *);
  uint8_t (*calcrssi)(uint8_t agc, uint8_t sq);
};
 
/* Tx/Rx flags are common between RTL818X chips */

enum rtl818x_tx_desc_flags_e
{
  RTL818X_TX_DESC_FLAG_NO_ENC     = set_bit(15), /* Disable hardware based encryption */
  RTL818X_TX_DESC_FLAG_TX_OK      = set_bit(15), /* TX frame was ACKed */
  RTL818X_TX_DESC_FLAG_SPLCP      = set_bit(16), /* Use short preamble */
  RTL818X_TX_DESC_FLAG_RX_UNDER   = set_bit(16), 
  RTL818X_TX_DESC_FLAG_MOREFRAG   = set_bit(17), /* More fragments follow */
  RTL818X_TX_DESC_FLAG_CTS        = set_bit(18), /* Use CTS-to-self protection */
  RTL818X_TX_DESC_FLAG_RTS        = set_bit(23), /* Use RTS/CTS protection */
  RTL818X_TX_DESC_FLAG_LS         = set_bit(28), /* Last segment of the frame */
  RTL818X_TX_DESC_FLAG_FS         = set_bit(29), /* First segment of the frame */
  RTL818X_TX_DESC_FLAG_DMA        = set_bit(30),
  RTL818X_TX_DESC_FLAG_OWN        = set_bit(31)
};
 
enum rtl818x_rx_desc_flags_e 
{
  RTL818X_RX_DESC_FLAG_ICV_ERR    = set_bit(12),
  RTL818X_RX_DESC_FLAG_CRC32_ERR  = set_bit(13),
  RTL818X_RX_DESC_FLAG_PM         = set_bit(14),
  RTL818X_RX_DESC_FLAG_RX_ERR     = set_bit(15),
  RTL818X_RX_DESC_FLAG_BCAST      = set_bit(16),
  RTL818X_RX_DESC_FLAG_PAM        = set_bit(17),
  RTL818X_RX_DESC_FLAG_MCAST      = set_bit(18),
  RTL818X_RX_DESC_FLAG_QOS        = set_bit(19), /* RTL8187(B) only */
  RTL818X_RX_DESC_FLAG_TRSW       = set_bit(24), /* RTL8187(B) only */
  RTL818X_RX_DESC_FLAG_SPLCP      = set_bit(25),
  RTL818X_RX_DESC_FLAG_FOF        = set_bit(26),
  RTL818X_RX_DESC_FLAG_DMA_FAIL   = set_bit(27),
  RTL818X_RX_DESC_FLAG_LS         = set_bit(28),
  RTL818X_RX_DESC_FLAG_FS         = set_bit(29),
  RTL818X_RX_DESC_FLAG_EOR        = set_bit(30),
  RTL818X_RX_DESC_FLAG_OWN        = set_bit(31)
};

#endif /* __DRIVERS_NET_RTL818X_H */