/****************************************************************************
 * drivers/usbhost/rtl8187.h
 *
 * This file is part of NuttX:
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2011 Rafael Noronha. All rights reserved.
 *   Authors: Gregoyr Nutt <spudmonkey@racsa.co.cr>
 *            Rafael Noronha <rafael@pdsolucoes.com.br>
 *
 * Portions of the logic in this file derives from the KisMAC RTL8187x driver
 *
 *    Created by pr0gg3d on 02/24/08.
 *
 * Which, in turn, came frm the SourceForge rt2x00 project:
 *
 *   Copyright (C) 2004 - 2006 rt2x00 SourceForge Project
 *   <http://rt2x00.serialmonkey.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * There are probably also pieces from the Linux RTL8187x driver
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
 *
 ****************************************************************************/

#ifndef __DRIVERS_NET_RTL8187X_H
#define __DRIVERS_NET_RTL8187X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CSR Bit Field Definitions ************************************************/

/* Refers to "cmd" field of "rtl8187x_csr_s" struct */

#define RTL8187X_CMD_TXENABLE           (1 << 2)
#define RTL8187X_CMD_RXENABLE           (1 << 3)
#define RTL8187X_CMD_RESET              (1 << 4)

/* Refers to "status" field of "rtl8187x_csr_s" struct */

#define RTL8187X_INT_RXOK               (1 << 0)
#define RTL8187X_INT_RXERR              (1 << 1)
#define RTL8187X_INT_TXLOK              (1 << 2)
#define RTL8187X_INT_TXLERR             (1 << 3)
#define RTL8187X_INT_RXDU               (1 << 4)
#define RTL8187X_INT_RXFO               (1 << 5)
#define RTL8187X_INT_TXNOK              (1 << 6)
#define RTL8187X_INT_TXNERR             (1 << 7)
#define RTL8187X_INT_TXHOK              (1 << 8)
#define RTL8187X_INT_TXHERR             (1 << 9)
#define RTL8187X_INT_TXBOK              (1 << 10)
#define RTL8187X_INT_TXBERR             (1 << 11)
#define RTL8187X_INT_ATIM               (1 << 12)
#define RTL8187X_INT_BEACON             (1 << 13)
#define RTL8187X_INT_TIMEOUT            (1 << 14)
#define RTL8187X_INT_TXFO               (1 << 15)

/* Refers to "tx_conf" field of "rtl8187x_csr_s" struct */

#define RTL8187X_TXCONF_LOOPBACKMAC     (1 << 17)
#define RTL8187X_TXCONF_LOOPBACKCONT    (3 << 17)
#define RTL8187X_TXCONF_NOICV           (1 << 19)
#define RTL8187X_TXCONF_DISCW           (1 << 20)
#define RTL8187X_TXCONF_SATHWPLCP       (1 << 24)
#define RTL8187X_TXCONF_R8180ABCD       (2 << 25)
#define RTL8187X_TXCONF_R8180F          (3 << 25)
#define RTL8187X_TXCONF_R8185ABC        (4 << 25)
#define RTL8187X_TXCONF_R8185D          (5 << 25)
#define RTL8187X_TXCONF_R8187VD         (5 << 25)
#define RTL8187X_TXCONF_R8187VDB        (6 << 25)
#define RTL8187X_TXCONF_HWVERMASK       (7 << 25)
#define RTL8187X_TXCONF_DISREQQSIZE     (1 << 28)
#define RTL8187X_TXCONF_PROBEDTS        (1 << 29)
#define RTL8187X_TXCONF_HWSEQNUM        (1 << 30)
#define RTL8187X_TXCONF_CWMIN           (1 << 31)

/* Refers to "rx_conf" field of "rtl8187x_csr_s" struct */

#define RTL8187X_RXCONF_MONITOR         (1 << 0)
#define RTL8187X_RXCONF_NICMAC          (1 << 1)
#define RTL8187X_RXCONF_MULTICAST       (1 << 2)
#define RTL8187X_RXCONF_BROADCAST       (1 << 3)
#define RTL8187X_RXCONF_FCS             (1 << 5)
#define RTL8187X_RXCONF_DATA            (1 << 18)
#define RTL8187X_RXCONF_CTRL            (1 << 19)
#define RTL8187X_RXCONF_MGMT            (1 << 20)
#define RTL8187X_RXCONF_ADDR3           (1 << 21)
#define RTL8187X_RXCONF_PM              (1 << 22)
#define RTL8187X_RXCONF_BSSID           (1 << 23)
#define RTL8187X_RXCONF_RXAUTORESETPHY  (1 << 28)
#define RTL8187X_RXCONF_CSDM1           (1 << 29)
#define RTL8187X_RXCONF_CSDM2           (1 << 30)
#define RTL8187X_RXCONF_ONLYERLPKT      (1 << 31)

/* Refers to "eeprom_cmd" field of "rtl8187x_csr_s" struct */

#define RTL8187X_EEPROMCMD_READ         (1 << 0)
#define RTL8187X_EEPROMCMD_WRITE        (1 << 1)
#define RTL8187X_EEPROMCMD_CK           (1 << 2)
#define RTL8187X_EEPROMCMD_CS           (1 << 3)
#define RTL8187X_EEPROMCMD_NORMAL       (0 << 6)
#define RTL8187X_EEPROMCMD_LOAD         (1 << 6)
#define RTL8187X_EEPROMCMD_PROGRAM      (2 << 6)
#define RTL8187X_EEPROMCMD_CONFIG       (3 << 6)

/* Refers to "config2" field of "rtl8187x_csr_s" struct */

#define RTL8187X_CONFIG2_ANTENNADIV     (1 << 6)

/* Refers to "msr" field of "rtl8187x_csr_s" struct */

#define RTL8187X_MSR_NOLINK             (0 << 2)
#define RTL8187X_MSR_ADHOC              (1 << 2)
#define RTL8187X_MSR_INFRA              (2 << 2)
#define RTL8187X_MSR_MASTER             (3 << 2)
#define RTL8187X_MSR_ENEDCA             (4 << 2)

/* Refers to "config3" field of "rtl8187x_csr_s" struct */

#define RTL8187X_CONFIG3_ANAPARAMWRITE  (1 << 6)
#define RTL8187X_CONFIG3_GNTSELECT      (1 << 7)

/* Refers to "config4" field of "rtl8187x_csr_s" struct */

#define RTL8187X_CONFIG4_POWEROFF       (1 << 6)
#define RTL8187X_CONFIG4_VCOOFF         (1 << 7)

/* Refers to "tx_agc_ctl" field of "rtl8187x_csr_s" struct */

#define RTL8187X_TXAGCCTL_PERPACKETGAINSHIFT    (1 << 0)
#define RTL8187X_TXAGCCTL_PERPACKETANTSELSHIFT  (1 << 1)
#define RTL8187X_TXAGCCTL_FEEDBACKANT           (1 << 2)

/* Refers to "cw_conf" field of "rtl8187x_csr_s" struct */

#define RTL8187X_CWCONF_PERPACKETCWSHIFT      (1 << 0)
#define RTL8187X_CWCONF_PERPACKETRETRYSHIFT   (1 << 1)

/* Refers to "rate_fallback" field of "rtl8187x_csr_s" struct */

#define RTL8187X_RATEFALLBACK_ENABLE    (1 << 7)

/* TX/RX Descriptor Bit Field Definitions ***********************************/
/* Tx/Rx flags are common between RTL818X chips */

/* Refers to "flags" field of "rtl8187x_txdesc_s" struct */

#define RTL8187X_TXDESC_FLAG_NOENC      (1 << 15) /* Disable hardware based encryption */
#define RTL8187X_TXDESC_FLAG_TXOK       (1 << 15) /* TX frame was ACKed */
#define RTL8187X_TXDESC_FLAG_SPLCP      (1 << 16) /* Use short preamble */
#define RTL8187X_TXDESC_FLAG_RXUNDER    (1 << 16) 
#define RTL8187X_TXDESC_FLAG_MOREFRAG   (1 << 17) /* More fragments follow */
#define RTL8187X_TXDESC_FLAG_CTS        (1 << 18) /* Use CTS-to-self protection */
#define RTL8187X_TXDESC_FLAG_RTS        (1 << 23) /* Use RTS/CTS protection */
#define RTL8187X_TXDESC_FLAG_LS         (1 << 28) /* Last segment of the frame */
#define RTL8187X_TXDESC_FLAG_FS         (1 << 29) /* First segment of the frame */
#define RTL8187X_TXDESC_FLAG_DMA        (1 << 30)
#define RTL8187X_TXDESC_FLAG_OWN        (1 << 31)

/* Refers to "flags" field of "rtl8187x_rxdesc_s" struct */

#define RTL8187X_RXDESC_FLAG_ICVERR     (1 << 12)
#define RTL8187X_RXDESC_FLAG_CRC32ERR   (1 << 13)
#define RTL8187X_RXDESC_FLAG_PM         (1 << 14)
#define RTL8187X_RXDESC_FLAG_RXERR      (1 << 15)
#define RTL8187X_RXDESC_FLAG_BCAST      (1 << 16)
#define RTL8187X_RXDESC_FLAG_PAM        (1 << 17)
#define RTL8187X_RXDESC_FLAG_MCAST      (1 << 18)
#define RTL8187X_RXDESC_FLAG_QOS        (1 << 19) /* RTL8187(B) only */
#define RTL8187X_RXDESC_FLAG_TRSW       (1 << 24) /* RTL8187(B) only */
#define RTL8187X_RXDESC_FLAG_SPLCP      (1 << 25)
#define RTL8187X_RXDESC_FLAG_FOF        (1 << 26)
#define RTL8187X_RXDESC_FLAG_DMAFAIL    (1 << 27)
#define RTL8187X_RXDESC_FLAG_LS         (1 << 28)
#define RTL8187X_RXDESC_FLAG_FS         (1 << 29)
#define RTL8187X_RXDESC_FLAG_EOR        (1 << 30)
#define RTL8187X_RXDESC_FLAG_OWN        (1 << 31)

/* TX descriptor rate values */

#define RTL8187X_RATE_1                 0
#define RTL8187X_RATE_2                 1
#define RTL8187X_RATE_5p5               2
#define RTL8187X_RATE_11                3
#define RTL8187X_RATE_6                 4
#define RTL8187X_RATE_9                 5
#define RTL8187X_RATE_12                6
#define RTL8187X_RATE_18                7
#define RTL8187X_RATE_24                8
#define RTL8187X_RATE_36                9
#define RTL8187X_RATE_48                10
#define RTL8187X_RATE_54                11

/* Other RTL8187x Definitions **********************************************/

/* Number of IEEE 802.11 Channels */

#define RTL8187X_NCHANNELS              14

/* Vendor-Specific Requests */

#define RTL8187X_REQT_READ              0xc0
#define RTL8187X_REQT_WRITE             0x40
#define RTL8187X_REQ_GETREG             0x05
#define RTL8187X_REQ_SETREG             0x05

/* EEPROM Definitions */

#define PCI_EEPROM_WIDTH_93C46          6
#define PCI_EEPROM_WIDTH_93C56          8
#define PCI_EEPROM_WIDTH_93C66          8
#define PCI_EEPROM_WIDTH_OPCODE         3
#define PCI_EEPROM_WRITE_OPCODE         0x05
#define PCI_EEPROM_READ_OPCODE          0x06
#define PCI_EEPROM_EWDS_OPCODE          0x10
#define PCI_EEPROM_EWEN_OPCODE          0x13

#define RTL8187X_EEPROM_TXPWRBASE       0x05
#define RTL8187X_EEPROM_MACADDR         0x07
#define RTL8187X_EEPROM_TXPWRCHAN1      0x16  /* 3 channels */
#define RTL8187X_EEPROM_TXPWRCHAN6      0x1b  /* 2 channels */
#define RTL8187X_EEPROM_TXPWRCHAN4      0x3d  /* 2 channels */

/* RT8187x Register Addresses ***********************************************/

#define RTL8187X_ADDR_RXCONF            0xff44
#define RTL8187X_ADDR_EEPROMCMD         0xff50
#define RTL8187X_ADDR_PGSELECT          0xff5e
#define RTL8187X_ADDR_RFPINSOUTPUT      0xff80
#define RTL8187X_ADDR_RFPINSENABLE      0xff82
#define RTL8187X_ADDR_RFPINSSELECT      0xff84
#define RTL8187X_ADDR_RFPINSINPUT       0xff86
#define RTL8187X_ADDR_TESTR             0xff5b
#define RTL8187X_ADDR_TXANTENNA         0xff9f
#define RTL8187X_ADDR_PHY3              0xff7f
#define RTL8187X_ADDR_PHY2              0xff7e
#define RTL8187X_ADDR_PHY1              0xff7d
#define RTL8187X_ADDR_PHY0              0xff7c
#define RTL8187X_ADDR_TXGAINCCK         0xff9d
#define RTL8187X_ADDR_CONFIG3           0xff59
#define RTL8187X_ADDR_ANAPARAM2         0xff60
#define RTL8187X_ADDR_TXGAINOFDM        0xff9e

#define RTL8187X_ADDR_ANAPARAM          0xff54
#define RTL8187X_ADDR_INTMASK           0xff3c
#define RTL8187X_ADDR_CMD               0xff37
#define RTL8187X_ADDR_GPIO              0xff91
#define RTL8187X_ADDR_GPENABLE          0xff90
#define RTL8187X_ADDR_CONFIG1           0xff52
#define RTL8187X_ADDR_INTTIMEOUT        0xff48
#define RTL8187X_ADDR_WPACONF           0xffb0
#define RTL8187X_ADDR_RATEFALLBACK      0xffbe
#define RTL8187X_ADDR_RESPRATE          0xff34
#define RTL8187X_ADDR_BRSR              0xff2c
#define RTL8187X_ADDR_RFTIMING          0xff8c
#define RTL8187X_ADDR_RFPARA            0xff88
#define RTL8187X_ADDR_TALLYSEL          0xfffc
#define RTL8187X_ADDR_INTMASK           0xff3c
#define RTL8187X_ADDR_MAR0              0xff08
#define RTL8187X_ADDR_MAR1              0xff0c
#define RTL8187X_ADDR_CWCONF            0xffbc
#define RTL8187X_ADDR_TXAGCCTL          0xff9c
#define RTL8187X_ADDR_TXCONF            0xff40
#define RTL8187X_ADDR_CMD               0xff37
#define RTL8187X_ADDR_CONFIG4           0xff5a

/* Other RTL8187x Register Values ******************************************/

#define RTL8225_ANAPARAM_ON             0xa0000a59
#define RTL8225_ANAPARAM2_ON            0x860c7312
#define RTL8225_ANAPARAM_OFF            0xa00beb59
#define RTL8225_ANAPARAM2_OFF           0x840dec11

/* Standard Helper Macros ***************************************************/

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#ifndef MAX
#  define MAX(a,b) (a > b ? a : b)
#endif

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif

#ifndef NULL
#  define NULL ((void*)0)
#endif

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

 /* RTL-818x mapping struct */

struct rtl8187x_csr_s 
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

/* RX and TX descriptors */

struct rtl8187x_rxdesc_s
{
  uint32_t  flags;
  uint8_t   noise;
  uint8_t   signal;
  uint8_t   agc;
  uint8_t   reserved;
  uint64_t  mactime;
} __attribute__((packed));

#define SIZEOF_RXDESC 16

struct rtl8187x_txdesc_s
{
  uint32_t  flags;
  uint16_t  rtsduration;
  uint16_t  len;
  uint32_t  retry;
} __attribute__((packed));

#define SIZEOF_TXDESC 12

#endif /* __DRIVERS_NET_RTL8187X_H */

