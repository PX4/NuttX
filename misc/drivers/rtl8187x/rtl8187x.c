/****************************************************************************
 * drivers/usbhost/rtl8187x_rtl8187.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Authors: Rafael Noronha <rafael@pdsolucoes.com.br>
 *            Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <time.h>
#include <wdog.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/irq.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>

#include <net/uip/uip.h>
#include <net/uip/uip-arp.h>
#include <net/uip/uip-arch.h>

#if defined(CONFIG_USBHOST) && defined(CONFIG_NET) && defined(CONFIG_NET_WLAN)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  warning "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

/* Driver support ***********************************************************/
/* This format is used to construct the /dev/wlan[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT          "/dev/wlan%c"
#define DEV_NAMELEN         12

/* Used in rtl8187x_cfgdesc() */

#define USBHOST_IFFOUND     0x01
#define USBHOST_BINFOUND    0x02
#define USBHOST_BOUTFOUND   0x04
#define USBHOST_ALLFOUND    0x07

#define USBHOST_MAX_CREFS   0x7fff

/* CONFIG_WLAN_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_WLAN_NINTERFACES
# define CONFIG_WLAN_NINTERFACES 1
#endif

/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define WLAN_WDDELAY   (1*CLK_TCK)
#define WLAN_POLLHSEC  (1*2)

/* TX timeout = 1 minute */

#define WLAN_TXTIMEOUT (60*CLK_TCK)

/* This is a helper pointer for accessing the contents of the WLAN header */

#define BUF ((struct uip_eth_hdr *)priv->ethdev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure contains the internal, private state of the USB host class
 * driver.
 */

struct rtl8187x_state_s
{
  /* This is the externally visible portion of the USB class state */

  struct usbhost_class_s  class;

  /* This is an instance of the USB host controller driver bound to this class instance */

  struct usbhost_driver_s *drvr;

  /* The following fields support the USB class driver */
  
  char                      devchar;      /* Character identifying the /dev/wlan[n] device */
  volatile bool             disconnected; /* TRUE: Device has been disconnected */
  bool                      bifup;        /* TRUE: Ethernet interface is up */
  uint8_t                   ifno;         /* Interface number */
  int16_t                   crefs;        /* Reference count on the driver instance */
  sem_t                     exclsem;      /* Used to maintain mutual exclusive access */
  struct work_s             work;         /* For interacting with the worker thread */
  FAR struct usb_ctrlreq_s *ctrlreq;      /* The allocated request buffer */
  FAR uint8_t              *tbuffer;      /* The allocated transfer buffer */
  size_t                    tbuflen;      /* Size of the allocated transfer buffer */
  usbhost_ep_t              epin;         /* IN endpoint */
  usbhost_ep_t              epout;        /* OUT endpoint */
  WDOG_ID                   txpoll;       /* Ethernet TX poll timer */
  WDOG_ID                   txtimeout;    /* Ethernet TX timeout timer */

  /* This holds the information visible to uIP/NuttX */

  struct uip_driver_s       ethdev;       /* Interface understood by uIP */
 };

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* General Utility Functions ************************************************/
/* Semaphores */

static void rtl8187x_takesem(sem_t *sem);
#define rtl8187x_givesem(s) sem_post(s);

/* Memory allocation services */

static inline FAR struct rtl8187x_state_s *rtl8187x_allocclass(void);
static inline void rtl8187x_freeclass(FAR struct rtl8187x_state_s *class);

/* Device name management */

static int rtl8187x_allocdevno(FAR struct rtl8187x_state_s *priv);
static void rtl8187x_freedevno(FAR struct rtl8187x_state_s *priv);
static inline void rtl8187x_mkdevname(FAR struct rtl8187x_state_s *priv, char *devname);

/* Standard USB host class functions ****************************************/
/* Worker thread actions */

static void rtl8187x_destroy(FAR void *arg);

/* Helpers for rtl8187x_connect() */

static inline int rtl8187x_cfgdesc(FAR struct rtl8187x_state_s *priv,
                                  FAR const uint8_t *configdesc, int desclen,
                                  uint8_t funcaddr);
static inline int rtl8187x_devinit(FAR struct rtl8187x_state_s *priv);

/* (Little Endian) Data helpers */

static inline uint16_t rtl8187x_host2le16(uint16_t val);
static inline uint32_t rtl8187x_host2le32(uint32_t val);
static inline uint16_t rtl8187x_getle16(const uint8_t *val);
static inline uint32_t rtl8187x_getle32(const uint8_t *val);
static inline void rtl8187x_putle16(uint8_t *dest, uint16_t val);
static void rtl8187x_putle32(uint8_t *dest, uint32_t val);

/* Transfer descriptor memory management */

static inline int rtl8187x_talloc(FAR struct rtl8187x_state_s *priv);
static inline int rtl8187x_tfree(FAR struct rtl8187x_state_s *priv);

/* struct usbhost_registry_s methods */
 
static struct usbhost_class_s *rtl8187x_create(FAR struct usbhost_driver_s *drvr,
                                               FAR const struct usbhost_id_s *id);

/* struct usbhost_class_s methods */

static int rtl8187x_connect(FAR struct usbhost_class_s *class,
                           FAR const uint8_t *configdesc, int desclen,
                           uint8_t funcaddr);
static int rtl8187x_disconnected(FAR struct usbhost_class_s *class);

/* Vendor-Specific USB host support *****************************************/

static uint8_t rtl8187x_ioread8(struct rtl8187x_state_s *priv, uint16_t addr);
static uint16_t rtl8187x_ioread16(struct rtl8187x_state_s *priv, uint16_t addr);
static uint32_t rtl8187x_ioread32(struct rtl8187x_state_s *priv, uint16_t addr);

/* Ethernet driver methods **************************************************/
/* Common TX logic */

static int  rtl8187x_transmit(FAR struct rtl8187x_state_s *priv);
static int  rtl8187x_uiptxpoll(struct uip_driver_s *dev);

/* RX/EX event handling */

static void rtl8187x_receive(FAR struct rtl8187x_state_s *priv);
static void rtl8187x_txdone(FAR struct rtl8187x_state_s *priv);

/* Watchdog timer expirations */

static void rtl8187x_polltimer(int argc, uint32_t arg, ...);
static void rtl8187x_txtimeout(int argc, uint32_t arg, ...);

/* NuttX callback functions */

static int rtl8187x_ifup(struct uip_driver_s *dev);
static int rtl8187x_ifdown(struct uip_driver_s *dev);
static int rtl8187x_txavail(struct uip_driver_s *dev);
#ifdef CONFIG_NET_IGMP
static int rtl8187x_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
static int rtl8187x_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
#endif

/* Register and unregister network device */

static int rtl8187x_initialize(FAR struct rtl8187x_state_s *priv);
static int rtl8187x_uninitialize(FAR struct rtl8187x_state_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure provides the registry entry ID informatino that will  be 
 * used to associate the USB class driver to a connected USB device.
 */

static const const struct usbhost_id_s g_id =
{
  USB_CLASS_VENDOR_SPEC,  /* base */
  0xff,                   /* subclass */
  0xff,                   /* proto */
  CONFIG_USB_WLAN_VID,    /* vid */
  CONFIG_USB_WLAN_PID     /* pid */
};

/* This is the USB host wireless LAN class's registry entry */

static struct usbhost_registry_s g_wlan =
{
  NULL,                   /* flink    */
  rtl8187x_create,        /* create   */
  1,                      /* nids     */
  &g_id                   /* id[]     */
};

/* This is a bitmap that is used to allocate device names /dev/wlana-z. */

static uint32_t g_devinuse;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8187x_takesem
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 ****************************************************************************/

static void rtl8187x_takesem(sem_t *sem)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(sem) != 0)
    {
      /* The only case that an error should occr here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: rtl8187x_allocclass
 *
 * Description:
 *   This is really part of the logic that implements the create() method
 *   of struct usbhost_registry_s.  This function allocates memory for one
 *   new class instance.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s.  NULL is returned on failure; this function will
 *   will fail only if there are insufficient resources to create another
 *   USB host class instance.
 *
 ****************************************************************************/

static inline FAR struct rtl8187x_state_s *rtl8187x_allocclass(void)
{
  FAR struct rtl8187x_state_s *priv;

  DEBUGASSERT(!up_interrupt_context());
  priv = (FAR struct rtl8187x_state_s *)malloc(sizeof(struct rtl8187x_state_s));
  uvdbg("Allocated: %p\n", priv);;
  return priv;
}

/****************************************************************************
 * Name: rtl8187x_freeclass
 *
 * Description:
 *   Free a class instance previously allocated by rtl8187x_allocclass().
 *
 * Input Parameters:
 *   class - A reference to the class instance to be freed.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline void rtl8187x_freeclass(FAR struct rtl8187x_state_s *class)
{
  DEBUGASSERT(class != NULL);

  /* Free the class instance (calling sched_free() in case we are executing
   * from an interrupt handler.
   */

  uvdbg("Freeing: %p\n", class);;
  free(class);
}

/****************************************************************************
 * Name: Device name management
 *
 * Description:
 *   Some tiny functions to coordinate management of device names.
 *
 ****************************************************************************/

static int rtl8187x_allocdevno(FAR struct rtl8187x_state_s *priv)
{
  irqstate_t flags;
  int devno;

  flags = irqsave();
  for (devno = 0; devno < 26; devno++)
    {
      uint32_t bitno = 1 << devno;
      if ((g_devinuse & bitno) == 0)
        {
          g_devinuse |= bitno;
          priv->devchar = 'a' + devno;
          irqrestore(flags);
          return OK;
        }
    }

  irqrestore(flags);
  return -EMFILE;
}

static void rtl8187x_freedevno(FAR struct rtl8187x_state_s *priv)
{
  int devno = 'a' - priv->devchar;

  if (devno >= 0 && devno < 26)
    {
      irqstate_t flags = irqsave();
      g_devinuse &= ~(1 << devno);
      irqrestore(flags);
    }
}

static inline void rtl8187x_mkdevname(FAR struct rtl8187x_state_s *priv, char *devname)
{
  (void)snprintf(devname, DEV_NAMELEN, DEV_FORMAT, priv->devchar);
}

/****************************************************************************
 * Name: rtl8187x_destroy
 *
 * Description:
 *   The USB device has been disconnected and the refernce count on the USB
 *   host class instance has gone to 1.. Time to destroy the USB host class
 *   instance.
 *
 * Input Parameters:
 *   arg - A reference to the class instance to be destroyed.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void rtl8187x_destroy(FAR void *arg)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)arg;

  DEBUGASSERT(priv != NULL);
  uvdbg("crefs: %d\n", priv->crefs);
 
  /* Unregister the driver */

  /* Release the device name used by this connection */

  rtl8187x_freedevno(priv);

  /* Free the endpoints */

  /* Free any transfer buffers */

  rtl8187x_tfree(priv);

  /* Destroy the semaphores */

  /* Disconnect the USB host device */

  DRVR_DISCONNECT(priv->drvr);

  /* And free the class instance.  Hmmm.. this may execute on the worker
   * thread and the work structure is part of what is getting freed.  That
   * should be okay because once the work contained is removed from the
   * queue, it should not longer be accessed by the worker thread.
   */

  rtl8187x_freeclass(priv);
}

/****************************************************************************
 * Name: rtl8187x_cfgdesc
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   priv - The USB host class instance.
 *   configdesc - A pointer to a uint8_t buffer container the configuration descripor.
 *   desclen - The length in bytes of the configuration descriptor.
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static inline int rtl8187x_cfgdesc(FAR struct rtl8187x_state_s *priv,
                                   FAR const uint8_t *configdesc, int desclen,
                                   uint8_t funcaddr)
{
  FAR struct usb_cfgdesc_s *cfgdesc;
  FAR struct usb_desc_s *desc;
  FAR struct usbhost_epdesc_s bindesc;
  FAR struct usbhost_epdesc_s boutdesc;
  int remaining;
  uint8_t found = 0;
  int ret;

  DEBUGASSERT(priv != NULL && 
              configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));
  
  /* Verify that we were passed a configuration descriptor */

  cfgdesc = (FAR struct usb_cfgdesc_s *)configdesc;
  if (cfgdesc->type != USB_DESC_TYPE_CONFIG)
    {
      return -EINVAL;
    }

  /* Get the total length of the configuration descriptor (little endian).
   * It might be a good check to get the number of interfaces here too.
  */

  remaining = (int)rtl8187x_getle16(cfgdesc->totallen);

  /* Skip to the next entry descriptor */

  configdesc += cfgdesc->len;
  remaining  -= cfgdesc->len;

  /* Loop where there are more dscriptors to examine */

  while (remaining >= sizeof(struct usb_desc_s))
    {
      /* What is the next descriptor? */

      desc = (FAR struct usb_desc_s *)configdesc;
      switch (desc->type)
        {
        /* Interface descriptor. We really should get the number of endpoints
         * from this descriptor too.
         */

        case USB_DESC_TYPE_INTERFACE:
          {
            FAR struct usb_ifdesc_s *ifdesc = (FAR struct usb_ifdesc_s *)configdesc;
 
            uvdbg("Interface descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_IFDESC);

            /* Save the interface number and mark ONLY the interface found */

            priv->ifno = ifdesc->ifno;
            found      = USBHOST_IFFOUND;
          }
          break;

        /* Endpoint descriptor.  Here, we expect two bulk endpoints, an IN
         * and an OUT.
         */

        case USB_DESC_TYPE_ENDPOINT:
          {
            FAR struct usb_epdesc_s *epdesc = (FAR struct usb_epdesc_s *)configdesc;

            uvdbg("Endpoint descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_EPDESC);

            /* Check for a bulk endpoint. */

            if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) == USB_EP_ATTR_XFER_BULK)
              {
                /* Yes.. it is a bulk endpoint.  IN or OUT? */

                if (USB_ISEPOUT(epdesc->addr))
                  {
                    /* It is an OUT bulk endpoint.  There should be only one
                     * bulk OUT endpoint.
                     */

                    if ((found & USBHOST_BOUTFOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know
                         * what to do with this.
                         */

                        return -EINVAL;
                      }
                    found |= USBHOST_BOUTFOUND;

                    /* Save the bulk OUT endpoint information */

                    boutdesc.addr         = epdesc->addr & USB_EP_ADDR_NUMBER_MASK;
                    boutdesc.in           = false;
                    boutdesc.funcaddr     = funcaddr;
                    boutdesc.xfrtype      = USB_EP_ATTR_XFER_BULK;
                    boutdesc.interval     = epdesc->interval;
                    boutdesc.mxpacketsize = rtl8187x_getle16(epdesc->mxpacketsize);
                    uvdbg("Bulk OUT EP addr:%d mxpacketsize:%d\n",
                          boutdesc.addr, boutdesc.mxpacketsize);
                  }
                else
                  {
                    /* It is an IN bulk endpoint.  There should be only one
                     * bulk IN endpoint.
                     */

                    if ((found & USBHOST_BINFOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know
                         * what to do with this.
                         */

                        return -EINVAL;
                      }
                    found |= USBHOST_BINFOUND;

                    /* Save the bulk IN endpoint information */
                    
                    bindesc.addr         = epdesc->addr & USB_EP_ADDR_NUMBER_MASK;
                    bindesc.in           = 1;
                    bindesc.funcaddr     = funcaddr;
                    bindesc.xfrtype      = USB_EP_ATTR_XFER_BULK;
                    bindesc.interval     = epdesc->interval;
                    bindesc.mxpacketsize = rtl8187x_getle16(epdesc->mxpacketsize);
                    uvdbg("Bulk IN EP addr:%d mxpacketsize:%d\n",
                          bindesc.addr, bindesc.mxpacketsize);
                  }
              }
          }
          break;

        /* Other descriptors are just ignored for now */

        default:
          break;
        }

      /* If we found everything we need with this interface, then break out
       * of the loop early.
       */

      if (found == USBHOST_ALLFOUND)
        {
          break;
        }

      /* Increment the address of the next descriptor */
 
      configdesc += desc->len;
      remaining  -= desc->len;
    }

  /* Sanity checking... did we find all of things that we need? */
    
  if (found != USBHOST_ALLFOUND)
    {
      ulldbg("ERROR: Found IF:%s BIN:%s BOUT:%s\n",
             (found & USBHOST_IFFOUND) != 0  ? "YES" : "NO",
             (found & USBHOST_BINFOUND) != 0 ? "YES" : "NO",
             (found & USBHOST_BOUTFOUND) != 0 ? "YES" : "NO");
      return -EINVAL;
    }

  /* We are good... Allocate the endpoints */

  ret = DRVR_EPALLOC(priv->drvr, &boutdesc, &priv->epout);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate Bulk OUT endpoint\n");
      return ret;
    }

  ret = DRVR_EPALLOC(priv->drvr, &bindesc, &priv->epin);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate Bulk IN endpoint\n");
      (void)DRVR_EPFREE(priv->drvr, priv->epout);
      return ret;
    }

  ullvdbg("Endpoints allocated\n");
  return OK;
}

/****************************************************************************
 * Name: rtl8187x_devinit
 *
 * Description:
 *   The USB device has been successfully connected.  This completes the
 *   initialization operations.  It is first called after the
 *   configuration descriptor has been received.
 *
 *   This function is called from the connect() method.  This function always
 *   executes on the thread of the caller of connect().
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline int rtl8187x_devinit(FAR struct rtl8187x_state_s *priv)
{
  int ret = OK;

  /* Set aside a transfer buffer for exclusive use by the class driver */

  /* Increment the reference count.  This will prevent rtl8187x_destroy() from
   * being called asynchronously if the device is removed.
   */

  priv->crefs++;
  DEBUGASSERT(priv->crefs == 2);

  /* Configure the device */

  /* Register the driver */

  if (ret == OK)
    {
      char devname[DEV_NAMELEN];

      uvdbg("Register block driver\n");
      rtl8187x_mkdevname(priv, devname);
      ret = rtl8187x_initialize(priv);
      // ret = register_blockdriver(devname, &g_bops, 0, priv);
    }

  /* Check if we successfully initialized. We now have to be concerned
   * about asynchronous modification of crefs because the block
   * driver has been registered.
   */

  if (ret == OK)
    {
      rtl8187x_takesem(&priv->exclsem);
      DEBUGASSERT(priv->crefs >= 2);

      /* Handle a corner case where (1) open() has been called so the
       * reference count is > 2, but the device has been disconnected.
       * In this case, the class instance needs to persist until close()
       * is called.
       */

      if (priv->crefs <= 2 && priv->disconnected)
        {
          /* We don't have to give the semaphore because it will be
           * destroyed when usb_destroy is called.
           */
  
          ret = -ENODEV;
        }
      else
        {
          /* Ready for normal operation as a block device driver */

          uvdbg("Successfully initialized\n");
          priv->crefs--;
          rtl8187x_givesem(&priv->exclsem);
        }
    }

  /* Disconnect on any errors detected during volume initialization */

  if (ret != OK)
    {
      udbg("ERROR! Aborting: %d\n", ret);
      rtl8187x_destroy(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: rtl8187x_host2le16 and rtl8187x_host2le32
 *
 * Description:
 *   Convert a 16/32-bit value in host byte order to little endian byte order.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the little endian value.
 *
 * Returned Values:
 *   A uint16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

static inline uint16_t rtl8187x_host2le16(uint16_t val)
{
#ifdef CONFIG_ENDIAN_BIG
  uint16_t ret = ((val & 0x00ff) << 8) |
                 ((val)) >> 8) & 0x00ff))
  return ret
#else
  return val;
#endif
}

static inline uint32_t rtl8187x_host2le32(uint32_t val)
{
#ifdef CONFIG_ENDIAN_BIG
  uint32_t ret = ((val & 0x000000ffL) << 24) |
                 ((val & 0x0000ff00L) <<  8) |
                 ((val & 0x00ff0000L) >>  8) |
                 ((val & 0xff000000L) >> 24))
  return ret
#else
  return val;
#endif
}

/****************************************************************************
 * Name: rtl8187x_getle16 and rtl8187x_getle32
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the little endian value.
 *
 * Returned Values:
 *   A uint16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

static inline uint16_t rtl8187x_getle16(const uint8_t *val)
{
  /* Little endian means LS byte first in byte stream */

  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

static inline uint32_t rtl8187x_getle32(const uint8_t *val)
{
 /* Little endian means LS halfword first in byte stream */

  return (uint32_t)rtl8187x_getle16(&val[2]) << 16 | (uint32_t)rtl8187x_getle16(val);
}

/****************************************************************************
 * Name: rtl8187x_putle16 and  rtl8187x_putle32
 *
 * Description:
 *   Put a (possibly unaligned) 16/32-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 16-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void rtl8187x_putle16(uint8_t *dest, uint16_t val)
{
  /* Little endian means LS byte first in byte stream */

  dest[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  dest[1] = val >> 8;
}

static void rtl8187x_putle32(uint8_t *dest, uint32_t val)
{
  /* Little endian means LS halfword first in byte stream */

  rtl8187x_putle16(dest, (uint16_t)(val & 0xffff));
  rtl8187x_putle16(dest+2, (uint16_t)(val >> 16));
}

/****************************************************************************
 * Name: rtl8187x_talloc
 *
 * Description:
 *   Allocate transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   On sucess, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int rtl8187x_talloc(FAR struct rtl8187x_state_s *priv)
{
  size_t maxlen;
  int ret;

  DEBUGASSERT(priv && priv->ctrlreq == NULL && priv->tbuffer == NULL);

  /* Allocate TD buffers for use in this driver.  We will need two:  One for
   * the request and one for the data buffer.
   */

  ret = DRVR_ALLOC(priv->drvr, (FAR uint8_t **)&priv->ctrlreq, &maxlen);
  if (ret != OK)
    {
      uvdbg("DRVR_ALLOC(ctrlreq) failed: %d\n", ret);
      return ret;
    }

  ret = DRVR_ALLOC(priv->drvr, &priv->tbuffer, &priv->tbuflen);
  if (ret != OK)
    {
      uvdbg("DRVR_ALLOC(tbuffer) failed: %d\n", ret);
    }
  return ret;
}

/****************************************************************************
 * Name: rtl8187x_tfree
 *
 * Description:
 *   Free transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   On sucess, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int rtl8187x_tfree(FAR struct rtl8187x_state_s *priv)
{
  DEBUGASSERT(priv);

  if (priv->ctrlreq)
    {
      DEBUGASSERT(priv->drvr && priv->tbuffer);

      /* Free the allocated control request */

      (void)DRVR_FREE(priv->drvr, (FAR uint8_t *)priv->ctrlreq);
      priv->ctrlreq = NULL;
      
      /* Free the allocated buffer */

      (void)DRVR_FREE(priv->drvr, priv->tbuffer);
      priv->tbuffer = NULL;
      priv->tbuflen = 0;
    }
  return OK;
}

/****************************************************************************
 * struct usbhost_registry_s methods
 ****************************************************************************/

/****************************************************************************
 * Name: rtl8187x_create
 *
 * Description:
 *   This function implements the create() method of struct usbhost_registry_s. 
 *   The create() method is a callback into the class implementation.  It is
 *   used to (1) create a new instance of the USB host class state and to (2)
 *   bind a USB host driver "session" to the class instance.  Use of this
 *   create() method will support environments where there may be multiple
 *   USB ports and multiple USB devices simultaneously connected.
 *
 * Input Parameters:
 *   drvr - An instance of struct usbhost_driver_s that the class
 *     implementation will "bind" to its state structure and will
 *     subsequently use to communicate with the USB host driver.
 *   id - In the case where the device supports multiple base classes,
 *     subclasses, or protocols, this specifies which to configure for.
 *
 * Returned Values:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s that can be used by the USB host driver to communicate
 *   with the USB host class.  NULL is returned on failure; this function
 *   will fail only if the drvr input parameter is NULL or if there are
 *   insufficient resources to create another USB host class instance.
 *
 ****************************************************************************/

static FAR struct usbhost_class_s *rtl8187x_create(FAR struct usbhost_driver_s *drvr,
                                                  FAR const struct usbhost_id_s *id)
{
  FAR struct rtl8187x_state_s *priv;
  int ret;

  /* Allocate a USB host class instance */

  priv = rtl8187x_allocclass();
  if (priv)
    {
      /* Initialize the allocated storage class instance */

      memset(priv, 0, sizeof(struct rtl8187x_state_s));

      /* Allocate buffering */

      ret = rtl8187x_talloc(priv);
      if (ret != OK)
        {
          udbg("ERROR: Failed to allocate buffers: %d\n", ret);
          goto errout;
        }

      /* Assign a device number to this class instance */

      if (rtl8187x_allocdevno(priv) == OK)
        {
         /* Initialize class method function pointers */

          priv->class.connect      = rtl8187x_connect;
          priv->class.disconnected = rtl8187x_disconnected;

          /* The initial reference count is 1... One reference is held by the driver */

          priv->crefs              = 1;

          /* Initialize semphores (this works okay in the interrupt context) */

          sem_init(&priv->exclsem, 0, 1);

          /* Bind the driver to the storage class instance */

          priv->drvr               = drvr;

          /* Return the instance of the USB class driver */
 
          return &priv->class;
        }
    }

  /* An error occurred. Free the allocation and return NULL on all failures */

errout:
  if (priv)
    {
      rtl8187x_freeclass(priv);
    }
  return NULL;
}

/****************************************************************************
 * struct usbhost_class_s methods
 ****************************************************************************/
/****************************************************************************
 * Name: rtl8187x_connect
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   class - The USB host class entry previously obtained from a call to create().
 *   configdesc - A pointer to a uint8_t buffer container the configuration descripor.
 *   desclen - The length in bytes of the configuration descriptor.
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - This function will *not* be called from an interrupt handler.
 *   - If this function returns an error, the USB host controller driver
 *     must call to DISCONNECTED method to recover from the error
 *
 ****************************************************************************/

static int rtl8187x_connect(FAR struct usbhost_class_s *class,
                           FAR const uint8_t *configdesc, int desclen,
                           uint8_t funcaddr)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)class;
  int ret;

  DEBUGASSERT(priv != NULL && 
              configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));

  /* Parse the configuration descriptor to get the endpoints */

  ret = rtl8187x_cfgdesc(priv, configdesc, desclen, funcaddr);
  if (ret != OK)
    {
      udbg("rtl8187x_cfgdesc() failed: %d\n", ret);
    }
  else
    {
      /* Now configure the device and register the NuttX driver */

      ret = rtl8187x_devinit(priv);
      if (ret != OK)
        {
          udbg("rtl8187x_devinit() failed: %d\n", ret);
        }
    }
 
  return ret;
}

/****************************************************************************
 * Name: rtl8187x_disconnected
 *
 * Description:
 *   This function implements the disconnected() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to inform the class that the USB device has
 *   been disconnected.
 *
 * Input Parameters:
 *   class - The USB host class entry previously obtained from a call to
 *     create().
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

static int rtl8187x_disconnected(struct usbhost_class_s *class)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)class;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  /* Set an indication to any users of the device that the device is no
   * longer available.
   */

  flags              = irqsave();
  priv->disconnected = true;

  /* Now check the number of references on the class instance.  If it is one,
   * then we can free the class instance now.  Otherwise, we will have to
   * wait until the holders of the references free them by closing the
   * block driver.
   */

  ullvdbg("crefs: %d\n", priv->crefs);
  if (priv->crefs == 1)
    {
      /* Destroy the class instance.  If we are executing from an interrupt
       * handler, then defer the destruction to the worker thread.
       * Otherwise, destroy the instance now.
       */

      if (up_interrupt_context())
        {
          /* Destroy the instance on the worker thread. */

          uvdbg("Queuing destruction: worker %p->%p\n", priv->work.worker, rtl8187x_destroy);
          DEBUGASSERT(priv->work.worker == NULL);
          (void)work_queue(&priv->work, rtl8187x_destroy, priv, 0);
       }
      else
        {
          /* Do the work now */

          rtl8187x_destroy(priv);
        }
    }

  /* Unregister WLAN network interface */

  rtl8187x_uninitialize(0);

  irqrestore(flags);  
  return OK;
}

/****************************************************************************
 * Function: rtl8187x_ioread8/16/32
 *
 * Description:
 *   Read 8, 16, or 32 bits from the RTL8187x.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *   addr  - Device addresses
 *
 * Returned Value:
 *   The value read from the the RTL8187x (no error indication returned).
 *
 * Assumptions:
 *   Not called from an interrupt handler.
 *
 ****************************************************************************/

static uint8_t rtl8187x_ioread8(struct rtl8187x_state_s *priv, uint16_t addr)
{
  FAR struct usb_ctrlreq_s *ctrlreq = priv->ctrlreq;
  int ret;

  DEBUGASSERT(ctrlreq && priv->tbuffer);
  ctrlreq->type = (USB_REQ_DIR_IN | USB_REQ_TYPE_VENDOR | USB_REQ_RECIPIENT_DEVICE);
  ctrlreq->req  = 0x05;
  rtl8187x_putle16(ctrlreq->value, addr);
  rtl8187x_putle16(ctrlreq->index, 0);
  rtl8187x_putle16(ctrlreq->len, sizeof(uint8_t));

  ret = DRVR_CTRLIN(priv->drvr, priv->ctrlreq, priv->tbuffer);
  if (ret != OK)
    {
      udbg("ERROR: DRVR_CTRLIN returned %d\n", ret);
      return 0;
    }

  return *((uint8_t*)priv->tbuffer);
}

static uint16_t rtl8187x_ioread16(struct rtl8187x_state_s*priv, uint16_t addr)
{
  FAR struct usb_ctrlreq_s *ctrlreq = priv->ctrlreq;
  int ret;

  DEBUGASSERT(ctrlreq && priv->tbuffer);
  ctrlreq->type = (USB_REQ_DIR_IN | USB_REQ_TYPE_VENDOR | USB_REQ_RECIPIENT_DEVICE);
  ctrlreq->req  = 0x05;
  rtl8187x_putle16(ctrlreq->value, addr);
  rtl8187x_putle16(ctrlreq->index, 0);
  rtl8187x_putle16(ctrlreq->len, sizeof(uint16_t));

  ret = DRVR_CTRLIN(priv->drvr, priv->ctrlreq, priv->tbuffer);
  if (ret != OK)
    {
      udbg("ERROR: DRVR_CTRLIN returned %d\n", ret);
      return 0;
    }

  return rtl8187x_getle16(priv->tbuffer);
}

static uint32_t rtl8187x_ioread32(struct rtl8187x_state_s*priv, uint16_t addr)
{
  FAR struct usb_ctrlreq_s *ctrlreq = priv->ctrlreq;
  int ret;

  DEBUGASSERT(ctrlreq && priv->tbuffer);
  ctrlreq->type = (USB_REQ_DIR_IN | USB_REQ_TYPE_VENDOR | USB_REQ_RECIPIENT_DEVICE);
  ctrlreq->req  = 0x05;
  rtl8187x_putle16(ctrlreq->value, addr);
  rtl8187x_putle16(ctrlreq->index, 0);
  rtl8187x_putle16(ctrlreq->len, sizeof(uint32_t));

  ret = DRVR_CTRLIN(priv->drvr, priv->ctrlreq, priv->tbuffer);
  if (ret != OK)
    {
      udbg("ERROR: DRVR_CTRLIN returned %d\n", ret);
      return 0;
    }

  return rtl8187x_getle32(priv->tbuffer);
}

/****************************************************************************
 * Function: rtl8187x_iowrite8/16/32
 *
 * Description:
 *   Write a 8, 16, or 32 bits to the RTL8187x.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *   addr  - Device addresses
 *   val   - The value to write
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   Not called from an interrupt handler.
 *
 ****************************************************************************/

static int rtl8187x_iowrite8(struct rtl8187x_state_s *priv, uint16_t addr, uint8_t val)
{
  FAR struct usb_ctrlreq_s *ctrlreq = priv->ctrlreq;

  DEBUGASSERT(ctrlreq && priv->tbuffer);
  ctrlreq->type = (USB_REQ_DIR_OUT | USB_REQ_TYPE_VENDOR | USB_REQ_RECIPIENT_DEVICE);
  ctrlreq->req  = 0x05;
  rtl8187x_putle16(ctrlreq->value, addr);
  rtl8187x_putle16(ctrlreq->index, 0);
  rtl8187x_putle16(ctrlreq->len, sizeof(uint8_t));

  priv->tbuffer[0] = val;
  return DRVR_CTRLOUT(priv->drvr, priv->ctrlreq, priv->tbuffer);
}

static int rtl8187x_iowrite16(struct rtl8187x_state_s *priv, uint16_t addr, uint16_t val)
{
  FAR struct usb_ctrlreq_s *ctrlreq = priv->ctrlreq;

  DEBUGASSERT(ctrlreq && priv->tbuffer);
  ctrlreq->type = (USB_REQ_DIR_OUT | USB_REQ_TYPE_VENDOR | USB_REQ_RECIPIENT_DEVICE);
  ctrlreq->req  = 0x05;
  rtl8187x_putle16(ctrlreq->value, addr);
  rtl8187x_putle16(ctrlreq->index, 0);
  rtl8187x_putle16(ctrlreq->len, sizeof(uint16_t));

  rtl8187x_putle16(priv->tbuffer, val);
  return DRVR_CTRLOUT(priv->drvr, priv->ctrlreq, priv->tbuffer);
}

static int rtl8187x_iowrite32(struct rtl8187x_state_s *priv, uint16_t addr, uint32_t val)
{
  FAR struct usb_ctrlreq_s *ctrlreq = priv->ctrlreq;

  DEBUGASSERT(ctrlreq && priv->tbuffer);
  ctrlreq->type = (USB_REQ_DIR_OUT | USB_REQ_TYPE_VENDOR | USB_REQ_RECIPIENT_DEVICE);
  ctrlreq->req  = 0x05;
  rtl8187x_putle16(ctrlreq->value, addr);
  rtl8187x_putle16(ctrlreq->index, 0);
  rtl8187x_putle16(ctrlreq->len, sizeof(uint32_t));

  rtl8187x_putle32(priv->tbuffer, val);
  return DRVR_CTRLOUT(priv->drvr, priv->ctrlreq, priv->tbuffer);
}

/****************************************************************************
 * Function: rtl8187x_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Parameters:
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

static int rtl8187x_transmit(FAR struct rtl8187x_state_s *priv)
{
  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is not transmission in progress.
   */

  /* Increment statistics */

  /* Send the packet: address=priv->ethdev.d_buf, length=priv->ethdev.d_len */

  /* Enable Tx interrupts */

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  (void)wd_start(priv->txtimeout, WLAN_TXTIMEOUT, rtl8187x_txtimeout, 1, (uint32_t)priv);
  return OK;
}

/****************************************************************************
 * Function: rtl8187x_uiptxpoll
 *
 * Description:
 *   The transmitter is available, check if uIP has any outgoing packets ready
 *   to send.  This is a callback from uip_poll().  uip_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Parameters:
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

static int rtl8187x_uiptxpoll(struct uip_driver_s *dev)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->ethdev.d_len > 0)
    {
      uip_arp_out(&priv->ethdev);
      rtl8187x_transmit(priv);

      /* Check if there is room in the device to hold another packet. If not,
       * return a non-zero value to terminate the poll.
       */
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: rtl8187x_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void rtl8187x_receive(FAR struct rtl8187x_state_s *priv)
{
  do
    {
      /* Check for errors and update statistics */

      /* Check if the packet is a valid size for the uIP buffer configuration */

      /* Copy the data data from the hardware to priv->ethdev.d_buf.  Set
       * amount of data in priv->ethdev.d_len
       */

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(UIP_ETHTYPE_IP6))
#else
      if (BUF->type == HTONS(UIP_ETHTYPE_IP))
#endif
        {
          uip_arp_ipin(&priv->ethdev);
          uip_input(&priv->ethdev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->ethdev.d_len > 0)
           {
             uip_arp_out(&priv->ethdev);
             rtl8187x_transmit(priv);
           }
        }
      else if (BUF->type == htons(UIP_ETHTYPE_ARP))
        {
          uip_arp_arpin(&priv->ethdev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->ethdev.d_len > 0)
            {
              rtl8187x_transmit(priv);
            }
        }
    }
  while (0); /* While there are more packets to be processed */
}

/****************************************************************************
 * Function: rtl8187x_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void rtl8187x_txdone(FAR struct rtl8187x_state_s *priv)
{
  /* Check for errors and update statistics */

  /* If no further xmits are pending, then cancel the TX timeout and
   * disable further Tx interrupts.
   */

  wd_cancel(priv->txtimeout);

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&priv->ethdev, rtl8187x_uiptxpoll);
}

/****************************************************************************
 * Function: rtl8187x_txtimeout
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Parameters:
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

static void rtl8187x_txtimeout(int argc, uint32_t arg, ...)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)arg;

  /* Increment statistics and dump debug info */

  /* Then reset the hardware */

  /* Then poll uIP for new XMIT data */

  (void)uip_poll(&priv->ethdev, rtl8187x_uiptxpoll);
}

/****************************************************************************
 * Function: rtl8187x_polltimer
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Parameters:
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

static void rtl8187x_polltimer(int argc, uint32_t arg, ...)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)arg;

  /* Check if there is room in the send another TX packet.  We cannot perform
   * the TX poll if he are unable to accept another packet for transmission.
   */

  /* If so, update TCP timing states and poll uIP for new XMIT data. Hmmm..
   * might be bug here.  Does this mean if there is a transmit in progress,
   * we will missing TCP time state updates?
   */

  (void)uip_timer(&priv->ethdev, rtl8187x_uiptxpoll, WLAN_POLLHSEC);

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->txpoll, WLAN_WDDELAY, rtl8187x_polltimer, 1, arg);
}

/****************************************************************************
 * Function: rtl8187x_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the WLAN interface when an IP address is
 *   provided
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int rtl8187x_ifup(struct uip_driver_s *dev)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)dev->d_private;

  ndbg("Bringing up: %d.%d.%d.%d\n",
       dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
       (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24 );

  /* Initialize PHYs, the WLAN interface, and setup up WLAN interrupts */

  /* Set and activate a timer process */

  (void)wd_start(priv->txpoll, WLAN_WDDELAY, rtl8187x_polltimer, 1, (uint32_t)priv);

  /* Enable the WLAN interrupt */

  priv->bifup = true;
  return OK;
}

/****************************************************************************
 * Function: rtl8187x_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int rtl8187x_ifdown(struct uip_driver_s *dev)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)dev->d_private;
  irqstate_t flags;

  /* Cancel the TX poll timer and TX timeout timers */

  flags = irqsave();
  wd_cancel(priv->txpoll);
  wd_cancel(priv->txtimeout);

  /* Put the the EMAC is its reset, non-operational state.  This should be
   * a known configuration that will guarantee the rtl8187x_ifup() always
   * successfully brings the interface back up.
   */

  /* Mark the device "down" */

  priv->bifup = false;
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: rtl8187x_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int rtl8187x_txavail(struct uip_driver_s *dev)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)dev->d_private;
  irqstate_t flags;

  /* Disable interrupts because this function may be called from interrupt
   * level processing.
   */

  flags = irqsave();

  /* Ignore the notification if the interface is not yet up */

  if (priv->bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing packet. */

      /* If so, then poll uIP for new XMIT data */

      (void)uip_poll(&priv->ethdev, rtl8187x_uiptxpoll);
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: rtl8187x_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int rtl8187x_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Function: rtl8187x_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int rtl8187x_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct rtl8187x_state_s *priv = (FAR struct rtl8187x_state_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Function: rtl8187x_initialize
 *
 * Description:
 *   Initialize the WLAN controller and driver
 *
 * Parameters:
 *   intf - In the case where there are multiple EMACs, this value
 *          identifies which EMAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int rtl8187x_initialize(FAR struct rtl8187x_state_s *priv)
{
  /* Initialize the driver structure */

  priv->ethdev.d_ifup    = rtl8187x_ifup;     /* I/F down callback */
  priv->ethdev.d_ifdown  = rtl8187x_ifdown;   /* I/F up (new IP address) callback */
  priv->ethdev.d_txavail = rtl8187x_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  priv->ethdev.d_addmac  = rtl8187x_addmac;   /* Add multicast MAC address */
  priv->ethdev.d_rmmac   = rtl8187x_rmmac;    /* Remove multicast MAC address */
#endif
  priv->ethdev.d_private = (void*)priv;       /* Used to recover private state from dev */

  /* Create a watchdog for timing polling for and timing of transmisstions */

  priv->txpoll           = wd_create();       /* Create periodic poll timer */
  priv->txtimeout        = wd_create();       /* Create TX timeout timer */

  /* Put the interface in the down state. */

  rtl8187x_ifdown(&priv->ethdev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&priv->ethdev);
  return OK;
}

/****************************************************************************
 * Function: rtl8187x_initialize
 *
 * Description:
 *   Initialize the WLAN controller and driver
 *
 * Parameters:
 *   intf - In the case where there are multiple EMACs, this value
 *          identifies which EMAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int rtl8187x_uninitialize(FAR struct rtl8187x_state_s *priv)
{
  /* Unregister the device */

  (void)netdev_unregister(&priv->ethdev);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_wlaninit
 *
 * Description:
 *   Initialize the USB class driver.  This function should be called
 *   be platform-specific code in order to initialize and register support
 *   for the USB host class device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

int usbhost_wlaninit(void)
{
  /* Perform any one-time initialization of the class implementation */

  /* Advertise our availability to support (certain) devices */

  return usbhost_registerclass(&g_wlan);
}

#endif /* CONFIG_USBHOST && CONFIG_NET && CONFIG_NET_WLAN */


