/****************************************************************************
 * net/can/can_callback.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_CAN)

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/netstats.h>

#include "devif/devif.h"
#include "can/can.h"

#ifdef CONFIG_NET_TIMESTAMP
#include <sys/time.h>
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_NET_CAN_SOCK_RXBUF

/****************************************************************************
 * Name: can_data_event
 *
 * Description:
 *   Handle data that is not accepted by the application because there is no
 *   listener in place ready to receive the data.
 *
 * Assumptions:
 * - The caller has checked that CAN_NEWDATA is set in flags and that is no
 *   other handler available to process the incoming data.
 * - This function must be called with the network locked.
 *
 ****************************************************************************/

static inline uint16_t
can_data_event(FAR struct net_driver_s *dev, FAR struct can_conn_s *conn,
               uint16_t flags)
{
  int buflen = dev->d_len;
  uint16_t recvlen;
  uint16_t ret;

#ifdef CONFIG_NET_TIMESTAMP
  buflen -= sizeof(struct timeval);
#endif

  ret = (flags & ~CAN_NEWDATA);

  /* Save as the packet data as in the read-ahead buffer.  NOTE that
   * partial packets will not be buffered.
   */

  recvlen = can_datahandler(dev, conn);
  if (recvlen < buflen)
    {
      /* There is no handler to receive new data and there are no free
       * read-ahead buffers to retain the data -- drop the packet.
       */

      ninfo("Dropped %d bytes\n", dev->d_len);

#ifdef CONFIG_NET_STATISTICS
      g_netstats.can.drop++;
#endif
      NETDEV_RXDROPPED(dev);
    }

  /* In any event, the new data has now been handled */

  dev->d_len = 0;
  return ret;
}
#endif /* !CONFIG_NET_CAN_SOCK_RXBUF */

#ifdef CONFIG_NET_CAN_SOCK_RXBUF

/****************************************************************************
 * Name: can_accepts
 *
 * Description:
 *   Check whether a socket wants a received frame.  This runs before the
 *   frame is copied into the receive buffer of the socket, so a
 *   frame the socket does not want costs neither a slot nor a copy.
 *
 * Input Parameters:
 *   dev  - The device which was active when the frame was received
 *   conn - A pointer to the CAN connection structure
 *
 * Returned Value:
 *   True if the socket wants the frame.
 *
 * Assumptions:
 *   This function can be called from an interrupt.
 *
 ****************************************************************************/

static bool can_accepts(FAR struct net_driver_s *dev,
                        FAR struct can_conn_s *conn)
{
#ifdef CONFIG_NET_CANPROTO_OPTIONS
  canid_t can_id;

  memcpy(&can_id, dev->d_appdata, sizeof(canid_t));
  if (can_recv_filter(conn, can_id) == 0)
    {
      return false;
    }
#endif

  /* Do not pass frames with DLC > 8 to a legacy socket */

#if defined(CONFIG_NET_CANPROTO_OPTIONS) && defined(CONFIG_NET_CAN_CANFD)
  if (!_SO_GETOPT(conn->sconn.s_options, CAN_RAW_FD_FRAMES))
#endif
    {
      if (dev->d_len > sizeof(struct can_frame))
        {
          return false;
        }
    }

  return true;
}

/****************************************************************************
 * Name: can_rxq_push
 *
 * Description:
 *   Append one received frame to the receive buffer of a socket as
 *   a struct can_rxhdr_s followed by the frame itself.  The record carries
 *   the arrival time if the socket asked for SO_TIMESTAMP.
 *
 * Input Parameters:
 *   conn  - The CAN connection that is to retain the frame
 *   frame - The frame, in the device buffer
 *   len   - The length of the frame
 *
 * Returned Value:
 *   The number of frame bytes retained, or -ENOBUFS if the record does not
 *   fit in the free space or would exceed the SO_RCVBUF limit of the
 *   socket.
 *
 * Assumptions:
 *   This function can be called from an interrupt.
 *
 ****************************************************************************/

static int can_rxq_push(FAR struct can_conn_s *conn,
                        FAR const void *frame, uint16_t len)
{
  struct can_rxhdr_s hdr;
#ifdef CONFIG_NET_TIMESTAMP
  struct timespec ts;
#endif
  irqstate_t flags;
  size_t record;

  record  = sizeof(hdr) + len;
  hdr.len = len;

#ifdef CONFIG_NET_TIMESTAMP
  /* Every record carries the arrival time, so a frame retained before the
   * socket set SO_TIMESTAMP still has one.  recvmsg() hands it out only
   * when the option is set.  The clock is read outside the lock below.
   */

  clock_systime_timespec(&ts);
  hdr.ts.tv_sec  = ts.tv_sec;
  hdr.ts.tv_usec = ts.tv_nsec / 1000;
#endif

  /* Some CAN drivers deliver a received frame from their receive interrupt
   * handler, so the buffer is guarded by an irqsave spinlock.
   */

  flags = spin_lock_irqsave(&conn->rxq_lock);

  if (circbuf_space(&conn->rxq) < record ||
      circbuf_used(&conn->rxq) + record > CAN_RXQ_LIMIT(conn))
    {
      spin_unlock_irqrestore(&conn->rxq_lock, flags);
      return -ENOBUFS;
    }

  circbuf_write(&conn->rxq, &hdr, sizeof(hdr));
  circbuf_write(&conn->rxq, frame, len);

  spin_unlock_irqrestore(&conn->rxq_lock, flags);
  return len;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_callback
 *
 * Description:
 *   Inform the application holding the packet socket of a change in state.
 *
 * Returned Value:
 *   OK if packet has been processed, otherwise ERROR.
 *
 * Assumptions:
 *   This function can be called from an interrupt.
 *
 ****************************************************************************/

uint16_t can_callback(FAR struct net_driver_s *dev,
                      FAR struct can_conn_s *conn, uint16_t flags)
{
#ifdef CONFIG_NET_CAN_SOCK_RXBUF
  bool newdata = (flags & CAN_NEWDATA) != 0;

  /* Some sanity checking */

  if (conn)
    {
      if (newdata)
        {
          if (!can_accepts(dev, conn))
            {
              /* The filters of the socket, or its CAN_RAW_FD_FRAMES
               * setting, reject the frame.  Nothing is copied and nothing
               * is dropped: another socket may still want it.
               */

              dev->d_len = 0;
              return flags & ~CAN_NEWDATA;
            }

          /* Retain the frame in the receive buffer of this
           * socket before any listener runs, so a reader always finds it
           * there and the shared I/O buffer pool is never touched.
           */

          if (can_rxq_push(conn, dev->d_appdata, dev->d_len) < 0)
            {
              ninfo("Dropped %d bytes\n", dev->d_len);

#ifdef CONFIG_NET_STATISTICS
              g_netstats.can.drop++;
#endif
              NETDEV_RXDROPPED(dev);
            }
          else
            {
#ifdef CONFIG_NET_CAN_NOTIFIER
              /* Provide notification(s) that additional CAN read-ahead
               * data is available.
               */

              can_readahead_signal(conn);
#endif
              /* Run the worker the socket registered with
               * CAN_RAW_RXNOTIFY.  A driver that delivers a batch of
               * frames in one pass queues it on the first frame of the
               * batch, so the socket is woken once per batch.
               */

              can_rxnotify(conn);
            }

          /* The listeners run even when the frame was dropped, so a reader
           * blocked on an already full buffer is woken and takes the
           * oldest record out of it.
           */
        }

      /* Deliver the event now if the network can be locked without
       * blocking.  Otherwise the frame stays retained without a wakeup;
       * the next frame, recvmsg() or poll() finds it.
       */

      if (net_trylock() == OK)
        {
          flags = devif_conn_event(dev, flags, conn->sconn.list);
          net_unlock();
        }

      if (newdata)
        {
          /* The frame is retained whether or not a listener took it, so
           * the device buffer is free again.
           */

          flags &= ~CAN_NEWDATA;
          dev->d_len = 0;
        }
    }

  return flags;
#else
  /* Some sanity checking */

  if (conn)
    {
#ifdef CONFIG_NET_TIMESTAMP
          /* TIMESTAMP sockopt is activated,
           * create timestamp and copy to iob
           */

          if (_SO_GETOPT(conn->sconn.s_options, SO_TIMESTAMP) &&
            (dev->d_iob != NULL))
            {
              struct timeval tv;
              FAR struct timespec *ts = (FAR struct timespec *)&tv;
              int len;

              clock_systime_timespec(ts);
              tv.tv_usec = ts->tv_nsec / 1000;

              len = iob_trycopyin(dev->d_iob, (FAR uint8_t *)&tv,
                                  sizeof(struct timeval),
                                  -CONFIG_NET_LL_GUARDSIZE, false);
              if (len == sizeof(struct timeval))
                {
                  dev->d_len += len;
                }
            }
#endif

      /* Try to lock the network when successful send data to the listener */

      if (net_trylock() == OK)
        {
          flags = devif_conn_event(dev, flags, conn->sconn.list);
          net_unlock();
        }

      /* Either we did not get the lock or there is no application listening
       * If we did not get a lock we store the frame in the read-ahead buffer
       */

      if ((flags & CAN_NEWDATA) != 0)
        {
          /* Data was not handled.. dispose of it appropriately */

          flags = can_data_event(dev, conn, flags);
        }
    }

  return flags;
#endif
}

#ifndef CONFIG_NET_CAN_SOCK_RXBUF

/****************************************************************************
 * Name: can_datahandler
 *
 * Description:
 *   Handle data that is not accepted by the application.  This may be called
 *   either (1) from the data receive logic if it cannot buffer the data, or
 *   (2) from the CAN event logic is there is no listener in place ready to
 *   receive the data.
 *
 * Input Parameters:
 *   dev  - The device which as active when the event was detected.
 *   conn - A pointer to the CAN connection structure
 *
 * Returned Value:
 *   The number of bytes actually buffered is returned.  This will be either
 *   zero or equal to buflen; partial packets are not buffered.
 *
 * Assumptions:
 * - The caller has checked that CAN_NEWDATA is set in flags and that is no
 *   other handler available to process the incoming data.
 * - This function must be called with the network locked.
 *
 ****************************************************************************/

uint16_t can_datahandler(FAR struct net_driver_s *dev,
                         FAR struct can_conn_s *conn)
{
  FAR struct iob_s *iob = dev->d_iob;
  int ret = 0;

#if CONFIG_NET_RECV_BUFSIZE > 0
  /* Check the frame count pending on conn->readahead */

  if (iob_get_queue_entry_count(&conn->readahead) >= conn->recv_buffnum)
    {
      nwarn("WARNING: There are no free receive buffer to retain the data. "
            "Receive buffer number:%"PRId32", received frames:%"PRIuPTR" \n",
            conn->recv_buffnum, iob_get_queue_entry_count(&conn->readahead));
      goto errout;
    }
#endif

  /* Concat the iob to readahead */

  ret = iob_tryadd_queue(iob, &conn->readahead);
  if (ret >= 0)
    {
#ifdef CONFIG_NET_CAN_NOTIFIER
      /* Provide notification(s) that additional CAN read-ahead data is
       * available.
       */

      can_readahead_signal(conn);
#endif
      /* Run the worker the socket registered with CAN_RAW_RXNOTIFY.  A
       * driver that delivers a batch of frames in one pass queues it on
       * the first frame of the batch, so the socket is woken once per
       * batch.
       */

      can_rxnotify(conn);

      ret = iob->io_pktlen;

      /* Device buffer has been enqueued, clear the handle */

      netdev_iob_clear(dev);
    }
  else
    {
      nerr("ERROR: Failed to queue the I/O buffer chain: %d\n", ret);
      ret = 0;
      goto errout;
    }

  return ret;

errout:
  netdev_iob_release(dev);
  return ret;
}
#endif /* !CONFIG_NET_CAN_SOCK_RXBUF */

#endif /* CONFIG_NET && CONFIG_NET_CAN */
