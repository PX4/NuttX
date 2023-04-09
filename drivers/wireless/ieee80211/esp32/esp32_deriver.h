//
// Created by zhengweiqian on 9/4/2023.
//

#ifndef PX4_AUTOPILOT_ESP32_DERIVER_H
#define PX4_AUTOPILOT_ESP32_DERIVER_H

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/net/netdev.h>
#include <nuttx/semaphore.h>
#include <net/if.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>

#define ESP32_EVENT_COUNT 128

struct esp32_frame_s;
struct esp32_bus_dev_s;


struct esp32_dev_s
{
    FAR struct esp32_bus_dev_s *bus; /* Bus interface structure */

    bool bc_bifup;             /* true:ifup false:ifdown */
    struct wdog_s bc_txpoll;   /* TX poll timer */
    struct work_s bc_irqwork;  /* For deferring interrupt work to the work queue */
    struct work_s bc_pollwork; /* For deferring poll work to the work queue */

    /* This holds the information visible to the NuttX network */

    struct net_driver_s bc_dev;        /* Network interface structure */
    struct esp32_frame_s *cur_tx_frame; /* Frame used to interface network layer */

    /* Event registration array */

    event_handler_t event_handlers[ESP32_EVENT_COUNT];

    sem_t control_mutex;         /* Cannot handle multiple control requests */
    sem_t control_timeout;       /* Semaphore to wait for control frame rsp */
    uint16_t control_reqid;      /* Current control request id */
    uint16_t control_rxdata_len; /* Received control frame out buffer length */
    uint8_t *control_rxdata;     /* Received control frame out buffer */
    uint32_t control_status;     /* Last received frame status */

    /* AP Scan state machine.
     * During scan, control_mutex is locked to prevent control requests
     */

    int scan_status;                     /* Current scan status */
    struct wdog_s scan_timeout;          /* Scan timeout timer */
    FAR uint8_t *scan_result;            /* Temp buffer that holds results */
    unsigned int scan_result_size;       /* Current size of temp buffer */

    sem_t auth_signal; /* Authentication notification signal */
    int   auth_status; /* Authentication status */
};


struct esp32_bus_dev_s
{
    void (*stop)(FAR struct esp32_dev_s *priv);
    int (*txframe)(FAR struct esp32_dev_s *priv, struct esp32_frame_s *frame,
                   bool control);
    struct esp32_frame_s *(*rxframe)(FAR struct esp32_dev_s *priv);

    /* Frame buffer allocation primitives
     * len     - requested payload length
     * control - true if control frame else false
     * block   - true to block until free frame is available
     */

    struct esp32_frame_s *(*allocate_frame)(FAR struct esp32_dev_s *priv,
                                           unsigned int len, bool block,
                                           bool control);

    void (*free_frame)(FAR struct esp32_dev_s *priv,
                       FAR struct esp32_frame_s *frame);
};

struct esp32_frame_s
{
    uint8_t *base; /* Frame base buffer used by low level layer (SDIO) */
    uint8_t *data; /* Payload data (Control, data and event messages) */
    uint16_t len;  /* Frame buffer size */
};

#endif //PX4_AUTOPILOT_ESP32_DERIVER_H
