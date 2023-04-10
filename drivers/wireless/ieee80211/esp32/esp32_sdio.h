//
// Created by zhengweiqian on 9/4/2023.
//

#ifndef PX4_AUTOPILOT_ESP32_SDIO_H
#define PX4_AUTOPILOT_ESP32_SDIO_H

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <queue.h>

#include <nuttx/sdio.h>
#include <nuttx/semaphore.h>



#define HEADER_SIZE        0x12 /* Default sdpcm + bdc header size */

/* TODO move to Kconfig */

#define BCMF_PKT_POOL_SIZE 4    /* Frame pool size */

/* SDIO chip configuration structure */

struct bcmf_sdio_chip
{
    uint32_t ram_size;
    uint32_t core_base[MAX_CORE_ID];

    /* In-memory file images */

    FAR uint8_t *nvram_image;
    FAR unsigned int *nvram_image_size;
};

/* SDIO bus structure extension */

struct esp32_sdio_dev_s
{
    struct esp32_bus_dev_s bus;       /* Default esp32 bus structure */
    FAR struct sdio_dev_s *sdio_dev; /* The SDIO device bound to this instance */
    int minor;                       /* Device minor number */

    int  cur_chip_id;                /* Chip ID read from the card */
    struct esp32_sdio_chip *chip;     /* Chip specific configuration */

    volatile bool ready;             /* Current device status */
    bool sleeping;                   /* Current sleep status */

    int thread_id;                   /* Processing thread id */
    sem_t thread_signal;             /* Semaphore for processing thread event */
    struct wdog_s waitdog;           /* Processing thread waitdog */

    uint32_t backplane_current_addr; /* Current function 1 backplane base addr */

    volatile bool irq_pending;       /* True if interrupt is pending */
    uint32_t intstatus;              /* Copy of device current interrupt status */

    uint8_t max_seq;                 /* Maximum transmit sequence allowed */
    uint8_t tx_seq;                  /* Transmit sequence number (next) */
    uint8_t rx_seq;                  /* Receive sequence number (expected) */

    sem_t queue_mutex;               /* Lock for TX/RX/free queues */
    dq_queue_t free_queue;           /* Queue of available frames */
    dq_queue_t tx_queue;             /* Queue of frames to transmit */
    dq_queue_t rx_queue;             /* Queue of frames used to receive */
    volatile int tx_queue_count;     /* Count of items in TX queue */
};

/* Structure used to manage SDIO frames */

struct esp32_sdio_frame
{
    struct esp32_frame_s header;
    bool                tx;
    dq_entry_t          list_entry;
    uint8_t             data[HEADER_SIZE + MAX_NETDEV_PKTSIZE +
                             CONFIG_NET_GUARDSIZE];
};


int esp32_bus_sdio_initialize(FAR struct esp32_dev_s *priv,
                             int minor, FAR struct sdio_dev_s *dev);

/* FIXME: Low level bus data transfer function
 * To avoid bus error, len will be aligned to:
 * - upper power of 2 iflen is lesser than 64
 * - upper 64 bytes block if len is greater than 64
 */

int esp32_transfer_bytes(FAR struct esp32_sdio_dev_s *sbus, bool write,
                        uint8_t function, uint32_t address,
                        uint8_t *buf, unsigned int len);

int esp32_read_reg(FAR struct esp32_sdio_dev_s *sbus, uint8_t function,
                  uint32_t address, uint8_t *reg);

int esp32_write_reg(FAR struct esp32_sdio_dev_s *sbus, uint8_t function,
                   uint32_t address, uint8_t reg);

struct esp32_sdio_frame *esp32_sdio_allocate_frame(FAR struct esp32_dev_s *priv,
                                                 bool block, bool tx);

void esp32_sdio_free_frame(FAR struct esp32_dev_s *priv,
                          struct esp32_sdio_frame *sframe);


#endif //PX4_AUTOPILOT_ESP32_SDIO_H
