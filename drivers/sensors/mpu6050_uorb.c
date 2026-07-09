/****************************************************************************
 * drivers/sensors/mpu6050_uorb.c
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

#include <debug.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/sensors/mpu6050.h>
#include <nuttx/sensors/sensor.h>

#ifdef CONFIG_SENSORS_MPU6050

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register addresses */

#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1a
#define MPU6050_GYRO_CONFIG     0x1b
#define MPU6050_ACCEL_CONFIG    0x1c
#define MPU6050_INT_ENABLE      0x38
#define MPU6050_INT_STATUS      0x3a
#define MPU6050_ACCEL_XOUT_H    0x3b
#define MPU6050_ACCEL_XOUT_L    0x3c
#define MPU6050_ACCEL_YOUT_H    0x3d
#define MPU6050_ACCEL_YOUT_L    0x3e
#define MPU6050_ACCEL_ZOUT_H    0x3f
#define MPU6050_ACCEL_ZOUT_L    0x40
#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_TEMP_OUT_L      0x42
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_XOUT_L     0x44
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_YOUT_L     0x46
#define MPU6050_GYRO_ZOUT_H     0x47
#define MPU6050_GYRO_ZOUT_L     0x48
#define MPU6050_PWR_MGMT_1      0x6b
#define MPU6050_WHO_AM_I        0x75

/* Full Scale Range Options */

#define MPU6050_ACCEL_FS_2G     0
#define MPU6050_ACCEL_FS_4G     1
#define MPU6050_ACCEL_FS_8G     2
#define MPU6050_ACCEL_FS_16G    3

#define MPU6050_GYRO_FS_250DPS  0
#define MPU6050_GYRO_FS_500DPS  1
#define MPU6050_GYRO_FS_1000DPS 2
#define MPU6050_GYRO_FS_2000DPS 3

#define MPU6050_I2C_FREQUENCY   400000 /* 400 kHz */
#define CONSTANTS_ONE_G         9.80665f

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mpu6050_dev_s
{
  FAR struct i2c_master_s *i2c;
  uint8_t addr;
  int freq;
  mutex_t dev_lock;
};

struct mpu6050_sensor_s
{
  struct sensor_lowerhalf_s lower;
  float scale;
  FAR struct mpu6050_dev_s *dev;
  bool enabled;
};

struct mpu6050_uorb_dev_s
{
  struct mpu6050_dev_s base;
  struct mpu6050_sensor_s accel;
  struct mpu6050_sensor_s gyro;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int mpu6050_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable);
static int mpu6050_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep,
                         FAR char *buffer, size_t buflen);
static int mpu6050_control(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_mpu6050_ops =
{
  .activate = mpu6050_activate,
  .fetch    = mpu6050_fetch,
  .control  = mpu6050_control,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * Name: mpu6050_write_reg
 */

static int mpu6050_write_reg(FAR struct mpu6050_dev_s *priv,
                             uint8_t regaddr, uint8_t value)
{
  struct i2c_msg_s msg;
  uint8_t buffer[2];
  int ret;

  buffer[0] = regaddr;
  buffer[1] = value;

  msg.frequency = priv->freq;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = buffer;
  msg.length    = 2;

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret < 0) ? ret : OK;
}

/**
 * Name: mpu6050_read_reg
 */

static int mpu6050_read_reg(FAR struct mpu6050_dev_s *priv,
                            uint8_t regaddr, FAR uint8_t *value)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = value;
  msg[1].length    = 1;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  return (ret < 0) ? ret : OK;
}

/**
 * Name: mpu6050_read_regs
 */

static int mpu6050_read_regs(FAR struct mpu6050_dev_s *priv,
                             uint8_t regaddr, FAR uint8_t *buffer,
                             uint8_t len)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buffer;
  msg[1].length    = len;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  return (ret < 0) ? ret : OK;
}

/**
 * Name: mpu6050_activate
 */

static int mpu6050_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable)
{
  FAR struct mpu6050_sensor_s *priv = (FAR struct mpu6050_sensor_s *)lower;
  FAR struct mpu6050_dev_s *dev = priv->dev;
  int ret;

  ret = nxmutex_lock(&dev->dev_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (enable)
    {
      ret = mpu6050_write_reg(dev, MPU6050_PWR_MGMT_1, 0x00);
    }
  else
    {
      ret = mpu6050_write_reg(dev, MPU6050_PWR_MGMT_1, 0x40);
    }

  if (ret >= 0)
    {
      priv->enabled = enable;
    }

  nxmutex_unlock(&dev->dev_lock);
  return ret;
}

/**
 * Name: mpu6050_fetch
 */

static int mpu6050_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep,
                         FAR char *buffer, size_t buflen)
{
  FAR struct mpu6050_sensor_s *priv = (FAR struct mpu6050_sensor_s *)lower;
  FAR struct mpu6050_dev_s *dev = priv->dev;
  uint8_t buf[14];
  float temp_c;
  int ret;

  if (buffer == NULL)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&dev->dev_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = mpu6050_read_regs(dev, MPU6050_ACCEL_XOUT_H, buf, 14);
  nxmutex_unlock(&dev->dev_lock);

  if (ret < 0)
    {
      return ret;
    }

  temp_c = ((float)(int16_t)((buf[6] << 8) | buf[7]) / 340.0f) + 36.53f;

  if (lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      struct sensor_accel accel;

      if (buflen < sizeof(struct sensor_accel))
        {
          return -EINVAL;
        }

      accel.timestamp   = sensor_get_timestamp();
      accel.x           = (float)(int16_t)((buf[0] << 8) | buf[1]) *
                          priv->scale;
      accel.y           = (float)(int16_t)((buf[2] << 8) | buf[3]) *
                          priv->scale;
      accel.z           = (float)(int16_t)((buf[4] << 8) | buf[5]) *
                          priv->scale;
      accel.temperature = temp_c;

      memcpy(buffer, &accel, sizeof(struct sensor_accel));
      return sizeof(struct sensor_accel);
    }
  else if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      struct sensor_gyro gyro;

      if (buflen < sizeof(struct sensor_gyro))
        {
          return -EINVAL;
        }

      gyro.timestamp   = sensor_get_timestamp();
      gyro.x           = (float)(int16_t)((buf[8] << 8) | buf[9]) *
                         priv->scale;
      gyro.y           = (float)(int16_t)((buf[10] << 8) | buf[11]) *
                         priv->scale;
      gyro.z           = (float)(int16_t)((buf[12] << 8) | buf[13]) *
                         priv->scale;
      gyro.temperature = temp_c;

      memcpy(buffer, &gyro, sizeof(struct sensor_gyro));
      return sizeof(struct sensor_gyro);
    }

  return -EINVAL;
}

/**
 * Name: mpu6050_control
 */

static int mpu6050_control(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           int cmd, unsigned long arg)
{
  FAR struct mpu6050_sensor_s *priv = (FAR struct mpu6050_sensor_s *)lower;
  FAR struct mpu6050_dev_s *dev = priv->dev;
  int ret;

  ret = nxmutex_lock(&dev->dev_lock);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
      case SNIOC_SET_SCALE_XL:
        {
          if (lower->type == SENSOR_TYPE_ACCELEROMETER)
            {
              uint8_t fs = (uint8_t)arg;
              ret = mpu6050_write_reg(dev, MPU6050_ACCEL_CONFIG, fs << 3);
              if (ret >= 0)
                {
                  switch (fs)
                    {
                      case MPU6050_ACCEL_FS_2G:
                        priv->scale = CONSTANTS_ONE_G / 16384.0f;
                        break;
                      case MPU6050_ACCEL_FS_4G:
                        priv->scale = CONSTANTS_ONE_G / 8192.0f;
                        break;
                      case MPU6050_ACCEL_FS_8G:
                        priv->scale = CONSTANTS_ONE_G / 4096.0f;
                        break;
                      case MPU6050_ACCEL_FS_16G:
                        priv->scale = CONSTANTS_ONE_G / 2048.0f;
                        break;
                      default:
                        break;
                    }
                }
            }
          else if (lower->type == SENSOR_TYPE_GYROSCOPE)
            {
              uint8_t fs = (uint8_t)arg;
              ret = mpu6050_write_reg(dev, MPU6050_GYRO_CONFIG, fs << 3);
              if (ret >= 0)
                {
                  switch (fs)
                    {
                      case MPU6050_GYRO_FS_250DPS:
                        priv->scale = (M_PI / 180.0f) / 131.0f;
                        break;
                      case MPU6050_GYRO_FS_500DPS:
                        priv->scale = (M_PI / 180.0f) / 65.5f;
                        break;
                      case MPU6050_GYRO_FS_1000DPS:
                        priv->scale = (M_PI / 180.0f) / 32.8f;
                        break;
                      case MPU6050_GYRO_FS_2000DPS:
                        priv->scale = (M_PI / 180.0f) / 16.4f;
                        break;
                      default:
                        break;
                    }
                }
            }
          else
            {
              ret = -EINVAL;
            }
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&dev->dev_lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * Name: mpu6050_register
 */

int mpu6050_register(int devno, FAR struct i2c_master_s *i2c, uint8_t addr)
{
  FAR struct mpu6050_uorb_dev_s *dev;
  FAR struct mpu6050_sensor_s *sensor;
  FAR struct sensor_lowerhalf_s *lower;
  uint8_t whoami;
  int ret;

  DEBUGASSERT(i2c != NULL);

  dev = (FAR struct mpu6050_uorb_dev_s *)
    kmm_zalloc(sizeof(struct mpu6050_uorb_dev_s));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  dev->base.i2c  = i2c;
  dev->base.addr = addr;
  dev->base.freq = MPU6050_I2C_FREQUENCY;

  nxmutex_init(&dev->base.dev_lock);

  /* Verify MPU6050 is present */

  ret = mpu6050_read_reg(&dev->base, MPU6050_WHO_AM_I, &whoami);
  if (ret < 0 || whoami != 0x68)
    {
      syslog(LOG_ERR, "MPU6050: WHO_AM_I = 0x%02x (expected 0x68)\n",
             whoami);
      nxmutex_destroy(&dev->base.dev_lock);
      kmm_free(dev);
      return -ENODEV;
    }

  syslog(LOG_INFO, "MPU6050: Found at 0x%02x\n", addr);

  /* Initialize MPU6050 */

  mpu6050_write_reg(&dev->base, MPU6050_PWR_MGMT_1, 0x00);   /* Wake up */
  mpu6050_write_reg(&dev->base, MPU6050_SMPLRT_DIV, 9);      /* 100 Hz */
  mpu6050_write_reg(&dev->base, MPU6050_ACCEL_CONFIG, 0x00); /* ±2g */
  mpu6050_write_reg(&dev->base, MPU6050_GYRO_CONFIG, 0x00);  /* ±250°/s */

  /* Register Accelerometer */

  sensor          = &dev->accel;
  lower           = &sensor->lower;

  sensor->dev     = &dev->base;
  sensor->enabled = false;
  sensor->scale   = CONSTANTS_ONE_G / 16384.0f;
  lower->type     = SENSOR_TYPE_ACCELEROMETER;
  lower->nbuffer  = 1;
  lower->ops      = &g_mpu6050_ops;

  ret = sensor_register(lower, devno);
  if (ret < 0)
    {
      syslog(LOG_ERR, "MPU6050: Failed to register accel: %d\n", ret);
      goto errout;
    }

  /* Register Gyroscope */

  sensor          = &dev->gyro;
  lower           = &sensor->lower;
  sensor->dev     = &dev->base;
  sensor->enabled = false;
  sensor->scale   = (M_PI / 180.0f) / 131.0f;
  lower->type     = SENSOR_TYPE_GYROSCOPE;
  lower->nbuffer  = 1;
  lower->ops      = &g_mpu6050_ops;

  ret = sensor_register(lower, devno);
  if (ret < 0)
    {
      syslog(LOG_ERR, "MPU6050: Failed to register gyro: %d\n", ret);
      goto errout;
    }

  syslog(LOG_INFO, "MPU6050: uORB driver registered successfully\n");
  return OK;

errout:
  if (dev->accel.lower.type != 0)
    {
      sensor_unregister(&dev->accel.lower, devno);
    }

  if (dev->gyro.lower.type != 0)
    {
      sensor_unregister(&dev->gyro.lower, devno);
    }

  nxmutex_destroy(&dev->base.dev_lock);
  kmm_free(dev);
  return ret;
}

#endif /* CONFIG_SENSORS_MPU6050 */
