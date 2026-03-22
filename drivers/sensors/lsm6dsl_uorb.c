/****************************************************************************
 * drivers/sensors/lsm6dsl_uorb.c
 *
 * LSM6DSL IMU uORB driver — accelerometer + gyroscope.
 * Supports interrupt-driven (DRDY) and kthread polling modes.
 * Based on lsm6dso32_uorb.c by Carleton University InSpace.
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
#include <nuttx/nuttx.h>

#include <debug.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/lsm6dsl_uorb.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/signal.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* WHO_AM_I value for LSM6DSL */

#define WHO_AM_I_VAL 0x6a

/* Convert milli-g to m/s^2 */

#define MILLIG_TO_MS2 (0.0098067f)

/* Convert milli-dps to rad/s */

#define MDPS_TO_RADS (3.141592653f / (180.0f * 1000.0f))

/* Registers (LSM6DSL) */

#define WHO_AM_I    0x0f
#define CTRL1_XL    0x10   /* Accel control reg */
#define CTRL2_G     0x11   /* Gyro control reg */
#define CTRL3_C     0x12   /* Control reg 3 */
#define CTRL4_C     0x13   /* Control reg 4 */
#define CTRL5_C     0x14   /* Control reg 5 */
#define CTRL6_C     0x15   /* Control reg 6 */
#define CTRL7_G     0x16   /* Control reg 7 */
#define CTRL8_XL    0x17   /* Control reg 8 */
#define CTRL9_XL    0x18   /* Control reg 9 */
#define CTRL10_C    0x19   /* Control reg 10 */
#define INT1_CTRL   0x0d   /* INT1 pin control */
#define INT2_CTRL   0x0e   /* INT2 pin control */
#define STATUS_REG  0x1e   /* Status register */
#define OUT_TEMP_L  0x20   /* Temperature low byte */
#define OUT_TEMP_H  0x21   /* Temperature high byte */
#define OUTX_L_G    0x22   /* Gyro X low byte */
#define OUTX_H_G    0x23
#define OUTY_L_G    0x24
#define OUTY_H_G    0x25
#define OUTZ_L_G    0x26
#define OUTZ_H_G    0x27
#define OUTX_L_A    0x28   /* Accel X low byte */
#define OUTX_H_A    0x29
#define OUTY_L_A    0x2a
#define OUTY_H_A    0x2b
#define OUTZ_L_A    0x2c
#define OUTZ_H_A    0x2d

/* Status bits */

#define BIT_STATUS_XLDA (1 << 0)  /* Accel data ready */
#define BIT_STATUS_GDA  (1 << 1)  /* Gyro data ready */
#define BIT_STATUS_TDA  (1 << 2)  /* Temp data ready */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* ODR settings (same encoding as LSM6DSO32) */

enum lsm6dsl_odr_e
{
  ODR_OFF     = 0x0,
  ODR_12_5HZ  = 0x1,
  ODR_26HZ    = 0x2,
  ODR_52HZ    = 0x3,
  ODR_104HZ   = 0x4,
  ODR_208HZ   = 0x5,
  ODR_416HZ   = 0x6,
  ODR_833HZ   = 0x7,
  ODR_1660HZ  = 0x8,
  ODR_3330HZ  = 0x9,
  ODR_6660HZ  = 0xa,
};

/* Gyroscope FSR settings */

enum lsm6dsl_fsr_gy_e
{
  FSR_GY_250DPS  = 0x0,  /* +-250 dps */
  FSR_GY_125DPS  = 0x1,  /* +-125 dps */
  FSR_GY_500DPS  = 0x2,  /* +-500 dps */
  FSR_GY_1000DPS = 0x4,  /* +-1000 dps */
  FSR_GY_2000DPS = 0x6,  /* +-2000 dps */
};

/* Accelerometer FSR settings (LSM6DSL: 2/4/8/16g, differs from LSM6DSO32) */

enum lsm6dsl_fsr_xl_e
{
  FSR_XL_2G  = 0x0,  /* +-2g */
  FSR_XL_16G = 0x1,  /* +-16g */
  FSR_XL_4G  = 0x2,  /* +-4g */
  FSR_XL_8G  = 0x3,  /* +-8g */
};

/* Lower-half sensor instance */

struct lsm6dsl_sens_s
{
  struct sensor_lowerhalf_s lower;
  FAR struct lsm6dsl_dev_s *dev;
  bool enabled;
  enum lsm6dsl_odr_e odr;
  int fsr;
  sem_t run;
  enum lsm6dsl_int_e intpin;
  bool interrupts;
  struct work_s work;
};

/* Device instance */

struct lsm6dsl_dev_s
{
  struct lsm6dsl_sens_s gyro;
  struct lsm6dsl_sens_s accel;
  FAR struct i2c_master_s *i2c;
  uint8_t addr;
  mutex_t devlock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int lsm6dsl_activate(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, bool enable);
static int lsm6dsl_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR uint32_t *period_us);
static int lsm6dsl_control(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, int cmd,
                            unsigned long arg);
static int lsm6dsl_get_info(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             FAR struct sensor_device_info_s *info);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ODR to microsecond intervals */

static const uint32_t g_odr_interval[] =
{
  0,       /* ODR_OFF */
  80000,   /* ODR_12_5HZ */
  38462,   /* ODR_26HZ */
  19230,   /* ODR_52HZ */
  9615,    /* ODR_104HZ */
  4807,    /* ODR_208HZ */
  2403,    /* ODR_416HZ */
  1200,    /* ODR_833HZ */
  602,     /* ODR_1660HZ */
  300,     /* ODR_3330HZ */
  150,     /* ODR_6660HZ */
};

/* Accel FSR sensitivities in m/s^2 per LSB (LSM6DSL datasheet Table 2) */

static const float g_fsr_xl_sens[] =
{
  0.061f * MILLIG_TO_MS2,  /* 2g */
  0.488f * MILLIG_TO_MS2,  /* 16g */
  0.122f * MILLIG_TO_MS2,  /* 4g */
  0.244f * MILLIG_TO_MS2,  /* 8g */
};

/* Gyro FSR sensitivities in rad/s per LSB (LSM6DSL datasheet Table 3) */

static const float g_fsr_gy_sens[] =
{
  8.75f * MDPS_TO_RADS,    /* 250 dps */
  4.375f * MDPS_TO_RADS,   /* 125 dps */
  17.50f * MDPS_TO_RADS,   /* 500 dps */
  0.0f,                    /* unused (3) */
  35.0f * MDPS_TO_RADS,    /* 1000 dps */
  0.0f,                    /* unused (5) */
  70.0f * MDPS_TO_RADS,    /* 2000 dps */
};

/* Interrupt control registers */

static const uint8_t g_int_ctrl[] =
{
  INT1_CTRL,  /* INT1 */
  INT2_CTRL,  /* INT2 */
};

/* Sensor operations */

static const struct sensor_ops_s g_sensor_ops =
{
  .fetch          = NULL,
  .activate       = lsm6dsl_activate,
  .control        = lsm6dsl_control,
  .set_interval   = lsm6dsl_set_interval,
  .selftest       = NULL,
  .set_calibvalue = NULL,
  .calibrate      = NULL,
  .get_info       = lsm6dsl_get_info,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm6dsl_write_bytes
 ****************************************************************************/

static int lsm6dsl_write_bytes(FAR struct lsm6dsl_dev_s *priv,
                                uint8_t addr, FAR void *buf, size_t nbytes)
{
  struct i2c_msg_s cmd[2];

  cmd[0].frequency = CONFIG_LSM6DSL_UORB_I2C_FREQUENCY;
  cmd[0].addr      = priv->addr;
  cmd[0].flags     = I2C_M_NOSTOP;
  cmd[0].buffer    = &addr;
  cmd[0].length    = sizeof(addr);

  cmd[1].frequency = CONFIG_LSM6DSL_UORB_I2C_FREQUENCY;
  cmd[1].addr      = priv->addr;
  cmd[1].flags     = I2C_M_NOSTART;
  cmd[1].buffer    = buf;
  cmd[1].length    = nbytes;

  return I2C_TRANSFER(priv->i2c, cmd, 2);
}

/****************************************************************************
 * Name: lsm6dsl_read_bytes
 ****************************************************************************/

static int lsm6dsl_read_bytes(FAR struct lsm6dsl_dev_s *priv,
                               uint8_t addr, FAR void *buf, size_t nbytes)
{
  struct i2c_msg_s cmd[2];

  cmd[0].frequency = CONFIG_LSM6DSL_UORB_I2C_FREQUENCY;
  cmd[0].addr      = priv->addr;
  cmd[0].flags     = I2C_M_NOSTOP;
  cmd[0].buffer    = &addr;
  cmd[0].length    = sizeof(addr);

  cmd[1].frequency = CONFIG_LSM6DSL_UORB_I2C_FREQUENCY;
  cmd[1].addr      = priv->addr;
  cmd[1].flags     = I2C_M_READ;
  cmd[1].buffer    = buf;
  cmd[1].length    = nbytes;

  return I2C_TRANSFER(priv->i2c, cmd, 2);
}

/****************************************************************************
 * Name: lsm6dsl_set_bits
 ****************************************************************************/

static int lsm6dsl_set_bits(FAR struct lsm6dsl_dev_s *priv, uint8_t addr,
                             uint8_t set_bits, uint8_t clear_bits)
{
  int err;
  uint8_t reg;

  err = lsm6dsl_read_bytes(priv, addr, &reg, sizeof(reg));
  if (err < 0)
    {
      return err;
    }

  reg = (reg & ~clear_bits) | set_bits;
  return lsm6dsl_write_bytes(priv, addr, &reg, sizeof(reg));
}

/****************************************************************************
 * Name: accel_set_odr
 ****************************************************************************/

static int accel_set_odr(FAR struct lsm6dsl_dev_s *dev,
                          enum lsm6dsl_odr_e odr)
{
  int err;

  err = lsm6dsl_set_bits(dev, CTRL1_XL, (odr & 0xf) << 4, 0xf0);
  if (err < 0)
    {
      return err;
    }

  dev->accel.odr = odr;
  return err;
}

/****************************************************************************
 * Name: gyro_set_odr
 ****************************************************************************/

static int gyro_set_odr(FAR struct lsm6dsl_dev_s *dev,
                         enum lsm6dsl_odr_e odr)
{
  int err;

  err = lsm6dsl_set_bits(dev, CTRL2_G, (odr & 0x0f) << 4, 0xf0);
  if (err < 0)
    {
      return err;
    }

  dev->gyro.odr = odr;
  return err;
}

/****************************************************************************
 * Name: accel_set_fsr
 ****************************************************************************/

static int accel_set_fsr(FAR struct lsm6dsl_dev_s *dev,
                          enum lsm6dsl_fsr_xl_e fsr)
{
  int err;

  err = lsm6dsl_set_bits(dev, CTRL1_XL, (fsr & 0x3) << 2, 0x0c);
  if (err < 0)
    {
      return err;
    }

  dev->accel.fsr = fsr;
  return err;
}

/****************************************************************************
 * Name: gyro_set_fsr
 ****************************************************************************/

static int gyro_set_fsr(FAR struct lsm6dsl_dev_s *dev,
                         enum lsm6dsl_fsr_gy_e fsr)
{
  int err;

  err = lsm6dsl_set_bits(dev, CTRL2_G, (fsr & 0x7) << 1, 0x0e);
  if (err < 0)
    {
      return err;
    }

  dev->gyro.fsr = fsr;
  return err;
}

/****************************************************************************
 * Name: gyro_int_enable / accel_int_enable
 ****************************************************************************/

static int gyro_int_enable(FAR struct lsm6dsl_dev_s *dev, bool enable)
{
  int err;
  uint8_t enable_bits  = enable ? 0x02 : 0x00;  /* DRDY_G bit */
  uint8_t disable_bits = enable ? 0x00 : 0x02;

  err = lsm6dsl_set_bits(dev, g_int_ctrl[dev->gyro.intpin],
                          enable_bits, disable_bits);
  if (err < 0)
    {
      return err;
    }

  dev->gyro.interrupts = enable;
  return err;
}

static int accel_int_enable(FAR struct lsm6dsl_dev_s *dev, bool enable)
{
  int err;
  uint8_t enable_bits  = enable ? 0x01 : 0x00;  /* DRDY_XL bit */
  uint8_t disable_bits = enable ? 0x00 : 0x01;

  err = lsm6dsl_set_bits(dev, g_int_ctrl[dev->accel.intpin],
                          enable_bits, disable_bits);
  if (err < 0)
    {
      return err;
    }

  dev->accel.interrupts = enable;
  return err;
}

/****************************************************************************
 * Name: lsm6dsl_convert_temp
 ****************************************************************************/

static float lsm6dsl_convert_temp(int16_t temp)
{
  return (float)((temp / 256) + 25);
}

/****************************************************************************
 * Name: lsm6dsl_read_gyro
 ****************************************************************************/

static int lsm6dsl_read_gyro(FAR struct lsm6dsl_dev_s *dev,
                              FAR struct sensor_gyro *data)
{
  int16_t raw[4];  /* temp + 3x gyro */
  int err;

  err = lsm6dsl_read_bytes(dev, OUT_TEMP_L, raw, sizeof(raw));
  if (err < 0)
    {
      return err;
    }

  data->timestamp   = sensor_get_timestamp();
  data->temperature = lsm6dsl_convert_temp(raw[0]);
  data->x = (float)(raw[1]) * g_fsr_gy_sens[dev->gyro.fsr];
  data->y = (float)(raw[2]) * g_fsr_gy_sens[dev->gyro.fsr];
  data->z = (float)(raw[3]) * g_fsr_gy_sens[dev->gyro.fsr];

  return err;
}

/****************************************************************************
 * Name: lsm6dsl_read_accel
 ****************************************************************************/

static int lsm6dsl_read_accel(FAR struct lsm6dsl_dev_s *dev,
                               FAR struct sensor_accel *data)
{
  int16_t raw[3];
  int16_t raw_temp;
  int err;

  err = lsm6dsl_read_bytes(dev, OUTX_L_A, raw, sizeof(raw));
  if (err < 0)
    {
      return err;
    }

  err = lsm6dsl_read_bytes(dev, OUT_TEMP_L, &raw_temp, sizeof(raw_temp));
  if (err < 0)
    {
      return err;
    }

  data->timestamp   = sensor_get_timestamp();
  data->temperature = lsm6dsl_convert_temp(raw_temp);
  data->x = (float)(raw[0]) * g_fsr_xl_sens[dev->accel.fsr];
  data->y = (float)(raw[1]) * g_fsr_xl_sens[dev->accel.fsr];
  data->z = (float)(raw[2]) * g_fsr_xl_sens[dev->accel.fsr];

  return err;
}

/****************************************************************************
 * Name: push_gyro / push_accel
 ****************************************************************************/

static int push_gyro(FAR struct lsm6dsl_dev_s *dev)
{
  int err;
  struct sensor_gyro data;

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  err = lsm6dsl_read_gyro(dev, &data);
  if (err < 0)
    {
      goto early_ret;
    }

  dev->gyro.lower.push_event(dev->gyro.lower.priv, &data, sizeof(data));

early_ret:
  nxmutex_unlock(&dev->devlock);
  return err;
}

static int push_accel(FAR struct lsm6dsl_dev_s *dev)
{
  int err;
  struct sensor_accel data;

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  err = lsm6dsl_read_accel(dev, &data);
  if (err < 0)
    {
      goto early_ret;
    }

  dev->accel.lower.push_event(dev->accel.lower.priv, &data, sizeof(data));

early_ret:
  nxmutex_unlock(&dev->devlock);
  return err;
}

/****************************************************************************
 * Name: gyro_worker / accel_worker (interrupt work queue handlers)
 ****************************************************************************/

static void gyro_worker(FAR void *arg)
{
  push_gyro(arg);
}

static void accel_worker(FAR void *arg)
{
  push_accel(arg);
}

/****************************************************************************
 * Name: gyro_int_handler / accel_int_handler
 ****************************************************************************/

static int gyro_int_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct lsm6dsl_dev_s *dev = (FAR struct lsm6dsl_dev_s *)(arg);
  int err;

  DEBUGASSERT(arg != NULL);

  err = work_queue(HPWORK, &dev->gyro.work, &gyro_worker, dev, 0);
  if (err < 0)
    {
      snerr("Could not queue LSM6DSL gyro work: %d\n", err);
    }

  return err;
}

static int accel_int_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct lsm6dsl_dev_s *dev = (FAR struct lsm6dsl_dev_s *)(arg);
  int err;

  DEBUGASSERT(arg != NULL);

  err = work_queue(HPWORK, &dev->accel.work, &accel_worker, dev, 0);
  if (err < 0)
    {
      snerr("Could not queue LSM6DSL accel work: %d\n", err);
    }

  return err;
}

/****************************************************************************
 * Name: gyro_thread / accel_thread (polling kthreads)
 ****************************************************************************/

static int gyro_thread(int argc, char **argv)
{
  FAR struct lsm6dsl_dev_s *dev =
      (FAR struct lsm6dsl_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
  int err = 0;

  while (true)
    {
      if (!dev->gyro.enabled)
        {
          err = nxsem_wait(&dev->gyro.run);
          if (err < 0)
            {
              continue;
            }
        }

      err = push_gyro(dev);
      if (err < 0)
        {
          continue;
        }

      nxsched_usleep(g_odr_interval[dev->gyro.odr]);
    }

  return err;
}

static int accel_thread(int argc, char **argv)
{
  FAR struct lsm6dsl_dev_s *dev =
      (FAR struct lsm6dsl_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
  int err = 0;

  while (true)
    {
      if (!dev->accel.enabled)
        {
          err = nxsem_wait(&dev->accel.run);
          if (err < 0)
            {
              continue;
            }
        }

      err = push_accel(dev);
      if (err < 0)
        {
          continue;
        }

      nxsched_usleep(g_odr_interval[dev->accel.odr]);
    }

  return err;
}

/****************************************************************************
 * Name: lsm6dsl_activate
 ****************************************************************************/

static int lsm6dsl_activate(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, bool enable)
{
  FAR struct lsm6dsl_sens_s *sens =
      container_of(lower, FAR struct lsm6dsl_sens_s, lower);
  FAR struct lsm6dsl_dev_s *dev = sens->dev;
  bool start_thread = false;
  int err;

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  if (enable && !sens->enabled)
    {
      start_thread = true;

      if (lower->type == SENSOR_TYPE_GYROSCOPE)
        {
          err = gyro_set_odr(dev, ODR_12_5HZ);
        }
      else
        {
          err = accel_set_odr(dev, ODR_12_5HZ);
        }

      if (err < 0)
        {
          goto early_ret;
        }
    }

  if (!enable && sens->enabled)
    {
      if (lower->type == SENSOR_TYPE_GYROSCOPE)
        {
          err = gyro_set_odr(dev, ODR_OFF);
        }
      else
        {
          err = accel_set_odr(dev, ODR_OFF);
        }

      if (err < 0)
        {
          goto early_ret;
        }
    }

  sens->enabled = enable;

  if (start_thread)
    {
      err = nxsem_post(&sens->run);
    }

early_ret:
  nxmutex_unlock(&dev->devlock);
  return err;
}

/****************************************************************************
 * Name: lsm6dsl_set_interval
 ****************************************************************************/

static int lsm6dsl_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR uint32_t *period_us)
{
  FAR struct lsm6dsl_sens_s *sens =
      container_of(lower, FAR struct lsm6dsl_sens_s, lower);
  FAR struct lsm6dsl_dev_s *dev = sens->dev;
  int err;
  enum lsm6dsl_odr_e odr;

  if (*period_us >= 80000)
    {
      odr = ODR_12_5HZ;
    }
  else if (*period_us >= 38462)
    {
      odr = ODR_26HZ;
    }
  else if (*period_us >= 19231)
    {
      odr = ODR_52HZ;
    }
  else if (*period_us >= 9615)
    {
      odr = ODR_104HZ;
    }
  else if (*period_us >= 4808)
    {
      odr = ODR_208HZ;
    }
  else if (*period_us >= 2404)
    {
      odr = ODR_416HZ;
    }
  else if (*period_us >= 1200)
    {
      odr = ODR_833HZ;
    }
  else if (*period_us >= 602)
    {
      odr = ODR_1660HZ;
    }
  else if (*period_us >= 300)
    {
      odr = ODR_3330HZ;
    }
  else
    {
      odr = ODR_6660HZ;
    }

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  if (lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      err = accel_set_odr(dev, odr);
    }
  else
    {
      err = gyro_set_odr(dev, odr);
    }

  if (err >= 0)
    {
      *period_us = g_odr_interval[odr];
    }

  nxmutex_unlock(&dev->devlock);
  return err;
}

/****************************************************************************
 * Name: lsm6dsl_get_info
 ****************************************************************************/

static int lsm6dsl_get_info(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             FAR struct sensor_device_info_s *info)
{
  FAR struct lsm6dsl_sens_s *sens =
      container_of(lower, FAR struct lsm6dsl_sens_s, lower);

  memset(info, 0, sizeof(struct sensor_device_info_s));
  info->version = 0;
  info->power   = 0.55f;  /* 0.55 mA in high performance */
  memcpy(info->name, "LSM6DSL", sizeof("LSM6DSL"));
  memcpy(info->vendor, "STMicro", sizeof("STMicro"));

  if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      info->resolution = g_fsr_gy_sens[sens->fsr];
      info->max_range  = g_fsr_gy_sens[sens->fsr] * INT16_MAX;
      info->min_delay  = (int32_t)g_odr_interval[ODR_6660HZ];
      info->max_delay  = (int32_t)g_odr_interval[ODR_12_5HZ];
    }
  else
    {
      info->resolution = g_fsr_xl_sens[sens->fsr];
      info->max_range  = g_fsr_xl_sens[sens->fsr] * INT16_MAX;
      info->min_delay  = (int32_t)g_odr_interval[ODR_6660HZ];
      info->max_delay  = (int32_t)g_odr_interval[ODR_12_5HZ];
    }

  return 0;
}

/****************************************************************************
 * Name: lsm6dsl_control
 ****************************************************************************/

static int lsm6dsl_control(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, int cmd,
                            unsigned long arg)
{
  FAR struct lsm6dsl_sens_s *sens =
      container_of(lower, FAR struct lsm6dsl_sens_s, lower);
  FAR struct lsm6dsl_dev_s *dev = sens->dev;
  int err;

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  switch (cmd)
    {
    case SNIOC_WHO_AM_I:
      {
        FAR uint8_t *id = (FAR uint8_t *)(arg);
        if (id == NULL)
          {
            err = -EINVAL;
            break;
          }

        err = lsm6dsl_read_bytes(dev, WHO_AM_I, id, sizeof(uint8_t));
      }
      break;

    case SNIOC_SETFULLSCALE:
      {
        if (lower->type == SENSOR_TYPE_ACCELEROMETER)
          {
            switch (arg)
              {
              case 2:
                err = accel_set_fsr(dev, FSR_XL_2G);
                break;
              case 4:
                err = accel_set_fsr(dev, FSR_XL_4G);
                break;
              case 8:
                err = accel_set_fsr(dev, FSR_XL_8G);
                break;
              case 16:
                err = accel_set_fsr(dev, FSR_XL_16G);
                break;
              default:
                err = -EINVAL;
                break;
              }
          }
        else
          {
            switch (arg)
              {
              case 125:
                err = gyro_set_fsr(dev, FSR_GY_125DPS);
                break;
              case 250:
                err = gyro_set_fsr(dev, FSR_GY_250DPS);
                break;
              case 500:
                err = gyro_set_fsr(dev, FSR_GY_500DPS);
                break;
              case 1000:
                err = gyro_set_fsr(dev, FSR_GY_1000DPS);
                break;
              case 2000:
                err = gyro_set_fsr(dev, FSR_GY_2000DPS);
                break;
              default:
                err = -EINVAL;
                break;
              }
          }
      }
      break;

    default:
      err = -EINVAL;
      break;
    }

  nxmutex_unlock(&dev->devlock);
  return err;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm6dsl_register_uorb
 ****************************************************************************/

int lsm6dsl_register_uorb(FAR struct i2c_master_s *i2c, uint8_t addr,
                           uint8_t devno,
                           FAR struct lsm6dsl_uorb_config_s *config)
{
  FAR struct lsm6dsl_dev_s *priv;
  int err;
  FAR char *argv[2];
  char arg1[32];
  int gyro_pid;

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(addr == 0x6b || addr == 0x6a);

#if !defined(CONFIG_SCHED_HPWORK)
  if (config->gy_attach != NULL || config->xl_attach != NULL)
    {
      snerr("CONFIG_SCHED_HPWORK required for interrupt driven mode.\n");
      return -ENOSYS;
    }
#endif

  if (config->gy_attach != NULL && config->xl_attach != NULL &&
      (config->gy_int == config->xl_int))
    {
      snerr("Cannot use the same interrupt pin for accel and gyro.\n");
      return -EINVAL;
    }

  DEBUGASSERT(config->gy_int == LSM6DSL_INT1 ||
              config->gy_int == LSM6DSL_INT2);
  DEBUGASSERT(config->xl_int == LSM6DSL_INT1 ||
              config->xl_int == LSM6DSL_INT2);

  /* Allocate device */

  priv = kmm_zalloc(sizeof(struct lsm6dsl_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate LSM6DSL driver.\n");
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;

  err = nxmutex_init(&priv->devlock);
  if (err < 0)
    {
      goto free_mem;
    }

  err = nxsem_init(&priv->gyro.run, 0, 0);
  if (err < 0)
    {
      goto del_mutex;
    }

  err = nxsem_init(&priv->accel.run, 0, 0);
  if (err < 0)
    {
      goto del_gyro_sem;
    }

  /* Register gyro lower half */

  priv->gyro.lower.type    = SENSOR_TYPE_GYROSCOPE;
  priv->gyro.lower.ops     = &g_sensor_ops;
  priv->gyro.lower.nbuffer = CONFIG_LSM6DSL_UORB_GYRO_BUFSIZE;
  priv->gyro.enabled       = false;
  priv->gyro.odr           = ODR_OFF;
  priv->gyro.fsr           = FSR_GY_250DPS;
  priv->gyro.interrupts    = false;
  priv->gyro.intpin        = config->gy_int;
  priv->gyro.dev           = priv;

  err = sensor_register(&priv->gyro.lower, devno);
  if (err < 0)
    {
      snerr("Failed to register LSM6DSL gyro: %d\n", err);
      goto del_accel_sem;
    }

  /* Register accel lower half */

  priv->accel.lower.type    = SENSOR_TYPE_ACCELEROMETER;
  priv->accel.lower.ops     = &g_sensor_ops;
  priv->accel.lower.nbuffer = CONFIG_LSM6DSL_UORB_ACCEL_BUFSIZE;
  priv->accel.enabled       = false;
  priv->accel.odr           = ODR_OFF;
  priv->accel.fsr           = FSR_XL_2G;
  priv->accel.interrupts    = false;
  priv->accel.intpin        = config->xl_int;
  priv->accel.dev           = priv;

  err = sensor_register(&priv->accel.lower, devno);
  if (err < 0)
    {
      snerr("Failed to register LSM6DSL accel: %d\n", err);
      goto unreg_gyro;
    }

  /* Gyroscope data acquisition setup */

  if (config->gy_attach != NULL)
    {
      err = config->gy_attach(gyro_int_handler, priv);
      if (err < 0)
        {
          snerr("Failed to attach gyro interrupt: %d\n", err);
          goto unreg_accel;
        }

      err = gyro_int_enable(priv, true);
      if (err < 0)
        {
          goto unreg_accel;
        }

      sninfo("LSM6DSL gyro using interrupt on %s.\n",
             config->gy_int == LSM6DSL_INT1 ? "INT1" : "INT2");
    }
  else
    {
      snprintf(arg1, sizeof(arg1), "%p", priv);
      argv[0] = arg1;
      argv[1] = NULL;
      err = kthread_create("lsm6dsl_gy", SCHED_PRIORITY_DEFAULT,
                           CONFIG_LSM6DSL_UORB_THREAD_STACKSIZE,
                           gyro_thread, argv);
      if (err < 0)
        {
          snerr("Failed to create gyro thread: %d\n", err);
          goto unreg_accel;
        }

      gyro_pid = err;
      sninfo("LSM6DSL gyro using polling thread.\n");
    }

  /* Accelerometer data acquisition setup */

  if (config->xl_attach != NULL)
    {
      err = config->xl_attach(accel_int_handler, priv);
      if (err < 0)
        {
          snerr("Failed to attach accel interrupt: %d\n", err);
          goto unreg_gyro_handler;
        }

      err = accel_int_enable(priv, true);
      if (err < 0)
        {
          goto unreg_gyro_handler;
        }

      sninfo("LSM6DSL accel using interrupt on %s.\n",
             config->xl_int == LSM6DSL_INT1 ? "INT1" : "INT2");
    }
  else
    {
      snprintf(arg1, sizeof(arg1), "%p", priv);
      argv[0] = arg1;
      argv[1] = NULL;
      err = kthread_create("lsm6dsl_xl", SCHED_PRIORITY_DEFAULT,
                           CONFIG_LSM6DSL_UORB_THREAD_STACKSIZE,
                           accel_thread, argv);
      if (err < 0)
        {
          snerr("Failed to create accel thread: %d\n", err);
          goto unreg_gyro_handler;
        }

      sninfo("LSM6DSL accel using polling thread.\n");
    }

  if (err < 0)
    {
    unreg_gyro_handler:
      if (config->xl_attach != NULL)
        {
          kthread_delete(gyro_pid);
        }

    unreg_accel:
      sensor_unregister(&priv->accel.lower, devno);
    unreg_gyro:
      sensor_unregister(&priv->gyro.lower, devno);
    del_accel_sem:
      nxsem_destroy(&priv->accel.run);
    del_gyro_sem:
      nxsem_destroy(&priv->gyro.run);
    del_mutex:
      nxmutex_destroy(&priv->devlock);
    free_mem:
      kmm_free(priv);
      snerr("ERROR: Failed to register LSM6DSL driver: %d\n", err);
    }
  else
    {
      sninfo("LSM6DSL driver registered!\n");
    }

  return err;
}
