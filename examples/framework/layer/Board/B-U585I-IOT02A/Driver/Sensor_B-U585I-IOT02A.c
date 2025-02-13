/*
 * Copyright (c) 2025 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Exported sensor drivers:
 * - SensorDrv_TemperatureSensor
 * - SensorDrv_HumiditySensor
 * - SensorDrv_PressureSensor
 * - SensorDrv_Accelerometer
 * - SensorDrv_Gyroscope
 * - SensorDrv_Magnetometer
 */

#include "Sensor_B-U585I-IOT02A.h"

#include "b_u585i_iot02a_env_sensors.h"
#include "b_u585i_iot02a_motion_sensors.h"

#include "ism330dhcx_fifo.h"

#define STATE_RESET        0U
#define STATE_INITIALIZED  1U
#define STATE_STARTED      2U

static uint8_t TemperatureSensor_State = STATE_RESET;
static uint8_t HumiditySensor_State    = STATE_RESET;
static uint8_t PressureSensor_State    = STATE_RESET;
static uint8_t Accelerometer_State     = STATE_RESET;
static uint8_t Gyroscope_State         = STATE_RESET;
static uint8_t Magnetometer_State      = STATE_RESET;

// Macro for defining driver structures (for instances)
#define SENSOR_DRV(name)                \
SensorDrv_t SensorDrv_##name = {        \
  name##_Initialize,                    \
  name##_Uninitialize,                  \
  name##_GetInfo,                       \
  name##_CaptureStart,                  \
  name##_CaptureStop,                   \
  name##_GetStatus,                     \
  name##_Read                           \
};

#ifndef SENSOR_NO_LOCK
#include "cmsis_os2.h"

// Mutex lock
static osMutexId_t lock_id  = NULL;
static uint32_t    lock_cnt = 0U;

static inline void SensorLockCreate (void) {
  if (lock_cnt == 0U) {
    lock_id = osMutexNew(NULL);
  }
  lock_cnt++;
}

static inline void SensorLockDelete (void) {
  if (lock_cnt != 0U) {
    lock_cnt--;
    if (lock_cnt == 0U) {
      osMutexDelete(lock_id);
    }
  }
}

static inline void SensorLock (void) {
  osMutexAcquire(lock_id, osWaitForever);
}

static inline void SensorUnLock (void) {
  osMutexRelease(lock_id);
}

#else

static inline void SensorLockCreate (void) {}
static inline void SensorLockDelete (void) {}
static inline void SensorLock       (void) {}
static inline void SensorUnLock     (void) {}

#endif

//--------------------------------------------------------------------
// Temperature Sensor
//--------------------------------------------------------------------
static SensorDrv_Info_t TemperatureSensor_Info = {
  .name       = "Temperature",
  .channels   = 1U,
  .data_type  = SENSOR_DRV_DATA_FLOAT,
  .data_rate  = 1.0f,
  .scale      = 1.0f,
  .offset     = 0.0f,
  .unit       = "C",
  .fifo_size  = 4U,
  .threshold  = 0U,
};

/// Initializes the temperature sensor driver.
static int32_t TemperatureSensor_Initialize (SensorDrv_Event_t event_cb, uint32_t event_mask) {
  int32_t ret = SENSOR_DRV_ERROR;

  if ((event_cb != NULL) || (event_mask != 0U)) {
    return SENSOR_DRV_UNSUPPORTED;
  }

  SensorLockCreate();
  SensorLock();
  if (BSP_ENV_SENSOR_Init(0U, ENV_TEMPERATURE) == BSP_ERROR_NONE) {
    if (BSP_ENV_SENSOR_Disable(0U, ENV_TEMPERATURE) == BSP_ERROR_NONE) {
      TemperatureSensor_State = STATE_INITIALIZED;
      ret = SENSOR_DRV_OK;
    }
  }
  SensorUnLock();

  return ret;
}

/// Un-initializes the temperature sensor driver.
static int32_t TemperatureSensor_Uninitialize (void) {
  int32_t ret, bsp_err = BSP_ERROR_NONE;

  if (TemperatureSensor_State == STATE_RESET) {
    return SENSOR_DRV_OK;
  }

  SensorLock();
  if (TemperatureSensor_State == STATE_STARTED) {
    bsp_err = BSP_ENV_SENSOR_Disable(0U, ENV_TEMPERATURE);
    if (bsp_err == BSP_ERROR_NONE) {
      TemperatureSensor_State = STATE_INITIALIZED;
    }
  }
  if (bsp_err == BSP_ERROR_NONE) {
    if (HumiditySensor_State == STATE_RESET) {
      bsp_err = BSP_ENV_SENSOR_DeInit(0U);
      if (bsp_err == BSP_ERROR_NONE) {
        HumiditySensor_State = STATE_RESET;
      }
    } else {
      HumiditySensor_State = STATE_RESET;
    }
  }
  SensorUnLock();
  SensorLockDelete();

  if (bsp_err == BSP_ERROR_NONE) {
    ret = SENSOR_DRV_OK;
  } else {
    ret = SENSOR_DRV_ERROR;
  }

  return ret;
}

/// Gets temperature sensor information.
static SensorDrv_Info_t *TemperatureSensor_GetInfo (void) {
  return &TemperatureSensor_Info;
}

/// Starts capturing temperature sensor data.
static int32_t TemperatureSensor_CaptureStart (void) {
  int32_t ret = SENSOR_DRV_ERROR;
  float   value;

  if (TemperatureSensor_State == STATE_RESET) {
    return SENSOR_DRV_ERROR;
  }
  if (TemperatureSensor_State == STATE_STARTED) {
    return SENSOR_DRV_OK;
  }

  SensorLock();
  if (BSP_ENV_SENSOR_Enable(0, ENV_TEMPERATURE) == BSP_ERROR_NONE) {
    BSP_ENV_SENSOR_GetValue(0, ENV_TEMPERATURE, &value);
    TemperatureSensor_State = STATE_STARTED;
    ret = SENSOR_DRV_OK;
  }
  SensorUnLock();

  return ret;
}

/// Stops capturing temperature sensor data.
static int32_t TemperatureSensor_CaptureStop (void) {
  int32_t ret = SENSOR_DRV_ERROR;

  if (TemperatureSensor_State == STATE_RESET) {
    return SENSOR_DRV_ERROR;
  }
  if (TemperatureSensor_State == STATE_INITIALIZED) {
    return SENSOR_DRV_OK;
  }

  SensorLock();
  if (BSP_ENV_SENSOR_Disable(0, ENV_TEMPERATURE) == BSP_ERROR_NONE) {
    TemperatureSensor_State = STATE_INITIALIZED;
    ret = SENSOR_DRV_OK;
  }
  SensorUnLock();

  return ret;
}

/// Returns temperature sensor's current status.
static SensorDrv_Status_t TemperatureSensor_GetStatus (void) {
  SensorDrv_Status_t status;

  if (TemperatureSensor_State == STATE_STARTED) {
    status.active = 1U;
  } else {
    status.active = 0U;
  }
  status.overflow = 0U;  // Overflow detection not supported

  return status;
}

/// Reads data from the temperature sensor into the provided buffer.
static uint32_t TemperatureSensor_Read (void *buf, uint32_t buf_size) {
  uint32_t size, num = 0U;
  int32_t  ret;
  uint8_t  stat;
  float    value;

  if (TemperatureSensor_State == STATE_RESET) {
    return 0;
  }
  size = SENSOR_DRV_DATA_SIZE(TemperatureSensor_Info.data_type) * TemperatureSensor_Info.channels;
  if ((buf == NULL) || (buf_size < size)) {
    return 0U;
  }

  SensorLock();
  ret = HTS221_TEMP_Get_DRDY_Status(Env_Sensor_CompObj[0], &stat);
  if ((ret == 0) && (stat != 0U)) {
    if (BSP_ENV_SENSOR_GetValue(0, ENV_TEMPERATURE, &value) == BSP_ERROR_NONE) {
      memcpy(buf, &value, size);
      num = size;
    }
  }
  SensorUnLock();

  return num;
}

//--------------------------------------------------------------------
// Humidity Sensor
//--------------------------------------------------------------------
static SensorDrv_Info_t HumiditySensor_Info = {
  .name       = "Humidity",
  .channels   = 1U,
  .data_type  = SENSOR_DRV_DATA_FLOAT,
  .data_rate  = 1.0f,
  .scale      = 1.0f,
  .offset     = 0.0f,
  .unit       = "%RH",
  .fifo_size  = 4U,
  .threshold  = 0U,
};

/// Initializes the humidity sensor driver.
static int32_t HumiditySensor_Initialize (SensorDrv_Event_t event_cb, uint32_t event_mask) {
  int32_t ret = SENSOR_DRV_ERROR;

  if ((event_cb != NULL) || (event_mask != 0U)) {
    return SENSOR_DRV_UNSUPPORTED;
  }

  SensorLockCreate();
  SensorLock();
  if (BSP_ENV_SENSOR_Init(0U, ENV_HUMIDITY) == BSP_ERROR_NONE) {
    if (BSP_ENV_SENSOR_Disable(0U, ENV_HUMIDITY) == BSP_ERROR_NONE) {
      HumiditySensor_State = STATE_INITIALIZED;
      ret = SENSOR_DRV_OK;
    }
  }
  SensorUnLock();

  return ret;
}

/// Un-initializes the humidity sensor driver.
static int32_t HumiditySensor_Uninitialize (void) {
  int32_t ret, bsp_err = BSP_ERROR_NONE;

  if (HumiditySensor_State == STATE_RESET) {
    return SENSOR_DRV_OK;
  }

  SensorLock();
  if (HumiditySensor_State == STATE_STARTED) {
    bsp_err = BSP_ENV_SENSOR_Disable(0U, ENV_HUMIDITY);
    if (bsp_err == BSP_ERROR_NONE) {
      HumiditySensor_State = STATE_INITIALIZED;
    }
  }
  if (bsp_err == BSP_ERROR_NONE) {
    if (TemperatureSensor_State == STATE_RESET) {
      bsp_err = BSP_ENV_SENSOR_DeInit(0U);
      if (bsp_err == BSP_ERROR_NONE) {
        HumiditySensor_State = STATE_RESET;
      }
    } else {
      TemperatureSensor_State = STATE_RESET;
    }
  }
  SensorUnLock();
  SensorLockDelete();

  if (bsp_err == BSP_ERROR_NONE) {
    ret = SENSOR_DRV_OK;
  } else {
    ret = SENSOR_DRV_ERROR;
  }

  return ret;
}

/// Gets humidity sensor information.
static SensorDrv_Info_t *HumiditySensor_GetInfo (void) {
  return &HumiditySensor_Info;
}

/// Starts capturing humidity sensor data.
static int32_t HumiditySensor_CaptureStart (void) {
  int32_t ret = SENSOR_DRV_ERROR;
  float   value;

  if (HumiditySensor_State == STATE_RESET) {
    return SENSOR_DRV_ERROR;
  }
  if (HumiditySensor_State == STATE_STARTED) {
    return SENSOR_DRV_OK;
  }

  SensorLock();
  if (BSP_ENV_SENSOR_Enable(0, ENV_HUMIDITY) == BSP_ERROR_NONE) {
    BSP_ENV_SENSOR_GetValue(0, ENV_HUMIDITY, &value);
    HumiditySensor_State = STATE_STARTED;
    ret = SENSOR_DRV_OK;
  }
  SensorUnLock();

  return ret;
}

/// Stops capturing humidity sensor data.
static int32_t HumiditySensor_CaptureStop (void) {
  int32_t ret = SENSOR_DRV_ERROR;

  if (HumiditySensor_State == STATE_RESET) {
    return SENSOR_DRV_ERROR;
  }
  if (HumiditySensor_State == STATE_INITIALIZED) {
    return SENSOR_DRV_OK;
  }

  SensorLock();
  if (BSP_ENV_SENSOR_Disable(0, ENV_HUMIDITY) == BSP_ERROR_NONE) {
    HumiditySensor_State = STATE_INITIALIZED;
    ret = SENSOR_DRV_OK;
  }
  SensorUnLock();

  return ret;
}

/// Returns humidity sensor's current status.
static SensorDrv_Status_t HumiditySensor_GetStatus (void) {
  SensorDrv_Status_t status;

  if (HumiditySensor_State == STATE_STARTED) {
    status.active = 1U;
  } else {
    status.active = 0U;
  }
  status.overflow = 0U;  // Overflow detection not supported

  return status;
}

/// Reads data from the humidity sensor into the provided buffer.
static uint32_t HumiditySensor_Read (void *buf, uint32_t buf_size) {
  uint32_t size, num = 0U;
  int32_t  ret;
  uint8_t  stat;
  float    value;

  if (HumiditySensor_State == STATE_RESET) {
    return 0;
  }
  size = SENSOR_DRV_DATA_SIZE(HumiditySensor_Info.data_type) * HumiditySensor_Info.channels;
  if ((buf == NULL) || (buf_size < size)) {
    return 0U;
  }

  SensorLock();
  ret = HTS221_HUM_Get_DRDY_Status(Env_Sensor_CompObj[0], &stat);
  if ((ret == 0) && (stat != 0U)) {
    if (BSP_ENV_SENSOR_GetValue(0, ENV_HUMIDITY, &value) == BSP_ERROR_NONE) {
      memcpy(buf, &value, size);
      num = size;
    }
  }
  SensorUnLock();

  return num;
}

//--------------------------------------------------------------------
// Pressure Sensor
//--------------------------------------------------------------------
static SensorDrv_Info_t PressureSensor_Info = {
  .name       = "Pressure",
  .channels   = 1U,
  .data_type  = SENSOR_DRV_DATA_FLOAT,
  .data_rate  = 40.0f,
  .scale      = 1.0f,
  .offset     = 0.0f,
  .unit       = "hPa",
  .fifo_size  = 4U,
  .threshold  = 0U,
};

/// Initializes the pressure sensor driver.
static int32_t PressureSensor_Initialize (SensorDrv_Event_t event_cb, uint32_t event_mask) {
  int32_t ret = SENSOR_DRV_ERROR;

  if ((event_cb != NULL) || (event_mask != 0U)) {
    return SENSOR_DRV_UNSUPPORTED;
  }

  SensorLockCreate();
  SensorLock();
  if (BSP_ENV_SENSOR_Init(1U, ENV_PRESSURE) == BSP_ERROR_NONE) {
    if (BSP_ENV_SENSOR_Disable(1U, ENV_PRESSURE) == BSP_ERROR_NONE) {
      PressureSensor_State = STATE_INITIALIZED;
      ret = SENSOR_DRV_OK;
    }
  }
  SensorUnLock();

  return ret;
}

/// Un-initializes the pressure sensor driver.
static int32_t PressureSensor_Uninitialize (void) {
  int32_t ret, bsp_err = BSP_ERROR_NONE;

  if (PressureSensor_State == STATE_RESET) {
    return SENSOR_DRV_OK;
  }

  SensorLock();
  if (PressureSensor_State == STATE_STARTED) {
    bsp_err = BSP_ENV_SENSOR_Disable(1U, ENV_PRESSURE);
    if (bsp_err == BSP_ERROR_NONE) {
      PressureSensor_State = STATE_INITIALIZED;
    }
  }
  if (bsp_err == BSP_ERROR_NONE) {
    bsp_err = BSP_ENV_SENSOR_DeInit(1U);
    if (bsp_err == BSP_ERROR_NONE) {
      PressureSensor_State = STATE_RESET;
    }
  }
  SensorUnLock();
  SensorLockDelete();

  if (bsp_err == BSP_ERROR_NONE) {
    ret = SENSOR_DRV_OK;
  } else {
    ret = SENSOR_DRV_ERROR;
  }

  return ret;
}

/// Gets pressure sensor information.
static SensorDrv_Info_t *PressureSensor_GetInfo (void) {
  return &PressureSensor_Info;
}

/// Starts capturing pressure sensor data.
static int32_t PressureSensor_CaptureStart (void) {
  int32_t ret = SENSOR_DRV_ERROR;
  float   value;

  if (PressureSensor_State == STATE_RESET) {
    return SENSOR_DRV_ERROR;
  }
  if (PressureSensor_State == STATE_STARTED) {
    return SENSOR_DRV_OK;
  }

  SensorLock();
  if (BSP_ENV_SENSOR_Enable(1, ENV_PRESSURE) == BSP_ERROR_NONE) {
    PressureSensor_State = STATE_STARTED;
    ret = SENSOR_DRV_OK;
  }
  SensorUnLock();

  return ret;
}

/// Stops capturing pressure sensor data.
static int32_t PressureSensor_CaptureStop (void) {
  int32_t ret = SENSOR_DRV_ERROR;

  if (PressureSensor_State == STATE_RESET) {
    return SENSOR_DRV_ERROR;
  }
  if (PressureSensor_State == STATE_INITIALIZED) {
    return SENSOR_DRV_OK;
  }

  SensorLock();
  if (BSP_ENV_SENSOR_Disable(1, ENV_PRESSURE) == BSP_ERROR_NONE) {
    PressureSensor_State = STATE_INITIALIZED;
    ret = SENSOR_DRV_OK;
  }
  SensorUnLock();

  return ret;
}

/// Returns pressure sensor's current status.
static SensorDrv_Status_t PressureSensor_GetStatus (void) {
  SensorDrv_Status_t status;

  if (PressureSensor_State == STATE_STARTED) {
    status.active = 1U;
  } else {
    status.active = 0U;
  }
  status.overflow = 0U;  // Overflow detection not supported

  return status;
}

/// Reads data from the pressure sensor into the provided buffer.
static uint32_t PressureSensor_Read (void *buf, uint32_t buf_size) {
  uint32_t size, num = 0U;
  int32_t  ret;
  uint8_t  stat;
  float    value;

  if (PressureSensor_State == STATE_RESET) {
    return 0;
  }
  size = SENSOR_DRV_DATA_SIZE(PressureSensor_Info.data_type) * PressureSensor_Info.channels;
  if ((buf == NULL) || (buf_size < size)) {
    return 0U;
  }

  SensorLock();
  ret = LPS22HH_PRESS_Get_DRDY_Status(Env_Sensor_CompObj[1], &stat);
  if ((ret == 0) && (stat != 0U)) {
    if (BSP_ENV_SENSOR_GetValue(1, ENV_PRESSURE, &value) == BSP_ERROR_NONE) {
      memcpy(buf, &value, size);
      num = size;
    }
  }
  SensorUnLock();

  return num;
}

//--------------------------------------------------------------------
// Accelerometer Sensor
//--------------------------------------------------------------------
static SensorDrv_Info_t Accelerometer_Info = {
  .name       = "Accelerometer",
  .channels   = 3U,
  .data_type  = SENSOR_DRV_DATA_INT16,
  .data_rate  = 1666.0f,
  .scale      = 0.00061f,
  .offset     = 0.0f,
  .unit       = "m/s^2",
  .fifo_size  = 32U,
  .threshold  = 0U,
};

/// Initializes the accelerometer sensor driver.
static int32_t Accelerometer_Initialize (SensorDrv_Event_t event_cb, uint32_t event_mask) {
  int32_t err, ret = SENSOR_DRV_ERROR;

  if ((event_cb != NULL) || (event_mask != 0U)) {
    return SENSOR_DRV_UNSUPPORTED;
  }

  SensorLockCreate();
  SensorLock();
  err = BSP_MOTION_SENSOR_Init(0U, MOTION_ACCELERO);
  if (err == 0) {
    err = BSP_MOTION_SENSOR_Disable(0U, MOTION_ACCELERO);
  }
  if (err == 0) {
    err = ISM330DHCX_ACC_SetFullScale(Motion_Sensor_CompObj[0], ISM330DHCX_2g);
  }
  if (err == 0) {
    err = ISM330DHCX_ACC_SetOutputDataRate(Motion_Sensor_CompObj[0], 1666.0f);
  }
  if (err == 0) {
    err = ISM330DHCX_FIFO_ACC_Set_BDR(Motion_Sensor_CompObj[0], 1666.0f);
  }
  if (err == 0) {
    err = ISM330DHCX_FIFO_Set_Mode(Motion_Sensor_CompObj[0], ISM330DHCX_STREAM_MODE);
  }
  SensorUnLock();

  if (err == 0) {
    Accelerometer_State = STATE_INITIALIZED;
    ret = SENSOR_DRV_OK;
  }

  return ret;
}

/// Un-initializes the accelerometer sensor driver.
static int32_t Accelerometer_Uninitialize (void) {
  int32_t ret, bsp_err = BSP_ERROR_NONE;

  if (Accelerometer_State == STATE_RESET) {
    return SENSOR_DRV_OK;
  }

  SensorLock();
  if (Accelerometer_State == STATE_STARTED) {
    bsp_err = BSP_MOTION_SENSOR_Disable(0U, MOTION_ACCELERO);
    if (bsp_err == BSP_ERROR_NONE) {
      Accelerometer_State = STATE_INITIALIZED;
    }
  }
  if (bsp_err == BSP_ERROR_NONE) {
    if (Gyroscope_State == STATE_RESET) {
      bsp_err = BSP_MOTION_SENSOR_DeInit(0U);
      if (bsp_err == BSP_ERROR_NONE) {
        Accelerometer_State = STATE_RESET;
      }
    } else {
      Accelerometer_State = STATE_RESET;
    }
  }
  SensorUnLock();
  SensorLockDelete();

  if (bsp_err == BSP_ERROR_NONE) {
    ret = SENSOR_DRV_OK;
  } else {
    ret = SENSOR_DRV_ERROR;
  }

  return ret;
}

/// Gets accelerometer sensor information.
static SensorDrv_Info_t *Accelerometer_GetInfo (void) {
  return &Accelerometer_Info;
}

/// Starts capturing accelerometer sensor data.
static int32_t Accelerometer_CaptureStart (void) {
  uint32_t buf[2];
  int32_t  ret = SENSOR_DRV_ERROR;

  if (Accelerometer_State == STATE_RESET) {
    return SENSOR_DRV_ERROR;
  }
  if (Accelerometer_State == STATE_STARTED) {
    return SENSOR_DRV_OK;
  }

  SensorLock();
  if (ISM330DHCX_FIFO_Init(ISM330DHCX_ID_ACCELEROMETER) == 0) {
    if ((Accelerometer_State != STATE_STARTED) && (Gyroscope_State != STATE_STARTED)) {
      // Clear ISM330DHCX FIFO
      while (ISM330DHCX_FIFO_Read(ISM330DHCX_ID_ACCELEROMETER, 1, (uint8_t *)buf) != 0U);
    }
    if (ISM330DHCX_ACC_Enable(Motion_Sensor_CompObj[0]) == 0) {
      Accelerometer_State = STATE_STARTED;
      ret = SENSOR_DRV_OK;
    }
  }
  SensorUnLock();

  return ret;
}

/// Stops capturing accelerometer sensor data.
static int32_t Accelerometer_CaptureStop (void) {
  int32_t ret = SENSOR_DRV_ERROR;

  if (Accelerometer_State == STATE_RESET) {
    return SENSOR_DRV_ERROR;
  }
  if (Accelerometer_State == STATE_INITIALIZED) {
    return SENSOR_DRV_OK;
  }

  SensorLock();
  if (ISM330DHCX_ACC_Disable(Motion_Sensor_CompObj[0]) == 0) {
    if (ISM330DHCX_FIFO_Uninit(ISM330DHCX_ID_ACCELEROMETER) == 0) {
      Accelerometer_State = STATE_INITIALIZED;
      ret = SENSOR_DRV_OK;
    }
  }
  SensorUnLock();

  return ret;
}

/// Returns accelerometer sensor's current status.
static SensorDrv_Status_t Accelerometer_GetStatus (void) {
  SensorDrv_Status_t status;

  if (Accelerometer_State == STATE_STARTED) {
    status.active = 1U;
  } else {
    status.active = 0U;
  }
  status.overflow = 0U;

  return status;
}

/// Reads data from the accelerometer sensor into the provided buffer.
static uint32_t Accelerometer_Read (void *buf, uint32_t buf_size) {
  uint32_t size, num = 0U;
  int32_t  ret;
  BSP_MOTION_SENSOR_Axes_t axes;

  if (Accelerometer_State == STATE_RESET) {
    return 0;
  }
  size = SENSOR_DRV_DATA_SIZE(Accelerometer_Info.data_type) * Accelerometer_Info.channels;
  if ((buf == NULL) || (buf_size < size)) {
    return 0U;
  }

  SensorLock();
  num = ISM330DHCX_FIFO_Read(ISM330DHCX_ID_ACCELEROMETER, buf_size / size, buf);
  num *= size;
  SensorUnLock();

  return num;
}

//--------------------------------------------------------------------
// Gyroscope Sensor
//--------------------------------------------------------------------
static SensorDrv_Info_t Gyroscope_Info = {
  .name       = "Gyroscope",
  .channels   = 3U,
  .data_type  = SENSOR_DRV_DATA_INT16,
  .data_rate  = 1666.0f,
  .scale      = 0.035f,
  .offset     = 0.0f,
  .unit       = "dps",
  .fifo_size  = 32U,
  .threshold  = 0U,
};

/// Initializes the gyroscope sensor driver.
static int32_t Gyroscope_Initialize (SensorDrv_Event_t event_cb, uint32_t event_mask) {
  int32_t err, ret = SENSOR_DRV_ERROR;

  if ((event_cb != NULL) || (event_mask != 0U)) {
    return SENSOR_DRV_UNSUPPORTED;
  }

  SensorLockCreate();
  SensorLock();
  err = BSP_MOTION_SENSOR_Init(0U, MOTION_GYRO);
  if (err == 0) {
    err = BSP_MOTION_SENSOR_Disable(0U, MOTION_GYRO);
  }
  if (err == 0) {
    err = ISM330DHCX_GYRO_SetFullScale(Motion_Sensor_CompObj[0], ISM330DHCX_2000dps);
  }
  if (err == 0) {
    err = ISM330DHCX_GYRO_SetOutputDataRate(Motion_Sensor_CompObj[0], 1666.0f);
  }
  if (err == 0) {
    err = ISM330DHCX_FIFO_GYRO_Set_BDR(Motion_Sensor_CompObj[0], 1666.0f);
  }
  if (err == 0) {
    err = ISM330DHCX_FIFO_Set_Mode(Motion_Sensor_CompObj[0], ISM330DHCX_STREAM_MODE);
  }
  SensorUnLock();

  if (err == 0) {
    Gyroscope_State = STATE_INITIALIZED;
    ret = SENSOR_DRV_OK;
  }

  return ret;
}

/// Un-initializes the gyroscope sensor driver.
static int32_t Gyroscope_Uninitialize (void) {
  int32_t ret, bsp_err = BSP_ERROR_NONE;

  if (Gyroscope_State == STATE_RESET) {
    return SENSOR_DRV_OK;
  }

  SensorLock();
  if (Gyroscope_State == STATE_STARTED) {
    bsp_err = BSP_MOTION_SENSOR_Disable(0U, MOTION_GYRO);
    if (bsp_err == BSP_ERROR_NONE) {
      Gyroscope_State = STATE_INITIALIZED;
    }
  }
  if (bsp_err == BSP_ERROR_NONE) {
    if (Accelerometer_State == STATE_RESET) {
      bsp_err = BSP_MOTION_SENSOR_DeInit(0U);
      if (bsp_err == BSP_ERROR_NONE) {
        Gyroscope_State = STATE_RESET;
      }
    } else {
      Gyroscope_State = STATE_RESET;
    }
  }
  SensorUnLock();
  SensorLockDelete();

  if (bsp_err == BSP_ERROR_NONE) {
    ret = SENSOR_DRV_OK;
  } else {
    ret = SENSOR_DRV_ERROR;
  }

  return ret;
}

/// Gets gyroscope sensor information.
static SensorDrv_Info_t *Gyroscope_GetInfo (void) {
  return &Gyroscope_Info;
}

/// Starts capturing gyroscope sensor data.
static int32_t Gyroscope_CaptureStart (void) {
  uint32_t buf[2];
  int32_t  ret = SENSOR_DRV_ERROR;

  if (Gyroscope_State == STATE_RESET) {
    return SENSOR_DRV_ERROR;
  }
  if (Gyroscope_State == STATE_STARTED) {
    return SENSOR_DRV_OK;
  }

  SensorLock();
  if (ISM330DHCX_FIFO_Init(ISM330DHCX_ID_GYROSCOPE) == 0) {
    if ((Gyroscope_State != STATE_STARTED) && (Accelerometer_State != STATE_STARTED)) {
      while (ISM330DHCX_FIFO_Read(ISM330DHCX_ID_GYROSCOPE, 1, (uint8_t *)buf) != 0U);
    }
    if (ISM330DHCX_GYRO_Enable(Motion_Sensor_CompObj[0]) == 0) {
      Gyroscope_State = STATE_STARTED;
      ret = SENSOR_DRV_OK;
    }
  }
  SensorUnLock();

  return ret;
}

/// Stops capturing gyroscope sensor data.
static int32_t Gyroscope_CaptureStop (void) {
  int32_t ret = SENSOR_DRV_ERROR;

  if (Gyroscope_State == STATE_RESET) {
    return SENSOR_DRV_ERROR;
  }
  if (Gyroscope_State == STATE_INITIALIZED) {
    return SENSOR_DRV_OK;
  }

  SensorLock();
  if (ISM330DHCX_GYRO_Disable(Motion_Sensor_CompObj[0]) == 0) {
    if (ISM330DHCX_FIFO_Uninit(ISM330DHCX_ID_GYROSCOPE) == 0) {
      Gyroscope_State = STATE_INITIALIZED;
      ret = SENSOR_DRV_OK;
    }
  }
  SensorUnLock();

  return ret;
}

/// Returns gyroscope sensor's current status.
static SensorDrv_Status_t Gyroscope_GetStatus (void) {
  SensorDrv_Status_t status;

  if (Gyroscope_State == STATE_STARTED) {
    status.active = 1U;
  } else {
    status.active = 0U;
  }
  status.overflow = 0U;

  return status;
}

/// Reads data from the gyroscope sensor into the provided buffer.
static uint32_t Gyroscope_Read (void *buf, uint32_t buf_size) {
  uint32_t size, num = 0U;
  int32_t  ret;
  BSP_MOTION_SENSOR_Axes_t axes;

  if (Gyroscope_State == STATE_RESET) {
    return 0;
  }
  size = SENSOR_DRV_DATA_SIZE(Gyroscope_Info.data_type) * Gyroscope_Info.channels;
  if ((buf == NULL) || (buf_size < size)) {
    return 0U;
  }

  SensorLock();
  num = ISM330DHCX_FIFO_Read(ISM330DHCX_ID_GYROSCOPE, buf_size / size, buf);
  num *= size;
  SensorUnLock();

  return num;
}

//--------------------------------------------------------------------
// Magnetometer Sensor
//--------------------------------------------------------------------
static SensorDrv_Info_t Magnetometer_Info = {
  .name       = "Magnetometer",
  .channels   = 3U,
  .data_type  = SENSOR_DRV_DATA_INT16,
  .data_rate  = 100.0f,
  .scale      = 0.15f,
  .offset     = 0.0f,
  .unit       = "uT",
  .fifo_size  = 6U,
  .threshold  = 0U,
};

/// Initializes the magnetometer sensor driver.
static int32_t Magnetometer_Initialize (SensorDrv_Event_t event_cb, uint32_t event_mask) {
  int32_t ret = SENSOR_DRV_ERROR;

  if ((event_cb != NULL) || (event_mask != 0U)) {
    return SENSOR_DRV_UNSUPPORTED;
  }

  SensorLockCreate();
  SensorLock();
  if (BSP_MOTION_SENSOR_Init(1U, MOTION_MAGNETO) == BSP_ERROR_NONE) {
    if (BSP_MOTION_SENSOR_Disable(1U, MOTION_MAGNETO) == BSP_ERROR_NONE) {
      Magnetometer_State = STATE_INITIALIZED;
      ret = SENSOR_DRV_OK;
    }
  }
  SensorUnLock();

  return ret;
}

/// Un-initializes the magnetometer sensor driver.
static int32_t Magnetometer_Uninitialize (void) {
  int32_t ret, bsp_err = BSP_ERROR_NONE;

  if (Magnetometer_State == STATE_RESET) {
    return SENSOR_DRV_OK;
  }

  SensorLock();
  if (Magnetometer_State == STATE_STARTED) {
    bsp_err = BSP_MOTION_SENSOR_Disable(1U, MOTION_MAGNETO);
    if (bsp_err == BSP_ERROR_NONE) {
      Magnetometer_State = STATE_INITIALIZED;
    }
  }
  if (bsp_err == BSP_ERROR_NONE) {
    bsp_err = BSP_MOTION_SENSOR_DeInit(1U);
    if (bsp_err == BSP_ERROR_NONE) {
      Magnetometer_State = STATE_RESET;
    }
  }
  SensorUnLock();
  SensorLockDelete();

  if (bsp_err == BSP_ERROR_NONE) {
    ret = SENSOR_DRV_OK;
  } else {
    ret = SENSOR_DRV_ERROR;
  }

  return ret;
}

/// Gets magnetometer sensor information.
static SensorDrv_Info_t *Magnetometer_GetInfo (void) {
  return &Magnetometer_Info;
}

/// Starts capturing magnetometer sensor data.
static int32_t Magnetometer_CaptureStart (void) {
  int32_t ret = SENSOR_DRV_ERROR;

  if (Magnetometer_State == STATE_RESET) {
    return SENSOR_DRV_ERROR;
  }
  if (Magnetometer_State == STATE_STARTED) {
    return SENSOR_DRV_OK;
  }

  SensorLock();
  if (BSP_MOTION_SENSOR_Enable(1, MOTION_MAGNETO) == BSP_ERROR_NONE) {
    Magnetometer_State = STATE_STARTED;
    ret = SENSOR_DRV_OK;
  }
  SensorUnLock();

  return ret;
}

/// Stops capturing magnetometer sensor data.
static int32_t Magnetometer_CaptureStop (void) {
  int32_t ret = SENSOR_DRV_ERROR;

  if (Magnetometer_State == STATE_RESET) {
    return SENSOR_DRV_ERROR;
  }
  if (Magnetometer_State == STATE_INITIALIZED) {
    return SENSOR_DRV_OK;
  }

  SensorLock();
  if (BSP_MOTION_SENSOR_Disable(1, MOTION_MAGNETO) == BSP_ERROR_NONE) {
    Magnetometer_State = STATE_INITIALIZED;
    ret = SENSOR_DRV_OK;
  }
  SensorUnLock();

  return ret;
}

/// Returns magnetometer sensor's current status.
static SensorDrv_Status_t Magnetometer_GetStatus (void) {
  SensorDrv_Status_t status;

  if (Magnetometer_State == STATE_STARTED) {
    status.active = 1U;
  } else {
    status.active = 0U;
  }
  status.overflow = 0U;

  return status;
}

/// Reads data from the magnetometer sensor into the provided buffer.
static uint32_t Magnetometer_Read (void *buf, uint32_t buf_size) {
  uint32_t size, num = 0U;
  int32_t  ret;
  uint8_t  stat;
  BSP_MOTION_SENSOR_AxesRaw_t axes;

  if (Magnetometer_State == STATE_RESET) {
    return 0;
  }
  size = SENSOR_DRV_DATA_SIZE(Magnetometer_Info.data_type) * Magnetometer_Info.channels;
  if ((buf == NULL) || (buf_size < size)) {
    return 0U;
  }

  SensorLock();
  ret = IIS2MDC_MAG_Get_DRDY_Status(Motion_Sensor_CompObj[1], &stat);
  if ((ret == 0) && (stat != 0U)) {
    if (BSP_MOTION_SENSOR_GetAxesRaw(1, MOTION_MAGNETO, &axes) == BSP_ERROR_NONE) {
      memcpy(buf, &axes, size);
      num = size;
    }
  }
  SensorUnLock();

  return num;
}

/// Exported drivers
SENSOR_DRV(TemperatureSensor);
SENSOR_DRV(HumiditySensor);
SENSOR_DRV(PressureSensor);
SENSOR_DRV(Accelerometer);
SENSOR_DRV(Gyroscope);
SENSOR_DRV(Magnetometer);






