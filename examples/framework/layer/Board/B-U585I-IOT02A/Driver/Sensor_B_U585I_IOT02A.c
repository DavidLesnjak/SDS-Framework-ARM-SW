/*
 * Copyright (c) 2025 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "sensor_drv.h"

#include "b_u585i_iot02a_env_sensors.h"
#include "b_u585i_iot02a_motion_sensors.h"
#include "ism330dhcx.h"

extern int32_t BSP_I2C2_Init(void);
extern int32_t BSP_I2C2_DeInit(void);
extern int32_t BSP_I2C2_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
extern int32_t BSP_I2C2_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
extern int32_t BSP_GetTick(void);

uint8_t TemperatureSensor_Initialized = 0U;
uint8_t TemperatureSensor_Started     = 0U;
uint8_t HumiditySensor_Initialized    = 0U;
uint8_t HumiditySensor_Started        = 0U;
uint8_t PressureSensor_Initialized    = 0U;
uint8_t PressureSensor_Started        = 0U;
uint8_t Accelerometer_Initialized     = 0U;
uint8_t Accelerometer_Started         = 0U;
uint8_t Gyroscope_Initialized         = 0U;
uint8_t Gyroscope_Started             = 0U;
uint8_t Magnetometer_Initialized      = 0U;
uint8_t Magnetometer_Started          = 0U;

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

static const SensorDrv_DataRates_t TemperatureSensor_Rates = {
  .num = 4,
  .rates = {0.0f, 1.0f, 7.0f, 12.5f}
};

static const SensorDrv_Ranges_t TemperatureSensor_Ranges = {
   .num = 1,
   .ranges = {{-40.0f, 120.0f}}
};

static SensorDrv_Info_t TemperatureSensor_Info = {
  .name       = "Temperature",
  .channels   = 1,
  .data_type  = SENSOR_DRV_DATA_FLOAT,
  .unit       = "C",
  .data_rates = &TemperatureSensor_Rates,
  .ranges     = &TemperatureSensor_Ranges,
  .fifo_size  = 4,
};

/// Initializes the temperature sensor driver.
static int32_t TemperatureSensor_Initialize (SensorDrv_Event_t event_cb, uint32_t event_mask) {
  int32_t ret;

  if ((event_cb != NULL) || (event_mask != 0U)) {
    return SENSOR_DRV_UNSUPPORTED;
  }

  SensorLockCreate();
  SensorLock();
  if (BSP_ENV_SENSOR_Init(0U, ENV_TEMPERATURE) == BSP_ERROR_NONE) {
    TemperatureSensor_Initialized = 1U;
    ret = SENSOR_DRV_OK;
  } else {
    ret = SENSOR_DRV_ERROR;
  }
  SensorUnLock();

  return ret;
}

/// Un-initializes the temperature sensor driver.
static int32_t TemperatureSensor_Uninitialize (void) {
  int32_t ret;

  if (HumiditySensor_Initialized == 0U) {
    SensorLock();
    if (BSP_ENV_SENSOR_DeInit(0U) == BSP_ERROR_NONE) {
      TemperatureSensor_Started     = 0U;
      TemperatureSensor_Initialized = 0U;
      ret = SENSOR_DRV_OK;
    } else {
      ret = SENSOR_DRV_ERROR;
    }
    SensorUnLock();
    SensorLockDelete();
  } else {
    TemperatureSensor_Started     = 0U;
    TemperatureSensor_Initialized = 0U;
    ret = SENSOR_DRV_OK;
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

  SensorLock();
  if (BSP_ENV_SENSOR_Enable(0, ENV_TEMPERATURE) == BSP_ERROR_NONE) {
    BSP_ENV_SENSOR_GetValue(0, ENV_TEMPERATURE, &value);
    TemperatureSensor_Started = 1U;
    ret = SENSOR_DRV_OK;
  }
  SensorUnLock();

  return ret;
}

/// Stops capturing temperature sensor data.
static int32_t TemperatureSensor_CaptureStop (void) {
  int32_t ret = SENSOR_DRV_ERROR;

  SensorLock();
  if (BSP_ENV_SENSOR_Disable(0, ENV_TEMPERATURE) == BSP_ERROR_NONE) {
    TemperatureSensor_Started = 0U;
    ret = SENSOR_DRV_OK;
  }
  SensorUnLock();

  return ret;
}

/// Sets the temperature sensor's data rate.
static int32_t TemperatureSensor_SetDataRate (float rate) {
  int32_t  ret = SENSOR_DRV_ERROR;
  uint32_t n;

  for (n = 0U; n < TemperatureSensor_Rates.num; n++) {
    if (rate == TemperatureSensor_Rates.rates[n]) {
      SensorLock();
      if (BSP_ENV_SENSOR_SetOutputDataRate(0U, ENV_TEMPERATURE, rate) == BSP_ERROR_NONE) {
        ret = SENSOR_DRV_OK;
      }
      SensorUnLock();
      break;
    }
  }

  return ret;
}

/// Gets the temperature sensor's current data rate.
static float TemperatureSensor_GetDataRate (void) {
  float rate = 0.0f;

  BSP_ENV_SENSOR_GetOutputDataRate(0U, ENV_TEMPERATURE, &rate);
  return rate;
}

/// Sets temperature sensor's measurement range (in measurement units).
static int32_t TemperatureSensor_SetRange (float min, float max) {
  int32_t ret;

  if ((min == TemperatureSensor_Ranges.ranges[0].min) &&
      (max == TemperatureSensor_Ranges.ranges[0].max)) {
      ret = SENSOR_DRV_OK;
  } else {
    ret = SENSOR_DRV_ERROR;
  }
  return ret;
}

/// Gets temperature sensor's current measurement range (in measurement units).
static int32_t TemperatureSensor_GetRange (float *min, float *max) {
  if ((min == NULL) || (max == NULL)) {
    return SENSOR_DRV_ERROR;
  }

  *min = TemperatureSensor_Ranges.ranges[0].min;
  *max = TemperatureSensor_Ranges.ranges[0].max;

  return SENSOR_DRV_OK;
}

/// Returns the temperature sensor's offset (in measurement units).
static float TemperatureSensor_GetOffset (void) {
  return 0.0f;
}

/// Returns temperature sensor's resolution.
static float TemperatureSensor_GetResolution (void) {
  return 1.0f;
}

/// Sets data event threshold in bytes.
static int32_t TemperatureSensor_SetDataThreshold (uint32_t threshold) {
  (void)threshold;
  return SENSOR_DRV_UNSUPPORTED;
}

/// Returns temperature sensor's current status.
static SensorDrv_Status_t TemperatureSensor_GetStatus (void) {
  SensorDrv_Status_t status;

  status.active   = (TemperatureSensor_Started != 0U) ? 1U : 0U;
  status.overflow = 0U;  // Overflow detection not supported

  return status;
}

/// Reads data from the temperature sensor into the provided buffer.
static uint32_t TemperatureSensor_Read (void *buf, uint32_t buf_size) {
  uint32_t num = 0U;
  int32_t  ret;
  uint8_t  stat;
  float    value;

  if ((buf == NULL) || (buf_size < sizeof(float))) {
    return 0U;
  }

  SensorLock();
  ret = HTS221_TEMP_Get_DRDY_Status(Env_Sensor_CompObj[0], &stat);
  if ((ret == 0) && (stat != 0U)) {
    if (BSP_ENV_SENSOR_GetValue(0, ENV_TEMPERATURE, &value) == BSP_ERROR_NONE) {
      memcpy(buf, &value, sizeof(float));
      num = (uint32_t)sizeof(float);
    }
  }
  SensorUnLock();

  return num;
}

//--------------------------------------------------------------------
// Humidity Sensor
//--------------------------------------------------------------------

static const SensorDrv_DataRates_t HumiditySensor_Rates = {
  .num = 4,
  .rates = {0.0f, 1.0f, 7.0f, 12.5f}
};

static const SensorDrv_Ranges_t HumiditySensor_Ranges = {
  .num = 1,
  .ranges = {{0.0f, 100.0f}}
};

static SensorDrv_Info_t HumiditySensor_Info = {
  .name       = "Humidity",
  .channels   = 1,
  .data_type  = SENSOR_DRV_DATA_FLOAT,
  .unit       = "%RH",
  .data_rates = &HumiditySensor_Rates,
  .ranges     = &HumiditySensor_Ranges,
  .fifo_size  = 4,
};

/// Initializes the humidity sensor driver.
static int32_t HumiditySensor_Initialize (SensorDrv_Event_t event_cb, uint32_t event_mask) {
  int32_t ret;

  if ((event_cb != NULL) || (event_mask != 0U)) {
    return SENSOR_DRV_UNSUPPORTED;
  }

  SensorLockCreate();
  SensorLock();
  if (BSP_ENV_SENSOR_Init(0U, ENV_HUMIDITY) == BSP_ERROR_NONE) {
    HumiditySensor_Initialized = 1U;
    ret = SENSOR_DRV_OK;
  } else {
    ret = SENSOR_DRV_ERROR;
  }
  SensorUnLock();

  return ret;
}

/// Un-initializes the humidity sensor driver.
static int32_t HumiditySensor_Uninitialize (void) {
  int32_t ret;

  if (TemperatureSensor_Initialized == 0U) {
    SensorLock();
    if (BSP_ENV_SENSOR_DeInit(0U) == BSP_ERROR_NONE) {
      HumiditySensor_Started     = 0U;
      HumiditySensor_Initialized = 0U;
      ret = SENSOR_DRV_OK;
    } else {
      ret = SENSOR_DRV_ERROR;
    }
    SensorUnLock();
    SensorLockDelete();
  } else {
    HumiditySensor_Started     = 0U;
    HumiditySensor_Initialized = 0U;
    ret = SENSOR_DRV_OK;
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

  SensorLock();
  if (BSP_ENV_SENSOR_Enable(0, ENV_HUMIDITY) == BSP_ERROR_NONE) {
    BSP_ENV_SENSOR_GetValue(0, ENV_HUMIDITY, &value);
    HumiditySensor_Started = 1U;
    ret = SENSOR_DRV_OK;
  }
  SensorUnLock();

  return ret;
}

/// Stops capturing humidity sensor data.
static int32_t HumiditySensor_CaptureStop (void) {
  int32_t ret = SENSOR_DRV_ERROR;

  SensorLock();
  if (BSP_ENV_SENSOR_Disable(0, ENV_HUMIDITY) == BSP_ERROR_NONE) {
    HumiditySensor_Started = 0U;
    ret = SENSOR_DRV_OK;
  }
  SensorUnLock();

  return ret;
}

/// Sets the humidity sensor's data rate.
static int32_t HumiditySensor_SetDataRate (float rate) {
  int32_t  ret = SENSOR_DRV_ERROR;
  uint32_t n;

  for (n = 0U; n < HumiditySensor_Rates.num; n++) {
    if (rate == HumiditySensor_Rates.rates[n]) {
      SensorLock();
      if (BSP_ENV_SENSOR_SetOutputDataRate(0U, ENV_HUMIDITY, rate) == BSP_ERROR_NONE) {
        ret = SENSOR_DRV_OK;
      }
      SensorUnLock();
      break;
    }
  }

  return ret;
}

/// Gets the humidity sensor's current data rate.
static float HumiditySensor_GetDataRate (void) {
  float rate = 0.0f;

  BSP_ENV_SENSOR_GetOutputDataRate(0U, ENV_HUMIDITY, &rate);
  return rate;
}

/// Sets the humidity sensor's measurement range (in measurement units).
static int32_t HumiditySensor_SetRange (float min, float max) {
  if ((min == HumiditySensor_Ranges.ranges[0].min) &&
      (max == HumiditySensor_Ranges.ranges[0].max)) {
    return SENSOR_DRV_OK;
  }
  return SENSOR_DRV_ERROR;
}

/// Gets the humidity sensor's current measurement range (in measurement units).
static int32_t HumiditySensor_GetRange (float *min, float *max) {
  if ((min == NULL) || (max == NULL)) {
    return SENSOR_DRV_ERROR;
  }

  *min = HumiditySensor_Ranges.ranges[0].min;
  *max = HumiditySensor_Ranges.ranges[0].max;

  return SENSOR_DRV_OK;
}

/// Returns the humidity sensor's offset (in measurement units).
static float HumiditySensor_GetOffset (void) {
  return 0.0f;
}

/// Returns the humidity sensor's resolution.
static float HumiditySensor_GetResolution (void) {
  return 1.0f;
}

/// Sets data event threshold in bytes for the humidity sensor.
static int32_t HumiditySensor_SetDataThreshold (uint32_t threshold) {
  (void)threshold;
  return SENSOR_DRV_UNSUPPORTED;
}

/// Returns the humidity sensor's current status.
static SensorDrv_Status_t HumiditySensor_GetStatus (void) {
  SensorDrv_Status_t status;

  status.active   = (HumiditySensor_Started != 0U) ? 1U : 0U;
  status.overflow = 0U; // Overflow detection not supported
  return status;
}

/// Reads data from the humidity sensor into the provided buffer.
static uint32_t HumiditySensor_Read (void *buf, uint32_t buf_size) {
  uint32_t num = 0U;
  int32_t  ret;
  uint8_t  stat;
  float    value;

  if ((buf == NULL) || (buf_size < sizeof(float))) {
    return 0U;
  }

  SensorLock();
  ret = HTS221_HUM_Get_DRDY_Status(Env_Sensor_CompObj[0], &stat);
  if ((ret == 0) && (stat != 0U)) {
    if (BSP_ENV_SENSOR_GetValue(0, ENV_HUMIDITY, &value) == BSP_ERROR_NONE) {
      memcpy(buf, &value, sizeof(float));
      num = (uint32_t)sizeof(float);
    }
  }
  SensorUnLock();

  return num;
}

//--------------------------------------------------------------------
// Pressure Sensor
//--------------------------------------------------------------------

/// Information about the pressure sensor.
static const SensorDrv_DataRates_t PressureSensor_Rates = {
  .num = 7,
  .rates = {1.0f, 10.0f, 25.0f, 50.0f, 75.0f, 100.0f, 200.0f}
};

static const SensorDrv_Ranges_t PressureSensor_Ranges = {
  .num = 1,
  .ranges = {{2.6f, 12.6f}} // Example range placeholder
};

static SensorDrv_Info_t PressureSensor_Info = {
  .name       = "Pressure",
  .channels   = 1,
  .data_type  = SENSOR_DRV_DATA_FLOAT,
  .unit       = "Pa",
  .data_rates = &PressureSensor_Rates,
  .ranges     = &PressureSensor_Ranges,
  .fifo_size  = 4,
};

/// Initializes the pressure sensor driver.
static int32_t PressureSensor_Initialize (SensorDrv_Event_t event_cb, uint32_t event_mask) {
  int32_t ret;

  if ((event_cb != NULL) || (event_mask != 0U)) {
    return SENSOR_DRV_UNSUPPORTED;
  }

  SensorLockCreate();
  SensorLock();
  if (BSP_ENV_SENSOR_Init(1U, ENV_PRESSURE) == BSP_ERROR_NONE) {
    PressureSensor_Initialized = 1U;
    ret = SENSOR_DRV_OK;
  } else {
    ret = SENSOR_DRV_ERROR;
  }
  SensorUnLock();

  return ret;
}

/// Un-initializes the pressure sensor driver.
static int32_t PressureSensor_Uninitialize (void) {
  int32_t ret;

  SensorLock();
  if (BSP_ENV_SENSOR_DeInit(0U) == BSP_ERROR_NONE) {
    PressureSensor_Started     = 0U;
    PressureSensor_Initialized = 0U;
    ret = SENSOR_DRV_OK;
  } else {
    ret = SENSOR_DRV_ERROR;
  }
  SensorUnLock();
  SensorLockDelete();

  return ret;
}

/// Gets pressure sensor information.
static SensorDrv_Info_t *PressureSensor_GetInfo (void) {
  return &PressureSensor_Info;
}

/// Starts capturing pressure sensor data.
static int32_t PressureSensor_CaptureStart (void) {
  int32_t ret = SENSOR_DRV_ERROR;

  SensorLock();
  if (BSP_ENV_SENSOR_Enable(0, ENV_PRESSURE) == BSP_ERROR_NONE) {
    PressureSensor_Started = 1U;
    ret = SENSOR_DRV_OK;
  }
  SensorUnLock();

  return ret;
}

/// Stops capturing pressure sensor data.
static int32_t PressureSensor_CaptureStop (void) {
  int32_t ret = SENSOR_DRV_ERROR;

  SensorLock();
  if (BSP_ENV_SENSOR_Disable(0, ENV_PRESSURE) == BSP_ERROR_NONE) {
    PressureSensor_Started = 0U;
    ret = SENSOR_DRV_OK;
  }
  SensorUnLock();

  return ret;
}

/// Sets the pressure sensor's data rate.
static int32_t PressureSensor_SetDataRate (float rate) {
  int32_t  ret = SENSOR_DRV_ERROR;
  uint32_t n;

  for (n = 0U; n < PressureSensor_Rates.num; n++) {
    if (rate == PressureSensor_Rates.rates[n]) {
      SensorLock();
      if (BSP_ENV_SENSOR_SetOutputDataRate(0U, ENV_PRESSURE, rate) == BSP_ERROR_NONE) {
        ret = SENSOR_DRV_OK;
      }
      SensorUnLock();
      break;
    }
  }

  return ret;
}

/// Gets the pressure sensor's current data rate.
static float PressureSensor_GetDataRate (void) {
  float rate = 0.0f;
  BSP_ENV_SENSOR_GetOutputDataRate(0U, ENV_PRESSURE, &rate);
  return rate;
}

/// Sets the pressure sensor's measurement range (in measurement units).
static int32_t PressureSensor_SetRange (float min, float max) {
  if ((min == PressureSensor_Ranges.ranges[0].min) &&
      (max == PressureSensor_Ranges.ranges[0].max)) {
    return SENSOR_DRV_OK;
  }
  return SENSOR_DRV_ERROR;
}

/// Gets the pressure sensor's current measurement range (in measurement units).
static int32_t PressureSensor_GetRange (float *min, float *max) {
  if ((min == NULL) || (max == NULL)) {
    return SENSOR_DRV_ERROR;
  }
  *min = PressureSensor_Ranges.ranges[0].min;
  *max = PressureSensor_Ranges.ranges[0].max;
  return SENSOR_DRV_OK;
}

/// Returns the pressure sensor's offset (in measurement units).
static float PressureSensor_GetOffset (void) {
  return 0.0f;
}

/// Returns the pressure sensor's resolution.
static float PressureSensor_GetResolution (void) {
  return 100.0f;
}

/// Sets data event threshold in bytes for the pressure sensor.
static int32_t PressureSensor_SetDataThreshold (uint32_t threshold) {
  (void)threshold;
  return SENSOR_DRV_UNSUPPORTED;
}

/// Returns the pressure sensor's current status.
static SensorDrv_Status_t PressureSensor_GetStatus (void) {
  SensorDrv_Status_t status;

  status.active   = (PressureSensor_Started != 0U) ? 1U : 0U;
  status.overflow = 0U; // Overflow detection not supported
  return status;
}

/// Reads data from the pressure sensor into the provided buffer.
static uint32_t PressureSensor_Read (void *buf, uint32_t buf_size) {
  uint32_t num = 0U;
  int32_t  ret;
  uint8_t  stat;
  float    value;

  if ((buf == NULL) || (buf_size < sizeof(float))) {
    return 0U;
  }

  SensorLock();
  ret = LPS22HH_PRESS_Get_DRDY_Status(Env_Sensor_CompObj[1], &stat);
  if ((ret == 0) && (stat != 0U)) {
    if (BSP_ENV_SENSOR_GetValue(0, ENV_PRESSURE, &value) == BSP_ERROR_NONE) {
      memcpy(buf, &value, sizeof(float));
      num = (uint32_t)sizeof(float);
    }
  }
  SensorUnLock();

  return num;
}

//--------------------------------------------------------------------
// Accelerometer
//--------------------------------------------------------------------

/// Information about the Accelerometer.
static const SensorDrv_DataRates_t Accelerometer_Rates = {
  .num = 10,
  .rates = {
    12.5f, 26.0f, 52.0f, 104.0f, 208.0f,
    416.0f, 833.0f, 1666.0f, 3333.0f, 6667.0f
  }
};

static const SensorDrv_Ranges_t Accelerometer_Ranges = {
  .num = 4,
  .ranges = { {-2.0f, 2.0f}, {-4.0f, 4.0f}, {-8.0f, 8.0f}, {-16.0f, 16.0f} }
};

static SensorDrv_Info_t Accelerometer_Info = {
  .name       = "Accelerometer",
  .channels   = 3,
  .data_type  = SENSOR_DRV_DATA_INT16,
  .unit       = "G",
  .data_rates = &Accelerometer_Rates,
  .ranges     = &Accelerometer_Ranges,
  .fifo_size  = 6,  // 3 channels * 2 bytes each
};

/// Initializes the accelerometer's driver.
static int32_t Accelerometer_Initialize (SensorDrv_Event_t event_cb, uint32_t event_mask) {
  int32_t ret;

  if ((event_cb != NULL) || (event_mask != 0U)) {
    return SENSOR_DRV_UNSUPPORTED;
  }

  SensorLockCreate();
  SensorLock();
  if (BSP_MOTION_SENSOR_Init(0U, MOTION_ACCELERO) == BSP_ERROR_NONE) {
    Accelerometer_Initialized = 1U;
    ret = SENSOR_DRV_OK;
  } else {
    ret = SENSOR_DRV_ERROR;
  }
  SensorUnLock();

  return ret;
}

/// Un-initializes the accelerometer driver.
static int32_t Accelerometer_Uninitialize (void) {
  int32_t ret;

  if (Gyroscope_Initialized == 0U) {
    SensorLock();
    if (BSP_ENV_SENSOR_DeInit(0U) == BSP_ERROR_NONE) {
      Accelerometer_Started      = 0U;
      Accelerometer__Initialized = 0U;
      ret = SENSOR_DRV_OK;
    } else {
      ret = SENSOR_DRV_ERROR;
    }
    SensorUnLock();
    SensorLockDelete();
  } else {
    Accelerometer_Started     = 0U;
    Accelerometer__Initialized = 0U;
    ret = SENSOR_DRV_OK;
  }

  return ret;
}

/// Gets accelerometer's information.
static SensorDrv_Info_t *Accelerometer_GetInfo (void) {
  return &Accelerometer_Info;
}

/// Starts capturing accelerometer's data.
static int32_t Accelerometer_CaptureStart (void) {
  uint32_t sample[2]
  int32_t ret = SENSOR_DRV_ERROR;

  SensorLock();
  if (ISM330DHCX_FIFO_Init(ISM330DHCX_ID_ACCELEROMETER) == 0) {
    if ((Accelerometer_Started == 0U ) && (Gyroscope_Started == 0U)) {
      // Clear ISM330DHCX FIFO
      while (ISM330DHCX_FIFO_Read(ISM330DHCX_ID_ACCELEROMETER, 1, sample) != 0U);
    }
    if (ISM330DHCX_ACC_Enable(&ISM330DHCX_Obj) == 0) {
      Accelerometer_Started = 1U;
      ret = SENSOR_OK;
    }
  }
  SensorUnLock();

  return ret;
}

/// Stops capturing accelerometer's data.
static int32_t Accelerometer_CaptureStop (void) {
  int32_t ret = SENSOR_DRV_ERROR;

  sensorLock();
  if (ISM330DHCX_ACC_Disable(&ISM330DHCX_Obj) == 0) {
    if (ISM330DHCX_FIFO_Uninit(ISM330DHCX_ID_ACCELEROMETER) == 0) {
      Accelerometer_Started = 0U;
      ret = SENSOR_OK;
    }
  }
  sensorUnLock();
  sensorLockDelete();

  return ret;
}

/// Sets the accelerometer's data rate.
static int32_t Accelerometer_SetDataRate (float rate) {
  int32_t  ret = SENSOR_DRV_ERROR;
  uint32_t n;

  for (n = 0U; n < Accelerometer_Rates.num; n++) {
    if (rate == Accelerometer_Rates.rates[n]) {
      SensorLock();
      if (BSP_MOTION_SENSOR_SetOutputDataRate(0U, MOTION_ACCELERO, rate) == BSP_ERROR_NONE) {
        ret = SENSOR_DRV_OK;
      }
      SensorUnLock();
      break;
    }
  }
  return ret;
}

/// Gets the accelerometer's current data rate.
static float Accelerometer_GetDataRate (void) {
  float rate = 0.0f;
  BSP_MOTION_SENSOR_GetOutputDataRate(0U, MOTION_ACCELERO, &rate);
  return rate;
}

/// Sets the accelerometer's measurement range.
static int32_t Accelerometer_SetRange (float min, float max) {
  int32_t  ret = SENSOR_DRV_ERROR;
  int32_t  FullScale;
  uint32_t n;

  for (n = 0U; n < Accelerometer_Rates.num; n++) {
    if ((min == Accelerometer_Ranges.ranges[0].min) &&
        (max == Accelerometer_Ranges.ranges[0].max)) {
      switch (max) {
        case 2.0f:
          FullScale = ISM330DHCX_2g;
          break;
        case 4.0f:
          FullScale = ISM330DHCX_4g;
          break;
        case 8.0f:
          FullScale = ISM330DHCX_8g;
          break;
        case 16.0f:
          FullScale = ISM330DHCX_16g;
          break;

      }
    }
    return SENSOR_DRV_OK;
  }
  return SENSOR_DRV_ERROR;
}

/// Gets the accelerometer's current measurement range.
static int32_t Accelerometer_GetRange (float *min, float *max) {
  if ((min == NULL) || (max == NULL)) {
    return SENSOR_DRV_ERROR;
  }
  *min = Accelerometer_Ranges.ranges[0].min;
  *max = Accelerometer_Ranges.ranges[0].max;
  return SENSOR_DRV_OK;
}

/// Returns the accelerometer's offset (in G).
static float Accelerometer_GetOffset (void) {
  return 0.0f;
}

/// Returns the accelerometer's resolution.
static float Accelerometer_GetResolution (void) {
  // If each LSB = 0.000061 g at ±2g, we could return 0.000061f,
  // but 1.0f here means "no scaling performed" at driver level.
  return 1.0f;
}

/// Sets data event threshold in bytes (not supported here).
static int32_t Accelerometer_SetDataThreshold (uint32_t threshold) {
  (void)threshold;
  return SENSOR_DRV_UNSUPPORTED;
}

/// Returns the accelerometer's current status.
static SensorDrv_Status_t Accelerometer_GetStatus (void) {
  SensorDrv_Status_t status;
  status.active   = (Accelerometer_Started != 0U) ? 1U : 0U;
  status.overflow = 0U;
  return status;
}

/// Reads data from the accelerometer into the provided buffer.
static uint32_t Accelerometer_Read (void *buf, uint32_t buf_size) {
  uint32_t num = 0U;
  BSP_MOTION_SENSOR_AxesRaw_t axes;
  int32_t  ret;

  // Expect 6 bytes: 3 channels (x,y,z) * 2 bytes each
  if ((buf == NULL) || (buf_size < 6U)) {
    return 0U;
  }

  SensorLock();
  ret = BSP_MOTION_SENSOR_GetAxesRaw(0U, MOTION_ACCELERO, &axes);
  if (ret == BSP_ERROR_NONE) {
    // If AxesRaw_t is int16 per axis: direct copy
    // If it's int32, cast or shift as needed.
    memcpy(buf, &axes, 6U);
    num = 6U;
  }
  SensorUnLock();

  return num;
}
