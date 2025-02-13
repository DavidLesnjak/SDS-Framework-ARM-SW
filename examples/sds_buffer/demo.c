/*
 * Copyright (c) 2023 Arm Limited. All rights reserved.
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

#include <stdio.h>
#include <math.h>

#include "main.h"
#include "cmsis_os2.h"

#include "sds.h"
#include "sensor_drv.h"
// #include "sensor_config.h"

// Configuration
#ifndef SDS_BUF_SIZE_ACCELEROMETER
#define SDS_BUF_SIZE_ACCELEROMETER      8192U
#endif

#ifndef SDS_THRESHOLD_ACCELEROMETER
#define SDS_THRESHOLD_ACCELEROMETER     2048U
#endif

#ifndef SENSOR_POLLING_INTERVAL
#define SENSOR_POLLING_INTERVAL         100U  // 100ms
#endif

#ifndef SENSOR_BUF_SIZE
#define SENSOR_BUF_SIZE                 8192U
#endif

#define ACCELEROMETER_FULL_SCALE        2     // Accelerometer full scale configuration: +-2G
#define MOTION_DETECTION_SENSITIVITY_mG 50

#define ACCELEROMETER_ABSOLUTE_LIMIT    (((double)INT16_MAX / (double)ACCELEROMETER_FULL_SCALE) * (1.0 + ((double)MOTION_DETECTION_SENSITIVITY_mG / 1000.0)))

// Accelerometer
extern SensorDrv_t SensorDrv_Accelerometer;
static SensorDrv_t *pAcc = &SensorDrv_Accelerometer;
static SensorDrv_Info_t *pAcc_info;

// SDS identifier
static sdsId_t sdsId_accelerometer;

// SDS buffer
static uint8_t sdsBuf_accelerometer[SDS_BUF_SIZE_ACCELEROMETER];

// Temporary sensor buffer
static uint8_t sensorBuf[SENSOR_BUF_SIZE];

// Thread identifiers
static osThreadId_t thrId_demo;

#define EVENT_DATA_ACCELEROMETER        (1U << 0)

// Read sensor thread
static __NO_RETURN void read_sensors (void *argument) {
  uint32_t num, buf_size, tick;
  (void)   argument;

  tick = osKernelGetTickCount();
  for (;;) {
    num = pAcc->Read(sensorBuf, sizeof(sensorBuf));
    if (num != 0U) {
      buf_size = num;
      num = sdsWrite(sdsId_accelerometer, sensorBuf, buf_size);
      if (num != buf_size) {
        printf("%s: SDS write failed\r\n", pAcc_info->name);
      }
    }
    tick += SENSOR_POLLING_INTERVAL;
    osDelayUntil(tick);
  }
}

// SDS event callback
static void sds_event_callback (sdsId_t id, uint32_t event, void *arg) {
  (void)arg;

  if ((event & SDS_EVENT_DATA_HIGH) != 0U) {
    if (id == sdsId_accelerometer) {
      osThreadFlagsSet(thrId_demo, EVENT_DATA_ACCELEROMETER);
    }
  }
}

// Sensor Demo
static __NO_RETURN void demo (void *argument) {
  uint32_t  n, num, sample_size, flags;
  uint32_t  alloc_buf[2];
  int16_t  *buf;
  double    abs;
  double    abs_max;

  (void)argument;

  buf = (int16_t *)alloc_buf;

  thrId_demo = osThreadGetId();

  // Initialize Accelerometer
  pAcc->Initialize(NULL, 0);

  // Get sensor configuration
  pAcc_info = pAcc->GetInfo();

  // Open SDS
  sdsId_accelerometer = sdsOpen(sdsBuf_accelerometer,
                                sizeof(sdsBuf_accelerometer),
                                0U, SDS_THRESHOLD_ACCELEROMETER);

  // Register SDS events
  sdsRegisterEvents(sdsId_accelerometer, sds_event_callback, SDS_EVENT_DATA_HIGH, NULL);

  // Accelerometer capture start
  pAcc->CaptureStart();
  printf("Accelerometer capture started\r\n");

  // Create sensor thread
  osThreadNew(read_sensors, NULL, NULL);

  // Sample size
  sample_size = SENSOR_DRV_DATA_SIZE(pAcc_info->data_type) * pAcc_info->channels;

  for(;;) {
    flags = osThreadFlagsWait(EVENT_DATA_ACCELEROMETER, osFlagsWaitAny, osWaitForever);
    if ((flags & osFlagsError) == 0U) {

      // Accelerometer data event
      if ((flags & EVENT_DATA_ACCELEROMETER) != 0U) {
        abs_max = 0.0;
        for (n = SDS_THRESHOLD_ACCELEROMETER / sample_size; n != 0U; n--) {
          num = sdsRead(sdsId_accelerometer, buf, sample_size);
          if (num == sample_size) {
            // Calculate absolute value of accelerometer vector
            abs = sqrt((buf[0] * buf[0]) + (buf[1] * buf[1]) + (buf[2] * buf[2]));

            if (abs > abs_max) {
              // Save max value
              abs_max = abs;
            }
          }
        }
        if (abs_max > ACCELEROMETER_ABSOLUTE_LIMIT) {
          printf("Motion detected!\r\n");
        } else {
          printf("Motion not detected!\r\n");
        }
      }
    }
  }
}

// Application initialization
int32_t app_initialize (void) {
  osThreadNew(demo, NULL, NULL);
  return 0;
}
