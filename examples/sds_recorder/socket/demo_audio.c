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

#include "main.h"
#include "cmsis_vio.h"
#include "cmsis_os2.h"

#include "sds_rec.h"

#include "audio_drv.h"


// Configuration
#ifndef REC_BUF_SIZE
#define REC_BUF_SIZE            65536U
#endif
#ifndef REC_IO_THRESHOLD
#define REC_IO_THRESHOLD        4096
#endif
#ifndef MICROPHONE_BLOCK_NUM
#define MICROPHONE_BLOCK_NUM    32U      // must be 2^n 
#endif
#ifndef MICROPHONE_BLOCK_SIZE
#define MICROPHONE_BLOCK_SIZE   4096
#endif

#define MICROPHONE_CHANNELS     1U
#define MICROPHONE_DATA_BITS    16U
#define MICROPHONE_SAMPLE_RATE  8000U

// Recorder identifier
static sdsRecId_t recId = NULL;

// Recorder buffer
static uint8_t rec_buf[REC_BUF_SIZE];

// Microphone buffer
static uint8_t microphone_buf[MICROPHONE_BLOCK_NUM][MICROPHONE_BLOCK_SIZE];

// Microphone block count
static uint32_t microphone_block_count = 0U;

// Thread identifiers
static osThreadId_t thrId_microphone;

#define EVENT_DATA_MICROPHONE   (1U << 0)

// External functions
extern int32_t socket_startup (void);

// Microphone thread
static __NO_RETURN void microphone (void *argument) {
  uint32_t count, index, num;
  uint32_t tick, flags;

  (void) argument;

  for (;;) {
    flags = osThreadFlagsWait(EVENT_DATA_MICROPHONE, osFlagsWaitAny, osWaitForever);
    if ((flags & osFlagsError) == 0U) {
      tick = osKernelGetTickCount();
      count = AudioDrv_GetRxCount();
      if ((count - microphone_block_count) >= MICROPHONE_BLOCK_NUM) {
        printf("Microphone overflow\r\n");
      }
      while (microphone_block_count <= count) {
        index = microphone_block_count & (MICROPHONE_BLOCK_NUM - 1);
        num = sdsRecWrite(recId, tick, microphone_buf[index], MICROPHONE_BLOCK_SIZE);
        microphone_block_count++;
        if (num != MICROPHONE_BLOCK_SIZE) {
          printf("Recorder write failed\r\n");
        }
      }
    }
  }
}

// Microphone event callback 
static void microphone_event_callback (uint32_t event) {
  if ((event & AUDIO_DRV_EVENT_RX_DATA) != 0U) {
    osThreadFlagsSet(thrId_microphone, EVENT_DATA_MICROPHONE);
  }
}

// Recorder event callback
static void recorder_event_callback (sdsRecId_t id, uint32_t event) {
  if (event & SDS_REC_EVENT_IO_ERROR) {
    if (id == recId) {
      printf("Recorder event - I/O error\r\n");
    }
  }
}

// Recorder start
static void recorder_start (void) {
  // Open Recorder
  recId = sdsRecOpen("Microphone",
                      rec_buf,
                      sizeof(rec_buf),
                      REC_IO_THRESHOLD);

  osDelay(1000);
  // Microphone enable
  AudioDrv_Control(AUDIO_DRV_CONTROL_RX_ENABLE);
  printf("Microphone enabled\r\n");
}

// Recorder stop
static void recorder_stop (void) {
  // Microphone disable
  AudioDrv_Control(AUDIO_DRV_CONTROL_RX_DISABLE);

  // Close Recorder
  sdsRecClose(recId);
  recId = NULL;
  printf("Microphone disabled\r\n");
}

// Demo
static __NO_RETURN void demo (void *argument) {
  uint32_t  value;
  uint32_t  value_last = 0U;
  uint8_t   rec_active = 0U;

  (void) argument;

  if (socket_startup() != 0) {
    printf("Socket startup failed\r\n");
    osThreadExit();
  }

  // Initialize and configure microphone
  AudioDrv_Initialize(microphone_event_callback);
  AudioDrv_Configure(AUDIO_DRV_INTERFACE_RX,
                     MICROPHONE_CHANNELS,
                     MICROPHONE_DATA_BITS,
                     MICROPHONE_SAMPLE_RATE);
  AudioDrv_SetBuf(AUDIO_DRV_INTERFACE_RX,
                  microphone_buf,
                  MICROPHONE_BLOCK_NUM,
                  MICROPHONE_BLOCK_SIZE);

  // Initialize recorder
  sdsRecInit(recorder_event_callback);

  // Create microphone thread
  thrId_microphone = osThreadNew(microphone, NULL, NULL);

  for(;;) {
    // Monitor user button
    value = vioGetSignal(vioBUTTON0);
    if (value != value_last) {
      value_last = value;
      if (value == vioBUTTON0) {
        // Button pressed
        if (rec_active == 0U) {
          rec_active = 1U;
          recorder_start();
        } else {
          rec_active = 0U;
          recorder_stop();
        }
      }
    }
    osDelay(100U);
  }
}

// Application initialization
int32_t app_initialize (void) {
  osThreadNew(demo, NULL, NULL);
  return 0;
}
