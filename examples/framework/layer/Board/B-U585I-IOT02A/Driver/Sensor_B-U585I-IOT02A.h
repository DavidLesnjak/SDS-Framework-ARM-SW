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

#ifndef SENSOR_B_U585I_IOT02A_H_
#define SENSOR_B_U585I_IOT02A_H_

#include "sensor_drv.h"

#ifdef  __cplusplus
extern  "C"
{
#endif


// ToDo: ...
// <<< Use Configuration Wizard in Context Menu >>>
// <e> Accelerometer
#define SENSOR_ACCELEROMETER        1
// <o> Accelerometer Full Scale <0=>2g <2=>4g <3=>8g <1=>16g
#define SENSOR_ACCELEROMETER_FS     1
// <o> Accelerometer Output Data Rate <12.5=>12.5Hz <26=>26Hz <52=>52Hz <104=>104Hz <208=>208Hz <416=>416Hz <833=>833Hz <1666=>1666Hz <3332=>3332Hz <6664=>6664Hz
#define SENSOR_ACCELEROMETER_ODR
// </e> Accelerometer

// <q> Enable Gyroscope Sensor
#define SENSOR_GYROSCOPE         125
// <o> Gyroscope Full Scale <0=>125dps <1=>250dps <2=>500dps <3=>1000dps <4=>2000dps <5=>4000dps
#define SENSOR_GYROSCOPE_FS       ISM330DHCX_2000dps
// <o> Gyroscope Output Data Rate <12.5=>12.5Hz <26=>26Hz <52=>52Hz <104=>104Hz <208=>208Hz <416=>416Hz <833=>833Hz <1666=>1666Hz <3332=>3332Hz <6664=>6664Hz
#define SENSOR_GYROSCOPE_ODR      1.0f

// <q> Enable Magnetometer Sensor
#define SENSOR_MAGNETOMETER      1
// <o> Magnetometer Output Data Rate <10=>10Hz <20=>20Hz <50=>50Hz <100=>100Hz
#define SENSOR_MAGNETOMETER_ODR   50.0f

// <q> Enable Pressure Sensor
#define SENSOR_PRESSURE          1
// <o> Pressure Sensor Output Data Rate <1=>1Hz <10=>10Hz <25=>25Hz <40=>40Hz <50=>50Hz <75=>75Hz
#define SENSOR_PRESSURE_ODR       40.0f

// <q> Enable Temperature Sensor
#define SENSOR_TEMPERATURE       1
// <o> Temperature Sensor Output Data Rate <1=>1Hz <10=>10Hz
#define SENSOR_TEMPERATURE_ODR    1.0f

// <q> Enable Humidity Sensor
#define SENSOR_HUMIDITY          1
// <o> Humidity Sensor Output Data Rate <1=>1Hz <10=>10Hz
#define SENSOR_HUMIDITY_ODR       1.0f
// <<< end of configuration section >>>




#endif  // SENSOR_B_U585I_IOT02A_H_


