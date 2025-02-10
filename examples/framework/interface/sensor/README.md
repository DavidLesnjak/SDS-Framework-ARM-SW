# **ToDo:**
# Sensor Driver Interface

The **Sensor Driver Interface** is designed for **ML** (Machine Learning) and **DSP** applications, providing a cohesive set of functions to configure, enable, and read sensor data, along with event-driven notifications. The **brief** descriptions below match those in the [`sensor_drv.h`](sensor_drv.h) header file.

---

## Overview

Typical usage sequence:

1. **Initialize** the driver with an optional event callback.
2. **Configure** data rate, measurement range, and thresholds.
3. **Start** capturing data from the sensor.
4. **Read** sensor data as needed.
5. **Stop** capturing data and **uninitialize** the driver when done.

Sensor details (such as supported data rates, measurement ranges, and FIFO size) are exposed through `SensorDrv_Info_t`, while events (e.g., data ready, overflow) are delivered through a callback function if registered.

---

## API Functions (Brief Descriptions)

| **Function**          | **Brief Description**                                                                                     |
|-----------------------|-----------------------------------------------------------------------------------------------------------|
| **Initialize**        | *Initializes the sensor driver and registers a callback function for sensor events.*                      |
| **Uninitialize**      | *Un-initializes the sensor driver.*                                                                       |
| **GetInfo**           | *Gets sensor information.*                                                                                |
| **CaptureStart**      | *Starts capturing sensor data.*                                                                           |
| **CaptureStop**       | *Stops capturing sensor data.*                                                                            |
| **SetDataRate**       | *Sets the sensor's data rate.*                                                                            |
| **GetDataRate**       | *Gets the sensor's current data rate.*                                                                    |
| **SetRange**          | *Sets sensor's measurement range (in measurement units).*                                                 |
| **GetRange**          | *Gets sensor's current measurement range (in measurement units).*                                         |
| **GetOffset**         | *Returns the sensor's offset (in measurement units).*                                                     |
| **GetResolution**     | *Returns sensor's resolution.*                                                                            |
| **SetDataThreshold**  | *Sets data event threshold in bytes.*                                                                     |
| **GetStatus**         | *Returns sensor's current status.*                                                                        |
| **Read**              | *Reads data from the sensor into the provided buffer.*                                                    |

For details on parameters, return values, and notes, consult the function prototypes and documentation comments in [`sensor_drv.h`](include/sensor_drv.h).
