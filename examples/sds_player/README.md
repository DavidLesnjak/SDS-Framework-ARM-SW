# SDS Player Example

This project shows how to use **Synchronous Data Stream Player** via Socket (IoT Socket). The application uses SDS Player to read pre-recorded accelerometer data, which is processed on the fly to determine if any movement is detected.

Available targets:
 - B-U585I-IOT02A: runs on STMicroelectronics B-U585I-IOT02A board
 - AVH_MPS3_Corstone-300: runs on Arm Virtual Hardware (AVH) for MPS3 platform with Corstone-300

## Prerequisites

### Hardware:
 - [B-U585I-IOT02A Discovery kit](https://www.st.com/en/evaluation-tools/b-u585i-iot02a.html) from STMicroelectronics

### Tools:
 - [CMSIS-Toolbox 2.0.0](https://github.com/Open-CMSIS-Pack/cmsis-toolbox/releases/) or later
 - Arm Compiler 6.18 or later
 - [python 3.9 or later](https://www.python.org/downloads/windows/) when using Arm Virtual Hardware (AVH)

 ### Configure:

Host PC IP Address:
 - Modify the following definitions in [sdsio_config_socket.h](./RTE/SDS/sdsio_config_socket.h):
   - `SERVER_IP`: IP address of the host PC running python script (`SDSIO-Server.py`)

WiFi Access Point:
 - Modify the following definitions in [socket_startup.c](../framework/layer/Socket/WiFi/socket_startup.c):
   - `SSID`:          WiFi Access Point SSID
   - `PASSWORD`:      WiFi Access Point Password
   - `SECURITY_TYPE`: WiFi Access Point Security

## Compile Project

The following commands convert and build the project with build type `Debug` and target type `B-U585I-IOT02A`:

```sh
cbuild SDS_Buffer.csolution.yml --update-rte -p --configuration .Debug+B-U585I-IOT02A
```

## Execute

### B-U585I-IOT02A target
 - connect the board's ST-Link USB to the PC (provides also power)
 - use a programmer to download the HW image to the hardware target
 - run [SDSIO-Server](../utilities/SDSIO-Server/README.md) to start the SDS I/O server on the host PC
 - open terminal on the PC and connect to the board's serial port (Baud rate: 115200)
 - reset the target (press RST button on the board)
 - wait until connected to WiFi (status printed to the terminal)
- reset the target (press RST button on the board)
- monitor results (if motion detected) in terminal window

### AVH_MPS3_Corstone-300 target

- copy recorded accelerometer files (Accelerometer.x.sds) from [recordings folder](../recordings) to [SDSIO-Server Folder](../../utilities/SDSIO-Server/)
- from the project folder execute following command to run the example on the VHT simulation model:

```sh
VHT_MPS3_Corstone_SSE-300 -f ../framework/layer/Board/AVH_MPS3_Corstone-300/fvp_config.txt -V ../framework/interface/VSI/sensor/python out/SDS_Buffer/AVH_MPS3_Corstone-300/Debug/SDS_Buffer.axf
```
 - run [SDSIO-Server](../../utilities/SDSIO-Server/README.md) to start the SDS I/O server on the host PC
- monitor results (if motion detected) in terminal window
