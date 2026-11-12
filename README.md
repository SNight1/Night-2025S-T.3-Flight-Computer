# Night-2025S-T.3-Flight-Computer

This project is a prototype rocket to validate altitude telemetry logging and flight state detection

*Mission* is make sure to reliable a computer flight

## Hardware
* **ESP32**
* **ESP32-CAM**
* **BMP280**
* **MPU6050**
* **NEO-6M**

## Problems & Solutions

### BMP280
*Problems*  The sensor module was a rebranded/clone version with a non-standard Chip ID, causing standard libraries (such as Adafruit BMP280) to fail initialization.
*Solution* Bypassed external libraries and wrote direct I2C register read/write routines (`Wire.h`) to read factory calibration parameters (`0x88`–`0xA1`) and raw 20-bit pressure/temperature data (`0xF7`–`0xFC`) directly from the chip.

### ESP32-CAM
*Problems* At the default `20MHz` `XCLK` frequency, the OV2640 sensor generated frames faster than the    ESP32 could read or save them, overflowing the DMA buffer and freezing captures.
*Solution* Lowered `xclk_freq_hz` to `10MHz`, set `grab_mode = CAMERA_GRAB_LATEST` to drop old frames,    and stored frame buffers in PSRAM (`CAMERA_FB_IN_PSRAM`).
