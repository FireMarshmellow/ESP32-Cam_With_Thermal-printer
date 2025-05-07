# Capture\_Dither\_Save\_Print\_V3

[🎥 Watch a demo on YouTube](https://www.youtube.com/watch?v=)

## Overview

`Capture_Dither_Save_Print_V3` is an Arduino sketch designed for the ESP32-CAM to:

* Capture an image from the camera module
* Convert the image to a black-and-white dithered format suitable for thermal printing
* Save the processed image (optional on SD card)
* Send the dithered image to a thermal printer for immediate physical output

This repository contains all the code and instructions needed to get your ESP32-CAM and DFRobot thermal printer up and running.

## Hardware Requirements

* **ESP32-CAM module** (with OPI PSRAM) from DFRobot

  * Product link: [https://www.dfrobot.com/product-2899.html?marketing=67c824f1362b5](https://www.dfrobot.com/product-2899.html?marketing=67c824f1362b5)
* **DFRobot Thermal Printer** (V2/TTL) from DFRobot

  * Product link: [https://www.dfrobot.com/product-1799.html](https://www.dfrobot.com/product-1799.html)


## Installation

1. **Clone this repository**

   ```bash
   git clone https://github.com/yourusername/your-repo.git
   cd your-repo
   ```
2. **Open the sketch**

   * Launch Arduino IDE and open `Capture_Dither_Save_Print_V3.ino` from this folder.
3. **Select board and port**

   * **Board**: Select **AI Thinker ESP32-CAM** (or the matching ESP32-CAM variant).
   * **PSRAM**: **Enabled** (required for image buffering)
   * **Upload Speed**: 115200
   * **Port**: Select the COM port for your USB-to-Serial adapter

## Usage

1. **Compile & Upload**

   * Click the **Upload** button in Arduino IDE.
2. **Monitor Serial Output**

   * Open the Serial Monitor at **115200 baud** to view status messages.
3. **Capture & Print**

   * After boot, the ESP32-CAM will automatically capture an image, apply Floyd–Steinberg dithering, save it to the SD card (if available), and send it to the thermal printer.
   * Wait a few seconds for the printer to finish outputting the image.

## Troubleshooting

* **Blank Prints**: Ensure PSRAM is enabled and sufficient power is supplied to the printer.
* **Compilation Errors**: Verify you have the correct ESP32 board package and PSRAM setting.
* **SD Card Issues**: Make sure card is formatted as FAT32 and wiring matches your pin definitions.

## Acknowledgments

* Based on example code from the [`esp32-camera`](https://github.com/espressif/esp32-camera) library
* DFRobot for hardware modules and documentation
