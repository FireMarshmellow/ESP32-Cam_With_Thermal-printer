#include "Adafruit_Thermal.h"
#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// ------------------ Printer Setup ------------------
#define TX_PIN 43  // ESP TX → Printer RX 
#define RX_PIN 44  // ESP RX → Printer TX 

HardwareSerial mySerial(1);  // Use HardwareSerial instance 1
Adafruit_Thermal printer(&mySerial);

// ------------------ Camera & SD Setup ------------------
// CAMERA PINS (DFRobot ESP32-S3 AI Camera OV3660)
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      5
#define SIOD_GPIO_NUM      8
#define SIOC_GPIO_NUM      9
#define Y9_GPIO_NUM        4
#define Y8_GPIO_NUM        6
#define Y7_GPIO_NUM        7
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       17
#define Y4_GPIO_NUM       21
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM       16
#define VSYNC_GPIO_NUM     1
#define HREF_GPIO_NUM      2
#define PCLK_GPIO_NUM     15

// SD Card Pins
#define SD_CS     10
#define SD_MOSI   11
#define SD_MISO   13
#define SD_CLK    12

// Button to trigger capture
#define BOOT_BUTTON_PIN  0  // Active LOW

int fileCounter = 0;

// ------------------ Function Prototypes ------------------
void waitForButtonPress();
String saveBMPtoSD();
bool printBMP(const String &bmpFilename);
void printBitmapRaw(const uint8_t *bitmap, uint16_t width, uint16_t height);
void flipBitmapVertically(uint8_t *bitmap, uint16_t height, uint16_t rowBytes);

// ------------------ Flip Bitmap Vertically ------------------
// This function swaps rows in the bitmap buffer so that the image is flipped vertically.
void flipBitmapVertically(uint8_t *bitmap, uint16_t height, uint16_t rowBytes) {
  uint8_t *temp = (uint8_t *) malloc(rowBytes);
  if (!temp) {
    Serial.println("Failed to allocate temp buffer for flipping");
    return;
  }
  for (uint16_t i = 0; i < height / 2; i++) {
    memcpy(temp, bitmap + i * rowBytes, rowBytes);
    memcpy(bitmap + i * rowBytes, bitmap + (height - 1 - i) * rowBytes, rowBytes);
    memcpy(bitmap + (height - 1 - i) * rowBytes, temp, rowBytes);
  }
  free(temp);
}

// ------------------ Setup ------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize hardware serial for printer
  mySerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  printer.begin();

  // Configure button pin
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

  // Camera configuration
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_CIF; // e.g., 400x296
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    Serial.println("PSRAM detected. Using 2 frame buffers.");
    config.fb_count = 2;
  } else {
    Serial.println("No PSRAM detected.");
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return;
  }

  // Initialize SD card
  SPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card init failed!");
    return;
  }

  Serial.println("Setup complete.");
}

// ------------------ Main Loop ------------------
void loop() {
  waitForButtonPress();

  // Capture image and save as BMP to SD card
  String bmpPath = saveBMPtoSD();
  if (bmpPath != "") {
    // Process the BMP and print it directly
    if (printBMP(bmpPath)) {
      Serial.println("Printing complete.");
    } else {
      Serial.println("Printing failed.");
    }
    fileCounter++;
  }
  delay(1000);
}

// ------------------ Wait for Button Press ------------------
void waitForButtonPress() {
  Serial.println("Waiting for BOOT button...");
  while (digitalRead(BOOT_BUTTON_PIN) == HIGH) {
    delay(10);
  }
  Serial.println("Button pressed!");
  delay(100); // debounce
}

// ------------------ Save BMP to SD ------------------
String saveBMPtoSD() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return "";
  }

  uint8_t* bmp_buf = NULL;
  size_t bmp_len = 0;
  bool success = frame2bmp(fb, &bmp_buf, &bmp_len);
  esp_camera_fb_return(fb);

  if (!success || !bmp_buf) {
    Serial.println("BMP conversion failed");
    return "";
  }

  char filename[32];
  snprintf(filename, sizeof(filename), "/capture_%03d.bmp", fileCounter);
  File file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open BMP file");
    free(bmp_buf);
    return "";
  }

  file.write(bmp_buf, bmp_len);
  file.close();
  free(bmp_buf);

  Serial.printf("Saved %s\n", filename);
  return String(filename);
}

// ------------------ Print BMP from SD ------------------
bool printBMP(const String &bmpFilename) {
  File bmp = SD.open(bmpFilename);
  if (!bmp) {
    Serial.println("Failed to open BMP file for reading");
    return false;
  }

  uint8_t header[54];
  if (bmp.read(header, 54) != 54) {
    Serial.println("Failed to read BMP header");
    bmp.close();
    return false;
  }

  int32_t width = *(int32_t*)&header[18];
  int32_t height_raw = *(int32_t*)&header[22];
  bool isTopDown = (height_raw < 0);
  int32_t height = abs(height_raw);
  uint16_t bpp = *(uint16_t*)&header[28];
  uint32_t offset = *(uint32_t*)&header[10];

  if (bpp != 24) {
    Serial.println("Only 24-bit BMP supported");
    bmp.close();
    return false;
  }

  int rowSize = (width * 3 + 3) & ~3;
  bmp.seek(offset);

  // Allocate row buffers for dithering
  float* rowA = (float*) ps_malloc(width * sizeof(float));
  float* rowB = (float*) ps_malloc(width * sizeof(float));
  if (!rowA || !rowB) {
    Serial.println("Failed to allocate dither rows");
    bmp.close();
    if (rowA) free(rowA);
    if (rowB) free(rowB);
    return false;
  }

  // Preload first row into rowB (depending on image orientation)
  int preloadY = isTopDown ? 0 : height - 1;
  bmp.seek(offset + rowSize * preloadY);
  for (int x = 0; x < width; x++) {
    uint8_t b = bmp.read();
    uint8_t g = bmp.read();
    uint8_t r = bmp.read();
    rowB[x] = 0.299f * r + 0.587f * g + 0.114f * b;
  }

  // Calculate number of bytes per row for the 1-bit bitmap
  uint16_t widthBytes = (width + 7) / 8;
  uint8_t* ditheredBitmap = (uint8_t*) malloc(height * widthBytes);
  if (!ditheredBitmap) {
    Serial.println("Failed to allocate dithered bitmap buffer");
    bmp.close();
    free(rowA);
    free(rowB);
    return false;
  }
  size_t byteIndex = 0;

  // Process each row
  for (int i = 0; i < height; i++) {
    // For bottom-up BMP files, calculate correct row order
    int y = isTopDown ? i : (height - 1 - i);

    // Copy current row (rowB) into rowA for processing
    memcpy(rowA, rowB, width * sizeof(float));

    // Load next row into rowB if available
    if (i < height - 1) {
      int nextY = isTopDown ? (i + 1) : (height - 2 - i);
      bmp.seek(offset + rowSize * nextY);
      for (int x = 0; x < width; x++) {
        uint8_t b = bmp.read();
        uint8_t g = bmp.read();
        uint8_t r = bmp.read();
        rowB[x] = 0.299f * r + 0.587f * g + 0.114f * b;
      }
    } else {
      // Last row padding
      memset(rowB, 255, width * sizeof(float));
    }

    uint8_t outByte = 0;
    int bit = 7;

    // Dither this row and pack 8 pixels per output byte
    for (int x = 0; x < width; x++) {
      float oldPixel = rowA[x];
      float newPixel = oldPixel < 128 ? 0 : 255;
      float error = oldPixel - newPixel;
      rowA[x] = newPixel;

      if (x + 1 < width) rowA[x + 1] += error * 7.0f / 16.0f;
      if (i < height - 1) {
        if (x > 0)         rowB[x - 1] += error * 3.0f / 16.0f;
                           rowB[x    ] += error * 5.0f / 16.0f;
        if (x + 1 < width) rowB[x + 1] += error * 1.0f / 16.0f;
      }

      if (newPixel < 128) outByte |= (1 << bit);
      bit--;

      if (bit < 0 || x == width - 1) {
        ditheredBitmap[byteIndex++] = outByte;
        outByte = 0;
        bit = 7;
      }
    }
  }

  // Flip the dithered bitmap vertically before printing
  flipBitmapVertically(ditheredBitmap, height, widthBytes);

  // Now print the dithered bitmap via the thermal printer
  printer.println("Printing bitmap (raw)...");
  printBitmapRaw(ditheredBitmap, width, height);
  printer.feed(2);
  printer.println("Bitmap printed!");
  printer.feed(2);

  // Clean up
  free(ditheredBitmap);
  free(rowA);
  free(rowB);
  bmp.close();
  return true;
}

// ------------------ Print Bitmap Raw ------------------
void printBitmapRaw(const uint8_t *bitmap, uint16_t width, uint16_t height) {
  uint16_t widthBytes = (width + 7) / 8;
  uint8_t xL = widthBytes & 0xFF;
  uint8_t xH = (widthBytes >> 8) & 0xFF;
  uint8_t yL = height & 0xFF;
  uint8_t yH = (height >> 8) & 0xFF;

  // ESC/POS raster bitmap command with explicit casts
  mySerial.write((uint8_t)0x1D);
  mySerial.write((uint8_t)0x76);
  mySerial.write((uint8_t)0x30);
  mySerial.write((uint8_t)0x00);
  mySerial.write((uint8_t)xL);
  mySerial.write((uint8_t)xH);
  mySerial.write((uint8_t)yL);
  mySerial.write((uint8_t)yH);

  for (uint16_t i = 0; i < height * widthBytes; i++) {
    mySerial.write(pgm_read_byte(bitmap + i));
  }
}
