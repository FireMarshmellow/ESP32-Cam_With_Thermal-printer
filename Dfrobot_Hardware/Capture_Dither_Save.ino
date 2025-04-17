#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

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

#define BOOT_BUTTON_PIN  0  // Active LOW

int fileCounter = 0;

void waitForButtonPress() {
  Serial.println("Waiting for BOOT button...");
  while (digitalRead(BOOT_BUTTON_PIN) == HIGH) {
    delay(10);
  }
  Serial.println("Button pressed!");
  delay(100); // Debounce
}

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

// This version of processBMPtoHeader streams the output directly to the SD card
// without allocating a full mono buffer. It dither-processes one row at a time
// and writes bytes to the .h file incrementally.

bool processBMPtoHeader(const String& bmpFilename) {
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

  int32_t width  = *(int32_t*)&header[18];
  int32_t height_raw = *(int32_t*)&header[22];
  bool isTopDown = height_raw < 0;
  int32_t height = abs(height_raw);
  uint16_t bpp   = *(uint16_t*)&header[28];
  uint32_t offset = *(uint32_t*)&header[10];

  if (bpp != 24) {
    Serial.println("Only 24-bit BMP supported");
    bmp.close();
    return false;
  }

  int rowSize = (width * 3 + 3) & ~3;
  bmp.seek(offset);

  float* rowA = (float*)ps_malloc(width * sizeof(float));
  float* rowB = (float*)ps_malloc(width * sizeof(float));
  if (!rowA || !rowB) {
    Serial.println("Failed to allocate dither rows");
    bmp.close();
    free(rowA); free(rowB);
    return false;
  }

  // Preload first row into rowB
  int preloadY = isTopDown ? 0 : height - 1;
  bmp.seek(offset + rowSize * preloadY);
  for (int x = 0; x < width; x++) {
    uint8_t b = bmp.read();
    uint8_t g = bmp.read();
    uint8_t r = bmp.read();
    rowB[x] = 0.299f * r + 0.587f * g + 0.114f * b;
  }

  char headerName[32];
  snprintf(headerName, sizeof(headerName), "/capture_%03d.h", fileCounter);
  File headerFile = SD.open(headerName, FILE_WRITE);
  if (!headerFile) {
    Serial.println("Failed to open .h file");
    free(rowA); free(rowB);
    bmp.close();
    return false;
  }

  headerFile.printf("// %d x %d dithered image\n", width, height);
  headerFile.printf("const uint16_t image_width = %d;\n", width);
  headerFile.printf("const uint16_t image_height = %d;\n", height);
  headerFile.printf("const uint8_t image_data[] PROGMEM = {\n");

  size_t byteCount = 0;

  for (int i = 0; i < height; i++) {
    int y = isTopDown ? i : (height - 1 - i);

    // Move rowB to rowA
    memcpy(rowA, rowB, width * sizeof(float));

    // Load next row into rowB
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
      memset(rowB, 255, width * sizeof(float)); // last row padding
    }

    // Dither and emit
    uint8_t outByte = 0;
    int bit = 7;

    for (int x = 0; x < width; x++) {
      float oldPixel = rowA[x];
      float newPixel = oldPixel < 128 ? 0 : 255;
      float error = oldPixel - newPixel;
      rowA[x] = newPixel;

      if (x + 1 < width) rowA[x + 1] += error * 7 / 16.0;
      if (i < height - 1) {
        if (x > 0)         rowB[x - 1] += error * 3 / 16.0;
                          rowB[x    ] += error * 5 / 16.0;
        if (x + 1 < width) rowB[x + 1] += error * 1 / 16.0;
      }

      if (newPixel < 128) outByte |= (1 << bit);
      bit--;

      if (bit < 0 || x == width - 1) {
        if (byteCount % 12 == 0) headerFile.print("  ");
        headerFile.printf("0x%02X", outByte);
        byteCount++;
        if (i != height - 1 || x != width - 1) headerFile.print(", ");
        if (byteCount % 12 == 0) headerFile.println();
        outByte = 0;
        bit = 7;
      }
    }
  }

  headerFile.println("\n};");
  headerFile.close();
  free(rowA);
  free(rowB);
  bmp.close();

  Serial.printf("Saved streamed %s\n", headerName);
  return true;
}


void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

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
  config.frame_size = FRAMESIZE_CIF; // 400x296
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

  SPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card init failed!");
    return;
  }

  Serial.println("Setup complete.");
}

void loop() {
  waitForButtonPress();
  String bmpPath = saveBMPtoSD();
  if (bmpPath != "") {
    processBMPtoHeader(bmpPath);
    fileCounter++;
  }
  delay(1000);
}