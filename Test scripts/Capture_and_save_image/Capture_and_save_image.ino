#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// ——— SD Card Pins ———
#define SD_CS     10
#define SD_MOSI   11
#define SD_MISO   13
#define SD_CLK    12

// ——— Camera Pins (DFRobot ESP32-S3 AI Camera OV3660) ———
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

// ——— Boot Button ———
#define BOOT_BUTTON_PIN    0  // ESP32 on-board BOOT button (active LOW)

int fileCounter = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // ——— Initialize SD card ———
  SPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("❌ SD card init failed!");
    while (true) delay(1000);
  }
  Serial.println("✅ SD card initialized");

  // ——— Configure camera ———
  camera_config_t config;
  // Pin assignments
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;

  // Camera settings
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size   = FRAMESIZE_VGA;

  // LEDC (XCLK) settings
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  // Frame buffers
  config.fb_count     = psramFound() ? 2 : 1;
  if (config.fb_count == 2) {
    Serial.println("PSRAM found: using 2 frame buffers");
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("❌ Camera init failed: 0x%x\n", err);
    while (true) delay(1000);
  }
  Serial.println("✅ Camera initialized");

  // ——— Prepare boot button ———
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

  // ——— Determine next free filename ———
  while (true) {
    char path[32];
    sprintf(path, "/photo%03d.bmp", fileCounter);
    if (!SD.exists(path)) break;
    fileCounter++;
  }
  Serial.printf("Next file: photo%03d.bmp\n", fileCounter);
}

void loop() {
  // ——— Wait for BOOT button press & release ———
  Serial.println("Waiting for BOOT button press...");
  while (digitalRead(BOOT_BUTTON_PIN) == HIGH) {
    delay(10);
  }
  delay(50);  // debounce
  while (digitalRead(BOOT_BUTTON_PIN) == LOW) {
    delay(10);
  }
  delay(50);
  Serial.println("Button pressed - capturing image");

  // ——— Capture frame ———
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("❌ Camera capture failed");
    delay(1000);
    return;
  }

  // ——— Convert to BMP ———
  uint8_t *bmp_buf = NULL;
  size_t bmp_len = 0;
  bool ok = frame2bmp(fb, &bmp_buf, &bmp_len);
  esp_camera_fb_return(fb);

  if (!ok || !bmp_buf) {
    Serial.println("❌ BMP conversion failed");
    if (bmp_buf) free(bmp_buf);
    delay(1000);
    return;
  }

  // ——— Save BMP to SD card ———
  char filename[32];
  sprintf(filename, "/photo%03d.bmp", fileCounter);
  File file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("❌ Failed to open file");
    free(bmp_buf);
    delay(1000);
    return;
  }
  file.write(bmp_buf, bmp_len);
  file.close();
  free(bmp_buf);

  Serial.printf("💾 Saved %s (%u bytes)\n", filename, (unsigned int)bmp_len);
  fileCounter++;

  // ——— After capture, wait a moment then loop again for next shot ———
  delay(500);
}
