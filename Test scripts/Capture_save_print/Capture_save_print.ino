#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "Adafruit_Thermal.h"

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

// ——— Thermal Printer Pins & Setup ———
#define TX_PIN 43
#define RX_PIN 44
HardwareSerial printerSerial(1);
Adafruit_Thermal printer(&printerSerial);

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
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size   = FRAMESIZE_VGA;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.fb_count     = psramFound() ? 2 : 1;
  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("❌ Camera init failed!");
    while (true) delay(1000);
  }
  Serial.println("✅ Camera initialized");

  // ——— Initialize thermal printer ———
  printerSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  printer.begin();

  // ——— Prepare boot button ———
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

  // ——— Find next free filename ———
  while (true) {
    char path[32];
    sprintf(path, "/photo%03d.bmp", fileCounter);
    if (!SD.exists(path)) break;
    fileCounter++;
  }
  Serial.printf("Next file: photo%03d.bmp\n", fileCounter);
}

// ——— Convert the RGB565 frame buffer into a 1-bit bitmap and send via GS v 0 ———
void printFrame(camera_fb_t *fb) {
  uint16_t w = fb->width, h = fb->height;
  uint16_t rowBytes = (w + 7) / 8;
  uint8_t xL = rowBytes & 0xFF, xH = rowBytes >> 8;
  uint8_t yL = h & 0xFF,       yH = h >> 8;

  // GS v 0 m = 0: start bit-image
  printerSerial.write(0x1D);
  printerSerial.write('v');
  printerSerial.write('0');
  printerSerial.write((uint8_t)0x00);
  printerSerial.write(xL);
  printerSerial.write(xH);
  printerSerial.write(yL);
  printerSerial.write(yH);

  // For each row, pack 8 pixels/byte
  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t bx = 0; bx < rowBytes; bx++) {
      uint8_t byte = 0;
      for (uint8_t bit = 0; bit < 8; bit++) {
        uint16_t x = bx * 8 + bit;
        if (x < w) {
          // RGB565 = two bytes per pixel (high, low)
          uint32_t idx = 2 * (y * w + x);
          uint16_t pix = (fb->buf[idx] << 8) | fb->buf[idx + 1];
          // extract channels
          uint8_t r = ((pix >> 11) & 0x1F) * 255 / 31;
          uint8_t g = ((pix >> 5)  & 0x3F) * 255 / 63;
          uint8_t b = ( pix        & 0x1F) * 255 / 31;
          uint8_t gray = (r + g + b) / 3;
          // black if dark
          if (gray < 128) byte |= (0x80 >> bit);
        }
      }
      printerSerial.write(byte);
    }
  }
  printer.feed(2);
}

void loop() {
  // ——— Wait for BOOT button press & release ———
  Serial.println("Waiting for BOOT button press...");
  while (digitalRead(BOOT_BUTTON_PIN) == HIGH) delay(10);
  delay(50);
  while (digitalRead(BOOT_BUTTON_PIN) == LOW)  delay(10);
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
  uint8_t *bmp_buf = nullptr;
  size_t  bmp_len = 0;
  bool ok = frame2bmp(fb, &bmp_buf, &bmp_len);
  if (!ok || !bmp_buf) {
    Serial.println("❌ BMP conversion failed");
    if (bmp_buf) free(bmp_buf);
    esp_camera_fb_return(fb);
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
    esp_camera_fb_return(fb);
    delay(1000);
    return;
  }
  file.write(bmp_buf, bmp_len);
  file.close();
  Serial.printf("💾 Saved %s (%u bytes)\n", filename, (unsigned)bmp_len);
  fileCounter++;

  // ——— Print the RAW frame buffer ———
  printFrame(fb);

  // ——— Clean up & loop again ———
  esp_camera_fb_return(fb);
  free(bmp_buf);
  delay(500);
}
