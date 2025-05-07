#include "Adafruit_Thermal.h"
#include "mybitmap.h"    // ← your bitmap header

// ——— Printer Setup ———
#define TX_PIN 43
#define RX_PIN 44

HardwareSerial printerSerial(1);
Adafruit_Thermal printer(&printerSerial);

// ——— Print Bitmap Raw ———
void printBitmapRaw(const uint8_t *bitmap, uint16_t width, uint16_t height) {
  uint16_t rowBytes = (width + 7) / 8;
  uint8_t xL = rowBytes & 0xFF;
  uint8_t xH = (rowBytes >> 8) & 0xFF;
  uint8_t yL = height & 0xFF;
  uint8_t yH = (height >> 8) & 0xFF;

  // GS v 0: start bitmap print
  printerSerial.write(0x1D);
  printerSerial.write('v');
  printerSerial.write('0');
  printerSerial.write((uint8_t)0x00);
  printerSerial.write(xL);
  printerSerial.write(xH);
  printerSerial.write(yL);
  printerSerial.write(yH);

  // Send the bitmap data from PROGMEM
  uint32_t total = (uint32_t)rowBytes * height;
  for (uint32_t i = 0; i < total; i++) {
    printerSerial.write(pgm_read_byte(bitmap + i));
  }
}

void setup() {
  // initialize printer serial
  printerSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  printer.begin();
}

void loop() {
  // Print the bitmap defined in mybitmap.h
  printBitmapRaw(my_bitmap, my_bitmap_width, my_bitmap_height);
  printer.feed(2);

  // Done—stop here
  while (true) delay(1000);
}
