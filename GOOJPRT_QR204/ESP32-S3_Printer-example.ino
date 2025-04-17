//https://github.com/AndersV209/Pos-Printer-Library/tree/master

#include "Pos_Printer.h"
#include "SoftwareSerial.h"
#define TX_PIN 43 // Arduino transmit  YELLOW WIRE  labeled RX on printer
#define RX_PIN 44 // Arduino receive   GREEN WIRE   labeled TX on printer

SoftwareSerial mySerial(RX_PIN, TX_PIN); // Declare SoftwareSerial obj first
Pos_Printer printer(&mySerial);     // Pass addr to printer constructor

void setup() {

mySerial.begin(9600);  // Initialize SoftwareSerial

printer.begin();        // Init printer (same regardless of serial type)

printer.println(F("HAL9K!"));
printer.feed(2);
printer.sleep();      // Tell printer to sleep
delay(3000L);         // Sleep for 3 seconds
printer.wake();       // MUST wake() before printing again, even if reset
printer.setDefault(); // Restore printer to defaults
}

void loop() {
}
  