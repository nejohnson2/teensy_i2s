#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioInputI2S            i2s1;           //xy=145,83
AudioOutputUSB           usb1;           //xy=367,75
AudioConnection          patchCord1(i2s1, 0, usb1, 0);
AudioConnection          patchCord2(i2s1, 1, usb1, 1);
// GUItool: end automatically generated code

void setup() {
  // put your setup code here, to run once:
  AudioMemory(60);
}

void loop() {
  // put your main code here, to run repeatedly:

}
