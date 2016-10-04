#include <SPI.h>
#include "SdFat.h"

const int chipSelect = 10;
SdFat sd;
SdFile dataFile;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SPI.begin();
  SPI.setMOSI(7);
  SPI.setSCK(14);
  
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    sd.initErrorHalt();
  }

  // open the file for write at end like the Native SD library
  if (!dataFile.open("DATAFILE.txt", O_RDWR | O_CREAT | O_AT_END)) {
    sd.errorHalt("opening test.txt for write failed");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
 String dataString;

 dataString = "justpinmo";
 Serial.println(dataString);
 dataFile.println(dataString);
 dataFile.flush();
 delay(3000);
}
