/*
 * Output data word length is 24bits/channel
 * 64 SCK in each WS stereo frame
 */
#include <teensy_i2s.h>
#define CLOCK_TYPE (I2S_CLOCK_48K_INTERNAL)
#define I2S_FRAME_SIZE         2            // Number of frames, 2=stereo
#define I2S_IO_BIT_DEPTH       16           // Number of bits per sample in the physical data (8, 16 or 32)
#define I2S_BUFFER_BIT_DEPTH   16           // Number of bits per sample in the DMA buffer (8, 16 or 32)
//const uint16_t buffersize = 2;
//volatile int16_t buffer[buffersize];
//uint16_t nRX = 0;

void i2s_rx_callback(int16_t *pBuf){
  // Read the data
  // buffer size = I2S_FRAME_SIZE (default: 2)
  // _I2S_SAMPLE_T  - default: int16_t
  Serial.println(".");
//  buffer[nRX++] = pBuf[0];
//  buffer[nRX++] = pBuf[1];
//  if( nRX>=buffersize ) nRX=0;  
}

void setup() {
  // put your setup code here, to run once:
  delay(2000);
  Serial.println("Initializing...");
  delay(1000);

  I2SRx0.begin(CLOCK_TYPE, i2s_rx_callback);

  // Set the buffer pointer
  //nRX = 0;
  
  // Start the I2S RX
  I2SRx0.start();
  //I2STx0.start(); 
  Serial.println("I2S initialized");
}

void loop() {
  // put your main code here, to run repeatedly:

}
