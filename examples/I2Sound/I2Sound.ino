/*
 * Output data word length is 24bits/channel
 * 64 SCK in each WS stereo frame
 * 
 * I2S_FRAME_SIZE = Number of frames, 2=stereo
 * 
 * 16bits = 2 bytes
 * 24bits = 3 bytes
 * 32bits = 4 bytes
 */
 
#include <I2Sound.h>

/* -- Direct I2S Receive, we get callback to read 2 words from the FIFO -- */
void i2s_rx_callback(_I2S_SAMPLE_T *pBuf){  

  Serial.print(sizeof(pBuf[0]));
  Serial.print('-');
  Serial.print(sizeof(pBuf[1]));
  Serial.print('-');
  Serial.println(sizeof(*pBuf));
}

void setup() {
  delay(2000);
  Serial.println("Initializing...");
  
  // sample rate, word bit width, number of channels
  I2S.begin(48000, 24, 2); 
  I2S.start_rx(i2s_rx_callback);

  Serial.println("Initialized");
}

void loop() {
  // put your main code here, to run repeatedly:

}
