// Daniel Gilbert
// loglow@gmail.com
// copyright 2013



#include "I2Sound.h"
#include <mk20dx128.h>
#include <stdint.h>
#include "core_pins.h"



// ------------------------------------------------------------
// pre-initialized class object for the I2S interface (only one)
// ------------------------------------------------------------
I2Sound I2S;

// Buffers for audio samples.
static _I2S_SAMPLE_T _i2s_Rx_Buffer[I2S_FRAME_SIZE];

// Configures the number of words in each frame. The value written should be one less than the number of
// words in the frame (for example, write 0 for one word per frame). The maximum supported frame size is
// 16 words.
#define FRSZ (I2S_FRAME_SIZE-1)

// Configures the length of the frame sync in number of bit clocks. The value written must be one less than
// the number of bit clocks. For example, write 0 for the frame sync to assert for one bit clock only. The sync
// width cannot be configured longer than the first word of the frame.
#define SYWD (I2S_IO_BIT_DEPTH-1)

// ------------------------------------------------------------
// initialize the MCLK divider and enable clock output
// ------------------------------------------------------------
bool I2Sound::init_mclk() {
  
  uint32_t fract, divide;
  if (nSamples == 48000) { fract = 16; divide = 125; }
  else if (nSamples == 44100) { fract = 1; divide = 16; } 
  else return false;

  SIM_SCGC6 |= SIM_SCGC6_I2S;                // enable clock to the I2S module
  I2S0_MDR |= I2S_MDR_FRACT((fract - 1));    // output = input * (FRACT + 1) / (DIVIDE + 1)
  I2S0_MDR |= I2S_MDR_DIVIDE((divide - 1));  // example: 12.288 MHz = 96 MHz * (16 / 125)
  I2S0_MCR |= I2S_MCR_MOE;                   // enable MCLK pin as output
  
  return true;
  
}



// ------------------------------------------------------------
// initialize all the RX (receive) registers, configure the
// bit and word (frame sync) clocks, and setup interrupts.
// note: SAI stands for "Synchronous Audio Interface"
// ------------------------------------------------------------
void I2Sound::init_rx() {
  // receive disable while we configure everything
  // I2S0_RCSR &= ~(I2S_RCSR_RE);

  // // Receiver remains enabled until (and TE set) the end of the current frame
  // for( int i=0; i<1000 && (I2S0_RCSR & I2S_RCSR_RE); i++ );
  // if( I2S0_RCSR & I2S_RCSR_RE )
  //     return;

  I2S0_RMR = 0;       // 0 = No word mask (stereo). 1 and 2 maske
                      // either left or right audio side  

  // SAI Receive Configuration 1 Register
  I2S0_RCR1 =
    I2S_RCR1_RFW(FRSZ)  // set FIFO watermark
  ;
  
  // SAI Receive Configuration 2 Register
  I2S0_RCR2 =
    I2S_RCR2_MSEL(1)  // use MCLK as BCLK source
  | I2S_RCR2_SYNC(0)  // use asynchronous mode
  | I2S_RCR2_DIV(1)   // (DIV + 1) * 2, example: 12.288 MHz / 4 = 3.072 MHz
  | I2S_RCR2_BCD      // generate BCLK, master mode
  | I2S_TCR2_BCP      // BCLK is active low
  ;
  
  // SAI Receive Configuration 3 Register
  I2S0_RCR3 =
    I2S_RCR3_RCE  // enable receive channel
  ;
  
  // SAI Receive Configuration 4 Register
  I2S0_RCR4 =
    I2S_RCR4_FRSZ(FRSZ)       // frame size in words
  | I2S_RCR4_SYWD(SYWD)       // bit width of WCLK
  | I2S_RCR4_MF                  // MSB (most significant bit) first
  | I2S_RCR4_FSD                 // generate WCLK, master mode
  | I2S_RCR4_FSE                 // extra bit before frame starts
  ;
  
  // SAI Receive Configuration 5 Register
  I2S0_RCR5 =
    I2S_RCR5_W0W(SYWD)  // bits per word, first frame
  | I2S_RCR5_WNW(SYWD)  // bits per word, nth frame
  | I2S_RCR5_FBT(SYWD)  // index shifted for FIFO
  ;
  
  // SAI Receive Control Register
  I2S0_RCSR =
    I2S_RCSR_BCE   // enable the BCLK output
  | I2S_RCSR_RE    // enable receive globally
  | I2S_RCSR_FRIE  // enable FIFO request interrupt
  | I2S_RCSR_FR    // FIFO Reset
  ;
  
}



// ------------------------------------------------------------
// TODO -- initialize all the TX (receive) registers
// ------------------------------------------------------------
void I2Sound::init_tx() {
    // transmit disable while we configure everything
    I2S0_TCSR &= ~(I2S_TCSR_TE);
    
    // Transmitter remains enabled until (and TE set) the end of the current frame
    for( int i=0; i<1000 && (I2S0_TCSR & I2S_TCSR_TE); i++ );
    if( I2S0_TCSR & I2S_TCSR_TE )
        return;

    I2S0_TMR = 0;         // No word mask
    
    // SAI Receive Configuration 1 Register
    I2S0_TCR1  = 
      I2S_TCR1_TFW(FRSZ)        // set FIFO watermark
    ;

    // SAI Receive Configuration 2 Register
    I2S0_TCR2  = 
      I2S_TCR2_SYNC(0)   // use asynchronous mode
    | I2S_TCR2_MSEL(1)    // use mc1 (notbus clock as BCLK source
    | I2S_TCR2_DIV(1)     // Divides down the audio master clock to generate the bit clock
                          // division value is (DIV + 1) * 2.
    | I2S_TCR2_BCP        // BCLK polarity: active low
    | I2S_TCR2_BCD        // BCLK is generated internally (master mode)
    ;          

    // SAI Receive Configuration 3 Register
    I2S0_TCR3  = 
      I2S_TCR3_TCE         // transmit data channel is enabled
    ; 

    // SAI Receive Configuration 4 Register
    I2S0_TCR4  = 
      I2S_TCR4_FRSZ(FRSZ)   // frame size in words (plus one)
    | I2S_TCR4_SYWD(SYWD)    // number of bits in frame sync (plus one)
    | I2S_TCR4_MF                 // MSB (most significant bit) first
    | I2S_TCR4_FSE                // Frame sync one bit before the frame
    | I2S_TCR4_FSD                // WCLK is generated internally (master mode)
    ;
    
    // SAI Receive Configuration 5 Register
    I2S0_TCR5  = 
      I2S_TCR5_W0W(SYWD)        // bits per word, first frame
    | I2S_TCR5_WNW(SYWD)        // bits per word, nth frame
    | I2S_TCR5_FBT(SYWD)        // index shifted for FIFO (TODO depend on I2S_BUFFER_BIT_DEPTH)  
    ;

    // SAI Receive Control Register
    I2S0_TCSR = 
      I2S_TCSR_TE       // Transmit Enable
    | I2S_TCSR_BCE      // Bit Clock Enable
    | I2S_TCSR_FRIE     // FIFO Request Interrrupt Enable
    | I2S_TCSR_FR       // FIFO Reset
    ;
}


// ------------------------------------------------------------
// Configure Pins for tx and rx
// ------------------------------------------------------------
void I2Sound::io_init(void){

  // TX
  CORE_PIN3_CONFIG  = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TXD0
  CORE_PIN4_CONFIG  = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TX_FS
  CORE_PIN9_CONFIG  = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TX_BCLK

  // RX
  CORE_PIN11_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RX_BCLK
  CORE_PIN12_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RX_FS
  CORE_PIN13_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RXD0  
}

// ------------------------------------------------------------
// public, universal init function. needs to be passed sample
// rate, bit depth, and number of audio channels
// ------------------------------------------------------------
bool I2Sound::begin(uint32_t newSamples, uint8_t newBits, uint8_t newChans) {
  nSamples = newSamples;
  nBits = newBits;
  nChans = newChans;
  if(!init_mclk()) return false;
  io_init();
  init_rx();
  init_tx(); 
}



// ------------------------------------------------------------
// start the I2S receive process. this function needs to be
// passed the name of a user-defined callback funtion to execute
// when an RX interrupt is generated. the callback function
// must take no args and return void. enables RX interrupts.
// also passed pointers to where the audio data will be stored
// ------------------------------------------------------------
void I2Sound::start_rx(void (*fptr)( _I2S_SAMPLE_T *pBuf)) {
  fnI2SCallback = fptr;
  NVIC_ENABLE_IRQ(IRQ_I2S0_RX);
}



// ------------------------------------------------------------
// start the I2S transmit process. this function needs to be
// passed the name of a user-defined callback funtion to execute
// when a TX interrupt is generated. the callback function
// must take no args and return void. enables TX interrupts.
// ------------------------------------------------------------
// void I2Sound::start_tx(ISR new_txISR) {
//   txISR = new_txISR;
//   NVIC_ENABLE_IRQ(IRQ_I2S0_TX);
// }



// ------------------------------------------------------------
// stop the I2S receive process by disabling RX interrupts
// ------------------------------------------------------------
void I2Sound::stop_rx() {
  NVIC_DISABLE_IRQ(IRQ_I2S0_RX);
}



// ------------------------------------------------------------
// stop the I2S transmit process by disabling TX interrupts
// ------------------------------------------------------------
void I2Sound::stop_tx() {
  NVIC_DISABLE_IRQ(IRQ_I2S0_TX);
}


void I2Sound::i2s_rx_callback(void){
  if( I2S_FRAME_SIZE>0 ) _i2s_Rx_Buffer[0] = (_I2S_SAMPLE_T)I2S0_RDR0;
  if( I2S_FRAME_SIZE>1 ) _i2s_Rx_Buffer[1] = (_I2S_SAMPLE_T)I2S0_RDR0;
  if( I2S_FRAME_SIZE>2 ) _i2s_Rx_Buffer[2] = (_I2S_SAMPLE_T)I2S0_RDR0;
  if( I2S_FRAME_SIZE>3 ) _i2s_Rx_Buffer[3] = (_I2S_SAMPLE_T)I2S0_RDR0;

  fnI2SCallback( _i2s_Rx_Buffer );
  
  I2S0_RCSR |= I2S_RCSR_FR;  // reset fifo
  if(I2S0_RCSR & I2S_RCSR_FEF)  I2S0_RCSR |= I2S_RCSR_FEF; // clear if underrun
  if(I2S0_RCSR & I2S_RCSR_SEF)  I2S0_RCSR |= I2S_RCSR_SEF; // clear if frame sync error  
}

// ------------------------------------------------------------
// wrapper for the default I2S receive ISR, this allows
// the definition of a custom callback function for interrupts.
// the callback must not have any arguments and return void.
// read() populates provided memory addresses with audio data
// ------------------------------------------------------------
void i2s0_rx_isr(void) {
  I2S.i2s_rx_callback();
}



// ------------------------------------------------------------
// wrapper for the default I2S transmit ISR, this allows
// the definition of a custom callback function for interrupts.
// the callback must not have any arguments and return void.
// ------------------------------------------------------------
// void i2s0_tx_isr() {
//   I2S.txISR();
// }


// EOF