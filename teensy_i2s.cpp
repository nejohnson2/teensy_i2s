/*
 * I2S interface for Teensy 3.0
 * Fork this on github https://github.com/hughpyle/teensy-i2s
 *
 * Copyright (c) 2013 by Hugh Pyle and contributors.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */
 
#include <teensy_i2s.h>
//#include <mk20dx128.h>
#include "mk20dx128.h"
#include "core_pins.h"

I2S_class I2STx0(0);
I2S_class I2SRx0(1);

// Buffers for 16 bit audio samples.
static _I2S_SAMPLE_T _i2s_Rx_Buffer[I2S_FRAME_SIZE];
static _I2S_SAMPLE_T _i2s_Tx_Buffer[I2S_FRAME_SIZE];

I2S_class::I2S_class(uint8_t isRx)
{
    receive = isRx;
}

/* Initialize the I2S interface */
void I2S_class::begin(uint8_t clk, void (*fptr)( _I2S_SAMPLE_T *pBuf))
{
    clock = clk;
    fnI2SCallback = fptr;
    io_init();
    clock_init();
    i2s_receive_init();
    i2s_transmit_init();
}

void I2S_class::start()
{
    // When FIFO needs data it generates an interrupt.
    // Receive enable
    NVIC_ENABLE_IRQ(IRQ_I2S0_RX);
    I2S0_RCSR |= I2S_RCSR_RE            // Receive Enable
               | I2S_RCSR_BCE           // Bit Clock Enable
               | I2S_RCSR_FRIE          // FIFO Request Interrupt Enable
               | I2S_RCSR_FR            // FIFO Reset
               ;

    // Transmit enable
    NVIC_ENABLE_IRQ(IRQ_I2S0_TX);
    I2S0_TCSR |= I2S_TCSR_TE            // Transmit Enable
               | I2S_TCSR_BCE           // Bit Clock Enable
               | I2S_TCSR_FRIE          // FIFO Request Interrupt Enable
               | I2S_TCSR_FR            // FIFO Reset
               ;
}

void I2S_class::stop()
{
    // turn off interrupts
    NVIC_DISABLE_IRQ(IRQ_I2S0_RX);
    NVIC_DISABLE_IRQ(IRQ_I2S0_TX);
}

void I2S_class::io_init(void){

  // TX
  CORE_PIN3_CONFIG  = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TXD0
  CORE_PIN4_CONFIG  = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TX_FS
  CORE_PIN9_CONFIG  = PORT_PCR_DSE | PORT_PCR_MUX(6);     // I2S0_TX_BCLK

  // RX
  CORE_PIN11_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RX_BCLK
  CORE_PIN12_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RX_FS
  CORE_PIN13_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(4);     // I2S0_RXD0  
}


void I2S_class::clock_init()
{
    // Disable system clock to the I2S module
    //SIM_SCGC6 &= ~(SIM_SCGC6_I2S);
    SIM_SCGC6 |= SIM_SCGC6_I2S;

    // Select input clock 0 and output enable
    I2S0_MCR = I2S_MCR_MICS(0) | I2S_MCR_MOE;
    
    // 8k, 12k, 16k, 32k, etc all clock the I2S module at 12.288 MHz
    // 11025Hz, 22050, 44100 clock the I2S module at 11.2896 MHz
    switch( clock )
    {
        case I2S_CLOCK_44K_INTERNAL:
            // Divide to get the 11.2896 MHz from 96MHz (96* (2/17))
            I2S0_MDR = I2S_MDR_FRACT(1) | I2S_MDR_DIVIDE(16);
            break;
        // remember: switch cases without break "fall trough" to later cases
        case I2S_CLOCK_8K_INTERNAL:
        case I2S_CLOCK_32K_INTERNAL:
        case I2S_CLOCK_48K_INTERNAL:
            // uses default case
        default:
            // Divide to get the 12.2880 MHz from 96MHz (96* (16/125))
            I2S0_MDR = I2S_MDR_FRACT(15) | I2S_MDR_DIVIDE(124);
            break;
    }

    // re-enable system clock to the I2S module
    SIM_SCGC6 |= SIM_SCGC6_I2S;
}

// Configures the number of words in each frame. The value written 
// should be one less than the number of words in the frame (for 
// example, write 0 for one word per frame). The maximum supported 
// frame size is 16 words.
#define FRSZ (I2S_FRAME_SIZE-1)

// Configures the length of the frame sync in number of bit clocks. 
// The value written must be one less than the number of bit clocks. For example, 
// write 0 for the frame sync to assert for one bit clock only. The sync
// width cannot be configured longer than the first word of the frame.
#define SYWD (I2S_IO_BIT_DEPTH-1)

void I2S_class::i2s_transmit_init()
{

    // transmit disable while we configure everything
    I2S0_TCSR &= ~(I2S_TCSR_TE);
    
    // Transmitter remains enabled until (and TE set) the end of the current frame
    for( int i=0; i<1000 && (I2S0_TCSR & I2S_TCSR_TE); i++ );
    if( I2S0_TCSR & I2S_TCSR_TE )
        return;

    I2S0_TMR = 0;                           // No word mask
    // --------------------------------------------------------------------------------
    I2S0_TCR1  = I2S_TCR1_TFW(FRSZ);        // set FIFO watermark
    // --------------------------------------------------------------------------------
    I2S0_TCR2  = I2S_TCR2_SYNC(0);          // use asynchronous mode
    I2S0_TCR2 |= I2S_TCR2_BCP;              // BCLK polarity: active low
    I2S0_TCR2 |= I2S_TCR2_MSEL(1);          // use mc1 (notbus clock as BCLK source
    I2S0_TCR2 |= I2S_TCR2_DIV(3);           // divide internal master clock to generate bit clock
    I2S0_TCR2 |= I2S_TCR2_BCD;              // BCLK is generated internally (master mode)
    
    // --------------------------------------------------------------------------------    
    I2S0_TCR3  = I2S_TCR3_TCE;              // transmit data channel is enabled
    // --------------------------------------------------------------------------------
    I2S0_TCR4  = I2S_TCR4_FRSZ(FRSZ);       // frame size in words (plus one)
    I2S0_TCR4 |= I2S_TCR4_SYWD(SYWD);       // number of bits in frame sync (plus one)
    I2S0_TCR4 |= I2S_TCR4_MF;               // MSB (most significant bit) first
    I2S0_TCR4 |= I2S_TCR4_FSE;              // Frame sync one bit before the frame
    I2S0_TCR4 |= I2S_TCR4_FSD;              // WCLK is generated internally (master mode)

    // --------------------------------------------------------------------------------
    I2S0_TCR5  = I2S_TCR5_W0W(SYWD);        // bits per word, first frame
    I2S0_TCR5 |= I2S_TCR5_WNW(SYWD);        // bits per word, nth frame
    I2S0_TCR5 |= I2S_TCR5_FBT(SYWD);        // index shifted for FIFO (TODO depend on I2S_BUFFER_BIT_DEPTH)
    
}

void I2S_class::i2s_receive_init()
{
    // receive disable while we configure everything
    // I2S0_RCSR &= ~(I2S_RCSR_RE);

    // // Receiver remains enabled until (and TE set) the end of the current frame
    // for( int i=0; i<1000 && (I2S0_RCSR & I2S_RCSR_RE); i++ );
    // if( I2S0_RCSR & I2S_RCSR_RE )
    //     return;

    I2S0_RMR = 0;                           // 0 = No word mask (stereo). 1 and 2 maske
                                            // either left or right audio side
    // --------------------------------------------------------------------------------
    I2S0_RCR1  = I2S_RCR1_RFW(FRSZ);        // set FIFO watermark
    // --------------------------------------------------------------------------------
    I2S0_RCR2  = I2S_RCR2_SYNC(0);          // synchronous with the transmitter
    I2S0_RCR2 |= I2S_RCR2_BCP;              // BCLK polarity: active low
    I2S0_RCR2 |= I2S_RCR2_MSEL(1);          // use MCLK as BCLK source
    I2S0_RCR2 |= I2S_RCR2_DIV(1);           // (DIV + 1) * 2, 12.288 MHz / 4 = 3.072 MHz
    //I2S0_RCR2 |= I2S_RCR2_DIV(1);           // 3.072 MHz / 64 bit per frame = 48kHz sampling
    I2S0_RCR2 |= I2S_RCR2_BCD;              // BCLK is generated internally in Master mode

    // --------------------------------------------------------------------------------
    I2S0_RCR3  = I2S_RCR3_RCE;              // receive data channel is enabled (channel 0)
    // --------------------------------------------------------------------------------
    I2S0_RCR4  = I2S_RCR4_FRSZ(FRSZ);       // frame size in words (plus one)
    I2S0_RCR4 |= I2S_RCR4_SYWD(SYWD);       // bit width of a word (plus one)
    I2S0_RCR4 |= I2S_RCR4_MF;               // MSB (most significant bit) first
    I2S0_RCR4 |= I2S_RCR4_FSE;              // Frame sync one bit before the frame
    I2S0_RCR4 |= I2S_RCR4_FSD;          // WCLK is generated internally (master mode)

    // --------------------------------------------------------------------------------
    I2S0_RCR5  = I2S_RCR5_W0W(SYWD);        // bits per word, first frame
    I2S0_RCR5 |= I2S_RCR5_WNW(SYWD);        // bits per word, nth frame
    I2S0_RCR5 |= I2S_RCR5_FBT(SYWD);        // index shifted for FIFO (TODO depend on I2S_BUFFER_BIT_DEPTH)
    //I2S0_RCR5 |= I2S_RCR5_FBT(SYWD);        // 
}

/* I2S Class-instance callback */
void I2S_class::i2s_rx_callback(void)
{  
    // Copy the data from FIFO into our buffer
    if( I2S_FRAME_SIZE>0 ) _i2s_Rx_Buffer[0] = (_I2S_SAMPLE_T)I2S0_RDR0;
    if( I2S_FRAME_SIZE>1 ) _i2s_Rx_Buffer[1] = (_I2S_SAMPLE_T)I2S0_RDR0;
    if( I2S_FRAME_SIZE>2 ) _i2s_Rx_Buffer[2] = (_I2S_SAMPLE_T)I2S0_RDR0;
    if( I2S_FRAME_SIZE>3 ) _i2s_Rx_Buffer[3] = (_I2S_SAMPLE_T)I2S0_RDR0;

    // Call your function to handle the data
    fnI2SCallback( _i2s_Rx_Buffer );
    
   if(I2S0_RCSR & I2S_RCSR_FEF)  I2S0_RCSR |= I2S_RCSR_FEF; // clear if underrun
   if(I2S0_RCSR & I2S_RCSR_SEF)  I2S0_RCSR |= I2S_RCSR_SEF; // clear if frame sync error
}
/* I2S class-instance callbacks */
// void I2S_class::i2s_tx_callback(void)
// {
//     if(!(I2S0_TCSR & I2S_TCSR_FRF))  return;
    
//     // Call your function to get the data into our buffer
//     fnI2SCallback( _i2s_Tx_Buffer );

//     // Copy the data from our buffer into FIFO
//     if( I2S_FRAME_SIZE>0 ) I2S0_TDR0 = (uint32_t)(_i2s_Tx_Buffer[0]);
//     if( I2S_FRAME_SIZE>1 ) I2S0_TDR0 = (uint32_t)(_i2s_Tx_Buffer[1]);
//     if( I2S_FRAME_SIZE>2 ) I2S0_TDR0 = (uint32_t)(_i2s_Tx_Buffer[2]);
//     if( I2S_FRAME_SIZE>3 ) I2S0_TDR0 = (uint32_t)(_i2s_Tx_Buffer[3]);
    
//     //for( uint8_t i=0; i<I2S_FRAME_SIZE-1; i++ )
//     //{
//         //I2S0_TDR0 = (uint32_t)(_I2S_SAMPLE_T)(_i2s_Tx_Buffer[i]);
//     //}
    
//     if(I2S0_TCSR & I2S_TCSR_FEF) {
//         I2S0_TCSR |= I2S_TCSR_FEF; // clear if underrun
//     }
//     if(I2S0_TCSR & I2S_TCSR_SEF) {
//         I2S0_TCSR |= I2S_TCSR_SEF; // clear if frame sync error
//     }
// }
/* I2S ISR */
void i2s0_rx_isr(void)
{
    I2SRx0.i2s_rx_callback();
}
/* I2S ISR (used when you're not using DMA) */
// void i2s0_tx_isr(void)
// {
//     I2STx0.i2s_tx_callback();
// }
