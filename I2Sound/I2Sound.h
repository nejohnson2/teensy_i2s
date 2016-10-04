// Daniel Gilbert
// loglow@gmail.com
// copyright 2013



#ifndef __I2SOUND_H__
#define __I2SOUND_H__

#include <stdint.h>
#include <inttypes.h>

#define I2S_FRAME_SIZE         2    // Number of frames, 2=stereo
#define I2S_IO_BIT_DEPTH       24   // Number of bits per sample in the physical data (8, 16 or 32)
#define I2S_BUFFER_BIT_DEPTH   32   // Bits in storage buffer

#if I2S_BUFFER_BIT_DEPTH==8
#define _I2S_SAMPLE_T          int8_t
#elif I2S_BUFFER_BIT_DEPTH==16
#define _I2S_SAMPLE_T          int16_t
#else
#define _I2S_SAMPLE_T          int32_t
#endif

class I2Sound {
  private:
    void (*fnI2SCallback)( _I2S_SAMPLE_T *pBuf );  

    uint8_t nChans;
    uint8_t nBits;
    uint32_t nSamples;
    bool init_mclk();
    void io_init();
    void init_rx();
    void init_tx();

  public:
    bool begin(uint32_t newSamples, uint8_t newBits, uint8_t newChans);
    void start_rx(void (*fptr)( _I2S_SAMPLE_T *pBuf ));
    
    void stop_rx();
    void stop_tx();


    inline void i2s_rx_callback(void);    
};

extern I2Sound I2S;



#endif



// EOF