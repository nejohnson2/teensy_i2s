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
 
#ifndef __TEENSY_I2S_H__
#define __TEENSY_I2S_H__

#include <inttypes.h> 
#include <WProgram.h>

// Audio configuration.  Edit these here if you need to.
#define I2S_FRAME_SIZE         2            // Number of frames, 2=stereo
#define I2S_IO_BIT_DEPTH       16           // Number of bits per sample in the physical data (8, 16 or 32)
#define I2S_BUFFER_BIT_DEPTH   16           // Number of bits per sample in the DMA buffer (8, 16 or 32)

// Clock type constants
#define I2S_CLOCK_EXTERNAL     0            // The bit clock is provided by an external device (e.g. the codec)
#define I2S_CLOCK_8K_INTERNAL  1            // The bit clock is 8kHz, internally generated
#define I2S_CLOCK_32K_INTERNAL 2            // The bit clock is 32kHz, internally generated
#define I2S_CLOCK_44K_INTERNAL 3            // The bit clock is 44.1kHz, internally generated
#define I2S_CLOCK_48K_INTERNAL 4            // The bit clock is 48kHz, internally generated

// Data type for the API
// #if I2S_BUFFER_BIT_DEPTH==8
// #define _I2S_SAMPLE_T          int8_t
// #elif I2S_BUFFER_BIT_DEPTH==16
// #define _I2S_SAMPLE_T          int16_t
// #else
// #define _I2S_SAMPLE_T          int32_t
// #endif
#define _I2S_SAMPLE_T int16_t

class I2S_class
{
    private:
        // Flags
        uint8_t clock;      /* one of I2S_CLOCK_xxx */
        uint8_t receive;

        // the I2S callback (buffer size = I2S_FRAME_SIZE)
        void (*fnI2SCallback)( _I2S_SAMPLE_T *pBuf );  

        void io_init();
        void clock_init();
        void i2s_transmit_init();
        void i2s_receive_init();
        
    public:
        I2S_class(uint8_t isRx);
        /*
         * @brief       Initialize the I2S interface 
         *
         * @param[in]   clk     The clock type and speed, one of I2S_CLOCK_xxx
         * @param[in]   fptr    The callback function that your sketch implements.
         *                      This will be called with a pointer to a buffer
         *                      where you will read or write I2S_FRAME_SIZE 
         *                      of _I2S_SAMPLE_T audio data.
         * @return      none.
         */
        void begin(uint8_t clk, void (*fptr)( _I2S_SAMPLE_T *pBuf ));
        
        
        /*
         * @brief   Start the I2S interface.  (You must have initialized first).
         * @return  none.
         */
        void start();
        
        /*
         * @brief   Stop the I2S interface.  (You must have initialized first).
         * @return  none.
         */
        void stop();

        /* internal */
        inline void i2s_rx_callback(void);
        inline void i2s_tx_callback(void);
};

extern I2S_class I2SRx0;
extern I2S_class I2STx0;

#endif
