# Teensy I2S

This reop is about connecting the ICS43432 to a Teensy 3.2 over I2S.  There are four external libraries which I tried to evaluate and use to create the library that I wanted.  I failed. 

Some of this is working and some is not.  I've run out of time and need to move on to working on something else.  Here are some of my notes.

## Notes

### Existing I2S Libraries
The most common library was built by [Hughpyle](https://github.com/hughpyle/teensy-i2s).  [A fork](https://github.com/nodae/teensy_i2s_experimental) of that library was changed to incorporate the INMP441 mems mic into the Teensy over I2S.  This version of the library is designed however to stream data through the Teensy over usb.  There are however, several important additions.  The third example is from [loglow](https://github.com/loglow/I2Sound).  This was the only library that I was able to get working well despite its lack of examples.

### Example Code

The ICS43432 outputs **2 words per frame**.  Each word contains 24 bit audio data and represents a left and right channel.  However, each half frame contains 32bits.  Caution must be used to ensure your dealing with the right information.  I believe you can account for this with the ```I2S_IO_BIT_DEPTH``` which controls the expected number of bits in the physical data (ie each word).  The challenge is matching the 24 incoming bits with the native 16/32bit integer types used in the Teensy. ```I2S_BUFFER_BIT_DEPTH``` controls what data type to store the incoming data.

I believe the ```I2Sound.ino``` example is the most complete and functioning example.  It should correctly capture the data and store it in a 32bit variable.  I'm not sure what to do with the data afterward.

### Hardware Setup
All of the code in this repo uses this hardware setup:

```
Mic  : Teensy
-------------------
+3.3 - +3.3
Gnd  - Gnd
L/R  - Gnd
SCK  - Pin 9  (I2S0_TX_BCLK - Transmit bit clock)
SD   - Pin 13 (I2S0_RXD0  - Receive data)
WS   - Pin 4  (I2S0_TX_FS - Transmit word clcock) 
```

These are not the default pin outputs used in the Teensy Audio library.  These are changed in the ```teensy_i2s.cpp``` file.  

> Note: There is only one ```I2S_RXD0``` which is on ```Pin 13```.