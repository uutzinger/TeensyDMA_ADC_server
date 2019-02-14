# TeensyDMA_ADC_server
Teensy ADC with DMA and dual buffer server

This is serial server that allows streaming of analog input data up to 700kS/s
The serial commands allow setting sampling rate and input configuration.
The data is read with internatl trigger and transfered using DMA to a buffer.
In streaming mode, once the buffer is full, the software switches to a second buffer.
