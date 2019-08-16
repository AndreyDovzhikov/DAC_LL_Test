# DAC_LL_Test
STM32F205 - Simple arbitrary waveform &amp; noise generator (scratch)

This is just a test (scratch).

The project has been generated by MXCUBE v5.2.0 and compiled in the Keil uVision 5.0 environment.

Timer 2 is configured to generate DMA and interrupt request simultaneously.
DMA controller is configured in the circular mode to transfer uint16_t data from the buffer (dacBuf) to the DAC data register ( DHR12L1 ).
Synchronously with the data transfer, data in the buffer is updated within TIM2_IRQHandler according to the waveform of your choice.

Hope this may be helpful or inspiring for someone.
