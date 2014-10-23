LAN TX example.

This example project use EFM32 CMSIS, the emlib peripheral library,
the libttdcore utility library and the libtfrf library to demonstrate
a simple LAN transmitter that sends a message to a LAN network every
1 to 10 s, and wait for an acknowledgment response.

On the TDxxxx EVB, the blue LED connected to the TDxxxx RF module TIM2
pin is also turned on during the LAN message transmission.

A random delay of up to 10 s is added for test purposes.
