Button poll example.

This example project use EFM32 CMSIS, the emlib peripheral library,
the libttdcore utility library and the libtfrf library to demonstrate
how to read a GPIO input pin on the TDxxxx RF modules using polling
and send a single-byte counter to the SIGFOX network when a low-level
is detected.

The default GPIO input pin used corresponds to the UART RX pin, so
the low input level can be obtained by sending an UART BREAK from
a terminal emulator software.

On the TDxxxx EVB, the blue LED connected to the TDxxxx RF module
TIM2 pin is also turned on during the SIGFOX message transmission.
 