Button interrupt example.

This example project use EFM32 CMSIS, the emlib peripheral library,
the libttdcore utility library and the libtfrf library to demonstrate
how to read a GPIO input pin on the TDxxxx RF modules using interrupts
and send a single-byte counter to the SIGFOX network when a low-level
is detected.

The default GPIO input pin used corresponds to the UART RX pin, so
the low input level can be obtained by sending an UART BREAK from
a terminal emulator software.

On the TDxxxx EVB, the blue LED connected to the TDxxxx RF module
TIM2 pin is also turned on during the SIGFOX message transmission.

Rather than processing the interrupt completely inside the interrupt
handler routine and blocking other interrupts in the meantime, this
program only sets an event flag in the interrupt routine, which is
processed in the main event loop when the interrupts wake up the
CPU.
