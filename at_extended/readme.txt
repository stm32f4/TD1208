Extended AT interpreter example.

This example project use EFM32 CMSIS, the emlib peripheral library,
the libttdcore utility library and the libtfrf library to demonstrate
how to implement an Hayes-compatible AT command parser corresponding
to the standard firmware delivered with the TDxxxx RF modules, with a
user extension.

Beside the AT command set supported by standard interpreter, the user
extension provides support for toggling the TDxxxx EVB LED connected
to the TDxxxx TIM2 pin, and to read both the chip temperature and
voltage.
