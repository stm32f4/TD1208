Standard AT interpreter example.

This example project use EFM32 CMSIS, the emlib peripheral library,
the libttdcore utility library and the libtfrf library to demonstrate
how to implement an Hayes-compatible AT command parser corresponding
to the standard firmware delivered with the TDxxxx RF modules.

Beside the basic AT command set supported by the basic interpreter,
an extension provides support for both the SIGFOX-compatible
command set and the new set of SIGFOX/AT commands.

Another extension provides additional information from the RF chip.
