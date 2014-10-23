Gateway example.

This example project use EFM32 CMSIS, the emlib peripheral library,
the libttdcore utility library and the libtfrf library to demonstrate
a LAN to SIGFOX gateway that receives a message from a LAN network,
sends an acknowledgment response containing the received payload, and
forward the received payload to the SIGFOX network.

A random delay of up to 2 s is added for test purposes.
