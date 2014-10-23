Template for new TDxxxx RF module projects.

This example project use EFM32 CMSIS, the emlib peripheral library and
the libttdcore utility library and is intended as a skeleton for new
projects.

This example uses a main() function that is managed by the libtdcore
library, providing the user with a TD_USER_Setup() function called once
at startup and a TD_USER_Loop() function called every time the TDxxxx
RF modules wakes up from sleep mode.

This managed mode is not mandatory: the user can provide a main()
function that will take precedence over the library one if this is
required by the application. In this case, the libtdcore's main()
function can serve as a skeleton for the user's main() function.

 