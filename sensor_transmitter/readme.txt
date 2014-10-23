Template for new TD12202 projects.

This example project use EFM32 CMSIS, the emlib peripheral library and
the libttd1202 utility library and is intended as a skeleton for new
projects.

This example uses a main() function that is managed by the libtd1202
library, providing the user with a TD_USER_Setup() function called once
at startup and a TD_USER_Loop() function called every time the TD1208
modules wakes up from sleep mode.

This managed mode is not mandatory: the user can provide a main()
function that will take precedence over the library one if this is
required by the application. In this case, the libtd1202's main()
function can serve as a skeleton for the user's main() function.

 