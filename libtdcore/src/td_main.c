/***************************************************************************//**
 * @file
 * @brief Main program for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 2.1.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012-2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
 ******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Telecom Design SA has no
 * obligation to support this Software. Telecom Design SA is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Telecom Design SA will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
  ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include <em_chip.h>
#include <em_gpio.h>

#include "td_boot.h"
#include "td_sigfox.h"
#include "td_gpio.h"
#include "td_rtc.h"
#include "td_cmu.h"
#include "td_uart.h"
#include "td_flash.h"
#include "td_timer.h"
#include "td_printf.h"
#include "td_scheduler.h"

/* This define permit to use tfp_printf even at early boot stage
 * UART_LOADER in bootloader is compliant with this flag, as long as 115200 baudrate is kept
 * */
//#define EARLY_SERIAL_DEBUG
#ifdef EARLY_SERIAL_DEBUG
#include <em_gpio.h>
#include <em_cmu.h>
#include <em_leuart.h>

/** printf function used for early debug purposes */
#define EARLY_DEBUG_PRINTF(...) tfp_printf(__VA_ARGS__)
#else

/** printf function used for early debug purposes */
#define EARLY_DEBUG_PRINTF(...)
#endif

/***************************************************************************//**
 * @addtogroup MAIN Main
 * @brief Main program for the TDxxxx RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************   PUBLIC VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup MAIN_GLOBAL_VARIABLES Global Variables
 * @{ */

/** @} */

/*******************************************************************************
 *************************   PROTOTYPES   **************************************
 ******************************************************************************/

/** @addtogroup MAIN_USER_FUNCTIONS User Functions
 * @{ */
/** @addtogroup MAIN_PROTOTYPES External Declarations
 * @{ */

/** User setup function called once */
extern void TD_USER_Setup(void);

/** User function called at each wake-up event */
extern void TD_USER_Loop(void);

/** Boot handler */
extern TD_BOOT_Handler_t const TD_BOOT_Handler;

/** @} */
/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup MAIN_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Main function for the TDxxxx RF modules.
 *
 * @details
 *   This function performs all the required initialization for the TDxxxx
 *   RF modules before calling a user supplied function to perform the user
 *   initialization.
 *   This function then enter an infinite loop going into sleep mode waiting
 *   for an interrupt, and calls a user supplied function upon wake-up.
 *
 * @note
 *   If you use this main() function, you must provide 2 functions
 *   - TD_USER_Setup(), which will be called once at startup
 *   - TD_SSER_Loop(), which will be called after waking up by an event
 *   If you provide your own main() function, it will take precedence over this
 *   one, which is completely possible. However, using the library main()
 *   function as a template for your implementation is recommended.
 *
 * @note
 *   This function never returns.
 ******************************************************************************/
int main(void)
{
	// Workarounds for chip errata
	CHIP_Init();

#ifdef EARLY_SERIAL_DEBUG
	BITBAND_Peripheral(&(CMU ->HFPERCLKDIV), (cmuClock_HFPER >> CMU_EN_BIT_POS) & CMU_EN_BIT_MASK, 1);
	BITBAND_Peripheral(&(CMU ->HFPERCLKEN0), ((cmuClock_GPIO) >> CMU_EN_BIT_POS) & CMU_EN_BIT_MASK, 1);
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_CORELEDIV2);
	CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_4);

	// Enable the LEUART0 clock
	CMU_ClockEnable(cmuClock_LEUART0, true);
	init_printf(TD_UART_Init(115200, true, false),
				TD_UART_Putc,
				TD_UART_Start,
				TD_UART_Stop);
	tfp_printf("--BOOT %s--\r\n", __TIME__);
#endif

	EARLY_DEBUG_PRINTF("TD_BOOT_Init(0x%08X)\r\n", TD_BOOT_Handler);
	if (TD_BOOT_Handler) {
		TD_BOOT_Handler(CONFIG_PRODUCT_LED_POLARITY, CONFIG_PRODUCT_TYPE);
	}

	// Initialize the clock Management Unit
	EARLY_DEBUG_PRINTF("TD_CMU_Init\r\n");
	TD_CMU_Init(true);

	// Initialize the RTC clock
	EARLY_DEBUG_PRINTF("TD_RTC_Init\r\n");
	TD_RTC_Init(0);

	// Initialize GPIOs
	EARLY_DEBUG_PRINTF("TD_GPIO_Init\r\n");
	TD_GPIO_Init();

	// Initialize SigFox (can be bypassed by : TD_SIGFOX_REMOVE_CODE)
	EARLY_DEBUG_PRINTF("TD_SIGFOX_Init\r\n");
	if (TD_SIGFOX_Init) {
		TD_SIGFOX_Init(true);
	}

	// Initialize Scheduler
	EARLY_DEBUG_PRINTF("TD_SCHEDULER_Init\r\n");
	TD_SCHEDULER_Init();

	// Call user setup function
	EARLY_DEBUG_PRINTF("TD_USER_Setup\r\n");
	TD_USER_Setup();

	EARLY_DEBUG_PRINTF("Entering Main Loop ...\r\n");

	// Main idle loop
	while (true) {

		// Prevent Exception and Interrupt from calling handler
		__set_PRIMASK(1);

		// Here:
		//  with "while" : while no background task should be called
		//                 all IRQs (or main loop) function that wanted main loop process MUST call TD_WakeMainLoop();
		//  with "if"    : if no background task should be called
		//                 all IRQs taken when in sleep mode, do a background loop
		//				   all function that wanted accurate main loop process MUST call TD_WakeMainLoop();

		if (!BackgroundRoundWanted) {

			// Go into EM2 sleep mode until an event occurs, exception/interrupt are masked
			TD_RTC_Sleep();

			// Allow exception/interrupts to call their handlers
			__set_PRIMASK(0);

			// Interrupt execution will take place at this stage

			// Now Prevent Exception and Interrupt from calling handler
			__set_PRIMASK(1);
		}

		// Clear flag. We will always do a round at this point, no sync needed
		BackgroundRoundWanted = false;

		// Allow exception/interrupts to call their handlers
		__set_PRIMASK(0);

		// Scheduler process
		TD_SCHEDULER_Process();

		// Call user loop function
		TD_USER_Loop();

		// RTC Process
		TD_RTC_Process();
	}
}

/** @} */

/** @} (end addtogroup MAIN) */
