/******************************************************************************
 * @file
 * @brief TD LAN TX example for TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 2.0.1
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
#include <efm32.h>
#include <em_gpio.h>
#include <td_core.h>
#include <td_uart.h>
#include <td_printf.h>
#include <td_rtc.h>
#include <td_gpio.h>
#include <td_lan.h>

/* This file declare all "dynamic" library data. It should be last included file
 * Standard size value can be override before including this file
 */

#define FORCE_RADIO_RESET 0
#define TD_SENSOR_USE_CODE 0
#define TD_GEOLOC_USE_CODE 0
#include <td_config.h>

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

#define LED_PORT		TIM2_PORT			/**< LED port */
#define LED_BIT			TIM2_BIT			/**< LED bit */

#define NODE_ADDRESS	0x00005203			/**< Node address */
#define NODE_NETWORK	0x00200000			/**< Node network */
#define	NODE_MASK		0x00FFFFFF			/**< Node mask */
#define RETRIES			2					/**< Number of transmit retries */

#define FREQUENCY		869000000			/**< Channel frequency */
#define POWER_LEVEL		14					/**< Transmit power level */

/*******************************************************************************
 ******************************  CONSTANTS  ************************************
 ******************************************************************************/

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *   User setup function.
 ******************************************************************************/
void TD_USER_Setup(void)
{
    uint32_t count = 0, ack = 0, nack = 0;
    int i, delay;
    TD_LAN_frame_t RX, TX;

	// Define the LED pin as an output in push-pull mode
	GPIO_PinModeSet(LED_PORT, LED_BIT, gpioModePushPull, 0);

	// Initialize the LEUART
    init_printf(TD_UART_Init(9600, true, false),
    		TD_UART_Putc,
    		TD_UART_Start,
    		TD_UART_Stop);

    // Initialize the LAN
    if (TD_LAN_Init(true, (NODE_ADDRESS | NODE_NETWORK), NODE_MASK) == false) {
    	printf("Problem in init\r\n");
	}

    // Set the operating frequency and power level
    if (TD_LAN_SetFrequencyLevel(FREQUENCY, POWER_LEVEL) == false) {
    	printf("Problem while setting frequency/level\r\n");
    }

    TX.header = 0;
    SET_ADDRESS(TX.header, (NODE_ADDRESS | NODE_NETWORK));

    TD_RTC_Delay(T3S);
    while (true) {
    	for (i = 0; i < 8; i++) {
    		TX.payload[i] = (count + i) & 0xFF;
    	}
    	count++;

		// Turn on the LED
		GPIO_PinOutSet(LED_PORT, LED_BIT);

		if (!TD_LAN_SendReceive(-1, 1, &TX, &RX)) {
            TD_RTC_Delay(T50MS);
            printf("NACK=%d\r\n", ++nack);
        } else {
            TD_RTC_Delay(T50MS);
            printf("ACK=%d\r\n", ++ack);
    		tfp_dump("RX ACK=", (uint8_t *) &RX, TD_LAN_FRAME_SIZE);
        }

		// Turn off the LED
		GPIO_PinOutClear(LED_PORT, LED_BIT);

		// Random delay up to 10 seconds
        //delay = rand() % 10000;
		delay = 1000;
        TD_RTC_Delay(delay * T1MS + T500MICROS);
	}

    // Never reached
}

/***************************************************************************//**
 * @brief
 *   User loop function.
 ******************************************************************************/
void TD_USER_Loop(void)
{
	// Never called
}
