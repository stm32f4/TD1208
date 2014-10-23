/******************************************************************************
 * @file
 * @brief template file for TDxxxx RF modules.
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

#include <stdbool.h>

#include <efm32.h>

#include <td_core.h>
#include <td_flash.h>
#include <td_rtc.h>
#include <td_printf.h>
#include <td_uart.h>
#include <td_scheduler.h>
#include <td_sensor.h>
#include <td_sensor_lan.h>
#include <td_sensor_device.h>
#include <sensor_register.h>


/* This file declare all "dynamic" library data. It should be last included file
 * Standard size value can be override before including this file
 */
#define TD_GEOLOC_USE_CODE 0
#include <td_config.h>

/*******************************************************************************
 ******************************  DEFINES  ************************************
 ******************************************************************************/

/** Define your own device class */
#define DEVICE_CLASS 0x0000

/** Registration retry interval in seconds */
#define REGISTRATION_RETRY_INTERVAL 5

/** Registration retries, 255 for infinite */
#define REGISTRATION_RETRIES 5

/*******************************************************************************
 ******************************  VARIABLES  ************************************
 ******************************************************************************/

/** Registration timer id */
uint8_t RegistrationTimer = 0xFF;

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *  Switch LED ON/OFF.
 *
 * @param[in] state
 *  If true, switch ON the LED. If Flase switch it OFF.
 ******************************************************************************/
static void SetLed(bool state)
{
	if (state) {

		// Switch on the LED
		GPIO_PinOutSet(TIM2_PORT, TIM2_BIT);
	} else {

		// Switch off the LED
		GPIO_PinOutClear(TIM2_PORT, TIM2_BIT);
	}
}

/***************************************************************************//**
 * @brief
 *  Activate all monitoring here.
 ******************************************************************************/
static void ApplyMonitoring(void)
{
	//TD_SENSOR_MonitorTemperature(true, 3600, 0, 300, 0);
}

/***************************************************************************//**
 * @brief
 *  Actions if registration gives up.
 ******************************************************************************/
static void RegistrationGiveUp(void)
{
	tfp_printf("Registration Give up\r\n");
}

/***************************************************************************//**
 * @brief
 *  Actions if registration failed.
 ******************************************************************************/
static void RegistrationFailed(void)
{
	tfp_printf("Registration Failed\r\n");
}

/***************************************************************************//**
 * @brief
 *  Actions if registration succeeded.
 ******************************************************************************/
static void RegistrationSuccess(void)
{
	SetLed(true);
	tfp_printf("Registration Success\r\n");

	// Beware: as long as the gateway is still in registration mode, it can't
	// communicate with the newly registered device, so wait a bit for the gateway
	// to stop registration
	TD_RTC_Delay(T2S);

	// Send registration
	TD_SENSOR_SendRegister();
	ApplyMonitoring();
	SetLed(false);
}

/***************************************************************************//**
 * @brief
 *  Perform a registration retry.
 *
 * @param[in] argument
 *  Argument passed by Scheduler.
 *
 * @param[in] repetition
 *  Remaining repetition count passed by Scheduler.
 ******************************************************************************/
static void RegistrationRetry(uint32_t argument, uint8_t repetition)
{
	// No more repetition, invalidate the timer ID
	if (repetition == 0) {
		RegistrationTimer = 0xFF;
	}

	// Try to register
	if (TD_SENSOR_DEVICE_Register() != ACK_OK) {

		// Failed! If no more retry, give up
		if (repetition > 0) {
			RegistrationFailed();
		} else {
			RegistrationGiveUp();
		}
	} else {

		// Success! Remove timer if it is still activated
		if (RegistrationTimer != 0xFF) {
			TD_SCHEDULER_Remove(RegistrationTimer);
			RegistrationTimer = 0xFF;
		}
		RegistrationSuccess();
	}
}

/***************************************************************************//**
 * @brief
 *   User setup function.
 ******************************************************************************/
void TD_USER_Setup(void)
{

	// Initialize the UART console
	init_printf(TD_UART_Init(9600, true, false),
		    		TD_UART_Putc,
		    		TD_UART_Start,
		    		TD_UART_Stop);

	// Changing the version will clear all flash content thus allowing
	// resetting the device
	TD_FLASH_SetVariablesVersion(0);
	tfp_printf("Device Start-up\r\n");

	// Set Device Class
	TD_SENSOR_SetDeviceClass(DEVICE_CLASS);

	// Initialize the module as a LAN device on 869Mhz at 14dB.
	if (TD_SENSOR_Init(SENSOR_DEVICE, 869000000, 14)) {
		tfp_printf("Sensor ok\r\n");
	}

	// Registration will be saved in flash, after reboot don't try to
	// register again
	if (!TD_SENSOR_DEVICE_isRegistered()) {
		tfp_printf("Not registered\r\n");

		// Register on the Gateway network, can't communicate otherwise
		if (TD_SENSOR_DEVICE_Register() != ACK_OK) {
			RegistrationFailed();

			// If registration failed, retry forever
			RegistrationTimer = TD_SCHEDULER_Append(REGISTRATION_RETRY_INTERVAL,
				0,
				0,
				REGISTRATION_RETRIES,
				RegistrationRetry,
				0);
		} else {
			RegistrationSuccess();
		}
	} else {
		tfp_printf("Registered\r\n");
		TD_SENSOR_SendRegister();
		ApplyMonitoring();
	}
}

/***************************************************************************//**
 * @brief
 *   User loop function.
 ******************************************************************************/
void TD_USER_Loop(void)
{
	TD_LAN_Process();
	TD_SENSOR_Process();

	if (TD_SENSOR_DEVICE_isRegistered()) {

		// Handle here post-registration events

	} else {

		// Handle here pre-registration events
	}
}
