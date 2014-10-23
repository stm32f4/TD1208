/******************************************************************************
 * @file
 * @brief Sensor Gateway Application example
 * @author Telecom Design S.A.
 * @version 2.0.2
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
#include <td_rtc.h>
#include <td_printf.h>
#include <td_uart.h>
#include <td_flash.h>
#include <td_scheduler.h>
#include <td_sensor.h>
#include <td_sensor_gateway.h>
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

/** Keep-alive in hours */
#define KEEPALIVE_INTERVAL 24

/** Time to wait for registration on start-up */
#define REGISTRATION_INTERVAL 30

/** Maximum expected device */
#define MAX_EXPECTED_DEVICE 1

/** LED timer IED */
static uint8_t LedTimer = 0xFF;

/** Registration timer ID */
static uint8_t RegistrationTimer = 0xFF;

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *  Switch LED ON/OFF
 *
 * @param[in] state
 *  If true, switch ON the LED. If Flase switch it OFF.
 ******************************************************************************/
static void SetLed(bool state)
{
	if (state) {

		// Turn on the LED
		GPIO_PinOutSet(TIM2_PORT, TIM2_BIT);
	} else {
		// Trun off the LED
		GPIO_PinOutClear(TIM2_PORT, TIM2_BIT);
	}
}

/***************************************************************************//**
 * @brief
 *  LED blinking. This function is being called by the Scheduler.
 *
 * @param[in] arg
 *  Argument passed by Scheduler.
 *
 * @param[in] repetition
 *  Remaining repetition count passed by Scheduler.
 ******************************************************************************/
static void led_blink(uint32_t arg, uint8_t repetition)
{
	GPIO_PinOutToggle(TIM2_PORT, TIM2_BIT);
}

/***************************************************************************//**
 * @brief
 *  Stop registration.
 *
 * @param[in] arg
 *  Argument passed by Scheduler.
 *
 * @param[in] repetition
 *  Remaining repetition count passed by Scheduler.
 ******************************************************************************/
static void StopRegistration(uint32_t arg, uint8_t repetition)
{
	TD_SENSOR_GATEWAY_StopRegistration();
	RegistrationTimer = 0xFF;
	if (LedTimer != 0xFF) {
		TD_SCHEDULER_Remove(LedTimer);
	}
	SetLed(false);
	tfp_printf("Registration stopped\r\n");
}

/***************************************************************************//**
 * @brief
 *  Device registration callback..
 *
 * @param[in] lan_address
 *  The LAN address that has been assigned to the device.
 *
 * @param[in] sigfox_id
 *  The received device SIGFOX ID.
 ******************************************************************************/
static void OnRegistration(uint32_t lan_address, uint32_t sigfox_id)
{
	tfp_printf("Device registered %02x - %02x \r\n", lan_address, sigfox_id);

	// Stop registration and registration timer
	if (RegistrationTimer != 0xFF) {
		TD_SCHEDULER_Remove(RegistrationTimer);
		StopRegistration(0,0);
	}
}

/***************************************************************************//**
 * @brief
 *   User setup function.
 ******************************************************************************/
void TD_USER_Setup(void)
{
	uint8_t devices;

	// Initialize the UART console
	init_printf(TD_UART_Init(9600, true, false),
		    		TD_UART_Putc,
		    		TD_UART_Start,
		    		TD_UART_Stop);

	// Changing the version will clear all flash content thus allowing resetting
	// the gateway
	TD_FLASH_SetVariablesVersion(0);
	tfp_printf("Gateway Start-up\r\n");

	// Initialize the LED GPIO
	GPIO_PinModeSet(TIM2_PORT, TIM2_BIT, gpioModePushPull, 0);

	// Set the device class
	TD_SENSOR_SetDeviceClass(DEVICE_CLASS);

	// Initialize the module as a gateway on 869MHz at 14dB
	TD_SENSOR_Init(SENSOR_GATEWAY, 869000000, 14);

	// Register on Sensor
//	TD_SENSOR_SendRegister();

	// Enable keepalive monitoring every day
//	TD_SENSOR_MonitorKeepAlive(true, KEEPALIVE_INTERVAL);

	// Start Reception
	TD_SENSOR_GATEWAY_StartReception();
	devices = TD_SENSOR_GATEWAY_GetDeviceCount();
	tfp_printf("Devices found = %d\r\n",devices);

	// If we are still waiting for devices open registration
	if (devices<MAX_EXPECTED_DEVICE) {

		// Open registration
		TD_SENSOR_GATEWAY_StartRegistration(OnRegistration);
		tfp_printf("Registration opened\r\n");

		// Blink led on interrupt while registration is opened
		LedTimer = TD_SCHEDULER_AppendIrq(0,
			8192 ,
			0,
			TD_SCHEDULER_INFINITE,
			led_blink,
			1);

		// Close registration after REGISTRATION_INTERVAL. One shot timer.
		RegistrationTimer = TD_SCHEDULER_Append(REGISTRATION_INTERVAL,
			0,
			0,
			1,
			StopRegistration,
			0);
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
}
