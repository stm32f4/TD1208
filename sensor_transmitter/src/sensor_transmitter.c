/******************************************************************************
 * @file
 * @brief Sensor Transmitter Application Example
 * @author Telecom Design S.A.
 * @version 1.0.1
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

#include <td_scheduler.h>
#include <td_uart.h>
#include <td_measure.h>
#include <td_flash.h>
#include <td_core.h>
#include <td_printf.h>
#include <td_module.h>
#include <td_gpio.h>
#include <td_sensor.h>
#include <sensor_event.h>
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
#define DEVICE_CLASS 0x0001

 /** Battery low level in mV */
#define BATTERY_LEVEL_LOW 2600

/* Battery OK level in mV */
#define BATTERY_LEVEL_OK 3100

//** Temperature low level in 1/10 degrees */
#define TEMPERATURE_LEVEL_LOW 50

/** Temperature high level in 1/10 degrees */
#define TEMPERATURE_LEVEL_HIGH 550

 /** Temperature checking interval in seconds */
#define TEMPERATURE_CHECKING_INTERVAL 20

/** Keepalive interval in hours */
#define KEEPALIVE_INTERVAL 24

/*******************************************************************************
 ******************************  VARIABLES  ************************************
 ******************************************************************************/

/** LED Timer IED */
static uint8_t LedTimer=0xFF;

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *  Switch LED ON/OFF
 *
 * @param[in] state
 *  If true, switch ON the LED. If false switch it OFF.
 ******************************************************************************/
static void SetLed(bool state)
{
	if (state) {

		// Turn on the LED
		GPIO_PinOutSet(TIM2_PORT, TIM2_BIT);
	} else {

		// Turn off the LED
		 GPIO_PinOutClear(TIM2_PORT, TIM2_BIT);
	}
}

/***************************************************************************//**
 * @brief
 *  LED blinking. This function is being called by the Scheduler.
 *
 * @param[in] arg
 *  Argument passed by Scheduler. Used to toggle the led.
 ******************************************************************************/
static void LedBlink(uint32_t arg, uint8_t repetition)
{
	GPIO_PinOutToggle(TIM2_PORT, TIM2_BIT);
}

/***************************************************************************//**
 * @brief
 *   Battery Callback. Called on battery Event.
 *   Switch on blinking the blue LED if battery state is low.
 *   Switch off blinking if battery state goes back to OK.
 ******************************************************************************/
static bool BatteryCallback(bool state, uint16_t level)
{
	tfp_printf("Battery Callback, state %d, level %d\r\n",state,level);

	// If battery level is low
	if (!state) {

		// Setup for blinking every seconds in interrupt
		LedTimer = TD_SCHEDULER_AppendIrq(1, 0, 0, 0xFF, LedBlink, 0);
	} else {

		// If a timer was started
		if (LedTimer != 0xFF) {

			// Stop it
			TD_SCHEDULER_Remove(LedTimer);
			LedTimer = 0xFF;
		}

		// Set LED off
		SetLed(false);
	}

	// Send a SENSOR RF frame
	return true;
}

/***************************************************************************//**
 * @brief
 * Temperature callback. Called on Temperature Events.
 ******************************************************************************/
static bool TemperatureCallback(TD_SENSOR_TemperatureState_t state, int16_t level)
{
	tfp_printf("Temperature Callback, state %d, level %d.C\r\n",state,level/10);

	// Send a SENSOR RF frame
	return true;
}

/***************************************************************************//**
 * @brief
 * Switch callback. Called on switch Events.
 ******************************************************************************/
bool SwitchCallback(GPIO_Port_TypeDef port, unsigned int bit, bool state)
{
	tfp_printf("Switch Callback, port: %d - bit: %d - state: %d\r\n", port, bit, state);

	// Don't send a SENSOR RF frame
	return false;
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
	tfp_printf("Transmitter Start-up\r\n");

	// Initialize the LED GPIO
	GPIO_PinModeSet(TIM2_PORT, TIM2_BIT, gpioModePushPull, 0);

	// Set the device class
	TD_SENSOR_SetDeviceClass(DEVICE_CLASS);

	// Initialize the device as a transmitter with no local RF configuration.
	TD_SENSOR_Init(SENSOR_TRANSMITTER, 0, 0);

	// Register on Sensor
	TD_SENSOR_SendRegister();

	// Enable battery monitoring and setup a callback on event
	TD_SENSOR_MonitorBattery(true, BATTERY_LEVEL_LOW,
		BATTERY_LEVEL_OK,
		BatteryCallback);

	// Print out current temperature
	tfp_printf("Temperature: %d.C\r\n",
		TD_MEASURE_VoltageTemperatureExtended(true) / 10);

	// Enable temperature monitoring and setup a callback on event
	TD_SENSOR_MonitorTemperature(true,
		TEMPERATURE_CHECKING_INTERVAL,
		TEMPERATURE_LEVEL_LOW,
		TEMPERATURE_LEVEL_HIGH,
		TemperatureCallback);

	// Enable Switch Monitoring and setup a callback on event
	TD_SENSOR_MonitorSwitch(true,
		USR0_PORT,
		USR0_BIT,
		true,
		true,
		true,
		true,
		SwitchCallback);

	// Enable KeepAlive Monitoring
	TD_SENSOR_MonitorKeepAlive(true, KEEPALIVE_INTERVAL);
}

/***************************************************************************//**
 * @brief
 *   User loop function.
 ******************************************************************************/
void TD_USER_Loop(void)
{
	TD_SENSOR_Process();
}
