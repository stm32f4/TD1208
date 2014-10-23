/******************************************************************************
 * @file
 * @brief User Application implementation for TD12xx RF module.
 * @author Telecom Design S.A.
 * @version 2.0.0
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
 *****************************************************************************/

#include "config.h"

#include <efm32.h>

#include <td_module.h>
#include <td_core.h>
#include <td_flash.h>
#include <td_uart.h>
#include <td_printf.h>
#include <at_parse.h>
#include <bmp180_i2cdrv.h>

#if AT_CORE
#include <at_core.h>
#endif

#if AT_LAN_RF
#include <at_lan.h>
#include <td_lan.h>
#endif
#if AT_RADIO_INFO
#include <at_radio.h>
#endif
#if AT_SIGFOX
#include <at_sigfox.h>
#endif

#if AT_SENSOR
#include <at_sensor.h>
#include <at_sensor_lan.h>
#include <at_sensor_send.h>
#endif

#if AT_GEOLOC
#include <at_geoloc.h>
#include <at_accelero.h>
#endif

#if defined(SIGFOX_V1) && defined(SIGFOX_DOWNLINK)
#include <td_sigfox.h>
#endif

/* This file declare all "dynamic" library data. It should be last included file
 * Standard size value can be override before including this file
 */
#if MODULE_REVISION == REVISION_TD1202
#define TD_ALL_DUMP_REMOVE_CODE
#endif

#if AT_SENSOR
#define TD_SENSOR_GATEWAY_MAX_DEVICE 5
#endif

#define TD_TRAP_RESET_CODE
#define PRODUCT_LED_BLINK 1
#include <td_config.h>

#define USE_PRINTF

/******************************************************************************
 *************************  Variables declaration   ****************************
 ******************************************************************************/

// Task ID for LED blink
uint8_t Scheduler_LED_Id;

// Task ID for BMP180 Acquisition
uint8_t Scheduler_BMP180_Id;

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

void TD_USER_Heartbeat(uint32_t arg, uint8_t repetition);
void TD_USER_Measure(uint32_t arg, uint8_t repetition);

/**
 * @brief  User setup function
 **/
void TD_USER_Setup(void) {
	// Initialize the LEUART
	init_printf(TD_UART_Init(9600, true, false), TD_UART_Putc, TD_UART_Start,
			TD_UART_Stop);

	// Define variables version to avoid wrong flash init
	TD_FLASH_SetVariablesVersion(FLASH_VARIABLES_VERSION);

#if AT_CORE
	AT_AddExtension(&core_extension);
#endif
#if AT_RADIO_INFO
	AT_AddExtension(&radio_extension);
#endif
#if AT_SIGFOX
	AT_AddExtension(&sigfox_extension);
#endif
#if AT_LAN_RF
	AT_AddExtension(&lan_extension);
#endif

#if AT_SENSOR
	AT_AddExtension(&sensor_extension);
	AT_AddExtension(&sensor_lan_extension);
	AT_AddExtension(&sensor_send_extension);
#endif

#if AT_GEOLOC
	AT_AddExtension(&geoloc_extension);
	AT_AddExtension(&accelero_extension);
#endif

	// Initialize the AT command parser
	AT_Init();

#if AT_SENSOR

	//Initialize Sensor
	TD_SENSOR_Init(TD_SENSOR_GetModuleType(), TD_LAN_GetFrequency(),
			TD_LAN_GetPowerLevel());
#endif

#if AT_GEOLOC

	//Initialize GPS
	TD_GEOLOC_Init();

	//Initialize Accelero
	TD_ACCELERO_Init();
#endif

	// Register a task for the heartbeat
	Scheduler_LED_Id = TD_SCHEDULER_Append(10, 0, 0, 0xFF, TD_USER_Heartbeat,
			1);

	// Register a task for the heartbeat
	Scheduler_BMP180_Id = TD_SCHEDULER_Append(30, 0, 0, 0xFF, TD_USER_Measure,
			0);
}

/**
 * @brief  User loop function
 **/
void TD_USER_Loop(void) {
	int c;

#if AT_LAN_RF

	// Process local RF
	TD_LAN_Process();
#endif

#if defined(SIGFOX_V1) && defined(SIGFOX_DOWNLINK)
	TD_SIGFOX_DOWNLINK_Process();
#endif

#if AT_SENSOR
	TD_SENSOR_Process();
#endif

#if AT_GEOLOC
	TD_GEOLOC_Process();
	TD_ACCELERO_Process();
#endif

	while ((c = TD_UART_GetChar()) >= 0) {
		// LED On
		GPIO->P[3].DOUTSET = 1 << ADC0_BIT;

		AT_Parse(c);

		// LED Off
		GPIO->P[3].DOUTCLR = 1 << ADC0_BIT;

	}
}

/**
 * @brief  Toggle LED
 **/
void TD_USER_Heartbeat(uint32_t arg, uint8_t repetition) {
	GPIO->P[3].DOUTTGL = 1 << ADC0_BIT;
	if (arg == 0) {
		// LED Off
		GPIO->P[3].DOUTCLR = 1 << ADC0_BIT;
		TD_SCHEDULER_SetArg(Scheduler_LED_Id, 10);
		TD_SCHEDULER_SetInterval(Scheduler_LED_Id, 10, 0, 1);
	} else {
		// LED On
		GPIO->P[3].DOUTSET = 1 << ADC0_BIT;
		TD_SCHEDULER_SetArg(Scheduler_LED_Id, 0);
		TD_SCHEDULER_SetInterval(Scheduler_LED_Id, 0, 0x100, 0);
	}
}

/**
 * @Brief Acquires temperature and pressure
 **/
void TD_USER_Measure(uint32_t arg, uint8_t repetition) {
	typedef struct structMes {
		float temp;
		int32_t press;
	} measures;

	typedef union {
		measures mes;
		uint8_t bmes[8];
	} allmes;

	allmes measure;

	//  Fist time just do I2C init
	if (arg == 0) {
		// Initialize bmp180
		I2CDRV_Init();
		BMP180_GetCalData();
		TD_SCHEDULER_SetArg(Scheduler_BMP180_Id, 1);
		tfp_printf("BMP180 Initialized.\r\n");
	}

	// Only send if counter is 1
	if (arg == 1) {
		// Measure pressure and temp
		int32_t pressure = 0;
		int32_t temperature = 0;
		BMP180_CalculateTempPressure(BMP180_GetTempData(),
				BMP180_GetPressureData(), &temperature, &pressure);

		// Prepare the buffer
		measure.mes.press = pressure;
		measure.mes.temp = temperature;
		measure.mes.temp = measure.mes.temp / 10.0f;

		// Trace to serial
		tfp_printf("Temp: %X %X %X %X Press: %X %X %X %X\r\n", measure.bmes[0],
				measure.bmes[1], measure.bmes[2], measure.bmes[3],
				measure.bmes[4], measure.bmes[5], measure.bmes[6],
				measure.bmes[7]);

		// Send to sigfox
		GPIO->P[3].DOUTSET = 1 << ADC0_BIT;
		TD_SIGFOX_Send(measure.bmes, 8, 2);
		GPIO->P[3].DOUTCLR = 1 << ADC0_BIT;
		tfp_printf("Data sent to sigfox.\r\n");
	}

	// When counter reaches 60 send again next time
	if (arg == 60) {
		arg = 0;
	}

	// increment counter
	arg++;

	// Update counter
	TD_SCHEDULER_SetArg(Scheduler_BMP180_Id, arg);
}
