/***************************************************************************//**
 * @file
 * @brief Accelerometer API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.0.1
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2013-2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
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
#include <stdint.h>

#include <em_gpio.h>

#include <td_spi.h>
#include <td_core.h>
#include <td_module.h>
#include <td_trap.h>
#include <td_rtc.h>
#include <td_spi.h>
#include <td_flash.h>
#include <td_printf.h>
#include <td_sensor.h>

#include "lis3dh.h"
#include "td_accelero.h"

/***************************************************************************//**
 * @addtogroup TD_ACCELERO Accelerometer
 * @brief Accelerometer API for the TDxxxx RF modules.
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *******************************
 ******************************************************************************/

/** @addtogroup TD_ACCELERO_DEFINES
 * @{ */

/** Turn on trace mode if tfp_printf not commented */
#define DEBUG_PRINTF(...) /*tfp_printf(__VA_ARGS__)*/

/** Max number of samples that can be read at once. 32 is FIFO size */
#define ACCELERO_BUF_SIZE	32

/** Soft high pass filter with Fc=1/(2*Pi*(ALPHA/10)) */
#define HIGH_PASS_ALPHA 1

/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup TD_ACCELERO_LOCAL_VARIABLES Local Variables
 * @{ */

/** Accelerometer data callback function pointer */
static void (*DataCallback)(TD_ACCELERO_Data_t data[32], uint8_t count, bool overrun);

/** Accelerometer event callback function pointer */
static void (*EventCallback)(uint8_t source);

/** Current Accelerometer scale */
//static TD_ACCELERO_Scales_t CurrentScale = TD_ACCELERO_2G;

/** Current High-pass filter enable flag */
static bool Filter = false;

/** Accelerometer configuration for the TDxxxx RF module */
static  TD_ACCELERO_Config_t TD_Accelero;

/** Configuration flag */
static bool ConfigInit = false;

/** Current Interrupt flag */
static bool AcceleroIrq = false;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_ACCELERO_FUNCTIONS Private Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Accelerometer event IRQ handler. Called by Sensor SwitchIrq Monitoring.
 *
 * @param[in] port
 *    Sensor parameter. Unused
 *
 * @param[in] bit
 *    Sensor parameter. Unused
 *
 * @param[in] state
 *   IRQ Pin state
 *
 * @return
 *   Returns false.
 ******************************************************************************/
static bool TD_ACCELERO_Irq(GPIO_Port_TypeDef port, unsigned int bit, bool state)
{
	if (state) {
		AcceleroIrq = true;
		TD_WakeMainLoop();
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *   Soft High pass as accelerometer is bugged on Z.
 *
 * @param data
 *  Pointer to accelerometry data array to filter
 *
 * @param count
 *  Array data count.
 ******************************************************************************/
static void TD_ACCELERO_HighPassFilter(TD_ACCELERO_Data_t *data, uint8_t count)
{
	int i;
	static int16_t xmem = 0, ymem = 0, zmem = 0, xf = 0, yf = 0, zf = 0;

	for (i = 0; i < count; i++) {
		xf = (HIGH_PASS_ALPHA * (xf + data[i].x - xmem)) / 10;
		xmem = data[i].x;
		yf = (HIGH_PASS_ALPHA * (yf + data[i].y - ymem)) / 10;
		ymem = data[i].y;
		zf = (HIGH_PASS_ALPHA * (zf + data[i].z - zmem)) / 10;
		zmem = data[i].z;
		data[i].x = xf;
		data[i].y = yf;
		data[i].z = zf;
	}
}

/***************************************************************************//**
 * @brief
 *   Power down the accelerometer.
 ******************************************************************************/
static void TD_ACCELERO_PowerDown(void)
{
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		TD_LIS3DH_Configure(TD_ACCELERO_POWER_DOWN, 0, 0, 0);
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
}

/***************************************************************************//**
 * @brief
 *   Set the accelerometer in normal power mode.
 *
 * @param[in] rate
 *    The accelerometer sampling rate taken from the available TD_ACCELERO_Rates.
 *
 * @param[in] axis
 *    The accelerometer axis mask.
 *
 * @param[in] scale
 *   The accelerometer measurement scale taken from the available
 *   TD_ACCELERO_Scales.
 ******************************************************************************/
static void TD_ACCELERO_NormalPower(TD_ACCELERO_Rates_t rate, uint8_t axis, TD_ACCELERO_Scales_t scale)
{
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		//CurrentScale = scale;
		TD_LIS3DH_Configure(TD_ACCELERO_NORMAL_POWER, rate, axis, scale);
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
}

/***************************************************************************//**
 * @brief
 *   Set the accelerometer in low power mode.
 *
 * @param[in] rate
 *    The accelerometer sampling rate taken from the available TD_ACCELERO_Rates.
 *
 * @param[in] axis
 *    The accelerometer axis mask.
 *
 * @param[in] scale
 *   The accelerometer measurement scale taken from the available
 *   TD_ACCELERO_Scales.
 ******************************************************************************/
static void TD_ACCELERO_LowPower(TD_ACCELERO_Rates_t rate, uint8_t axis, TD_ACCELERO_Scales_t scale)
{
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		//CurrentScale = scale;
		TD_LIS3DH_Configure(TD_ACCELERO_LOW_POWER, rate, axis, scale);
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
}

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_ACCELERO_GLOBAL_FUNCTIONS Global Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Dump all accelerometer registers for debug purposes.
 ******************************************************************************/
void TD_ACCELERO_Dump(void)
{
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		TD_LIS3DH_Dump();
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
}

/** @} */

/** @addtogroup TD_ACCELERO_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Initialize the accelerometer. Must be called in User_Setup for proper
 *   Accelerometer operation. Will startup monitoring if previously applied using
 *   SetConfig.
 *
 * @return
 *   Returns true if the accelerometer initialized successfully, false otherwise.
 ******************************************************************************/
bool TD_ACCELERO_Init(void)
{
	bool ret = false;

	// Register the accelerometer on the SPI bus
	TD_SPI_Register(ACCELERO_SPI_ID, 0xFF, 0, 10000000, usartClockMode3);

	// Lock SPI bus
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {

		// Driver initialization and register read
		if (TD_LIS3DH_Init()) {
			ret = true;
		}
		if (ret) {

			// If config was loaded using setconfig, use it, otherwise initialize
			if (ConfigInit) {
				switch (TD_Accelero.monitoring) {
				case TD_ACCELERO_MONITOR_DATA:
					TD_ACCELERO_MonitorEvent(false, (TD_ACCELERO_Rates_t) 0, 0, (TD_ACCELERO_Scales_t) 0, 0, 0, 0, 0, 0);
					TD_ACCELERO_MonitorData(true,
						TD_Accelero.config.low_power,
						TD_Accelero.config.rate,
						TD_Accelero.data_config.axis,
						TD_Accelero.config.scale,
						TD_Accelero.config.filter,
						(TD_ACCELERO_FifoModes_t) 2,
						0,
						TD_Accelero.data_config.user_callback);
					TD_LIS3DH_ClearIRQ();
					break;

				case TD_ACCELERO_MONITOR_EVENT:
					TD_ACCELERO_MonitorData(false, false, (TD_ACCELERO_Rates_t) 0, 0, (TD_ACCELERO_Scales_t) 0, 0, (TD_ACCELERO_FifoModes_t) 0, 0, 0);
					TD_ACCELERO_MonitorEvent(true,
						TD_Accelero.config.rate,
						TD_Accelero.event_config.axis,
						TD_Accelero.config.scale,
						TD_Accelero.event_config.event,
						TD_Accelero.event_config.threshold,
						TD_Accelero.event_config.duration,
						TD_Accelero.config.filter,
						TD_Accelero.event_config.user_callback);
					TD_LIS3DH_ClearIRQ();
					break;

				default:
					TD_ACCELERO_PowerDown();
					break;
				}

			} else {
				TD_ACCELERO_PowerDown();
				TD_Accelero.monitoring = TD_ACCELERO_NO_MONITORING;
			}
		}
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
	return ret;
}

/***************************************************************************//**
 * @brief
 *   Return accelerometer configuration
 *
 * @return cutoff
 *    Return pointer to accelerometer configuration structure
 ******************************************************************************/
TD_ACCELERO_Config_t *TD_ACCELERO_GetConfig(void)
{
	return &TD_Accelero;
}

/***************************************************************************//**
 * @brief
 *   Set accelerometer configuration but does NOT apply it
 *
 * @param[in] config
 *    Pointer to the accelerometer configuration structure
 *    containing new configuration
 ******************************************************************************/
void TD_ACCELERO_SetConfig(TD_ACCELERO_Config_t *config)
{
	memcpy(&TD_Accelero, config, sizeof (TD_ACCELERO_Config_t));
	ConfigInit = true;
}

/***************************************************************************//**
 * @brief
 *   Setup accelerometer event monitoring.
 *
 * @param[in] enable
 *    Enable accelerometer event monitoring if true, disable if false.
 *
 * @param[in] rate
 *    The accelerometer sampling rate taken from the available TD_ACCELERO_Rates.
 *
 * @param[in] axis
 *    The accelerometer axis mask.
 *
 * @param[in] scale
 *   The accelerometer measurement scale taken from the available
 *   TD_ACCELERO_Scales.
 *
 * @param[in] event
 *    The event mask to monitor.
 *
 * @param[in] threshold
 *    The threshold value to trigger an event.
 *
 * @param[in] duration
 *    The minimum duration to trigger an event.
 *
 * @param[in] filter
 *   Accelerometer high-pass filter enable flag: enabled if true, disabled if false.
 *
 * @param[in] callback
 *   Pointer to the function to call back whenever an event occurs. This function
 *   will receive the event source mask as its argument.
 ******************************************************************************/
void TD_ACCELERO_MonitorEvent(bool enable, TD_ACCELERO_Rates_t rate, uint8_t axis, TD_ACCELERO_Scales_t scale, uint8_t event, uint8_t threshold, uint8_t duration, int8_t filter, void (*callback)(uint8_t source))
{
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		if (enable)	{
			TD_Accelero.monitoring = TD_ACCELERO_MONITOR_EVENT;
			TD_Accelero.config.low_power = true;
			TD_Accelero.config.rate = rate;
			TD_Accelero.config.scale = scale;
			TD_Accelero.config.filter = filter;
			TD_Accelero.event_config.axis = axis;
			TD_Accelero.event_config.threshold = threshold;
			TD_Accelero.event_config.duration = duration;
			TD_Accelero.event_config.event = event;
			TD_Accelero.event_config.user_callback = callback;
			EventCallback = callback;
			Filter = (filter > 0 ? true : false);
			TD_LIS3DH_ConfigHighPass(Filter, filter);
			TD_SENSOR_MonitorSwitchIrq(true, USR4_PORT, USR4_BIT, 0, 1, 0, 0, TD_ACCELERO_Irq);
			TD_ACCELERO_LowPower(rate, axis, scale);
			TD_LIS3DH_ConfigureIRQ(LIS3DH_IRQ_AOI1, event, threshold, duration);

			// Usually the pin is up before monitoring the switch so the IRQ is missed
			if (GPIO_PinInGet(USR4_PORT, USR4_BIT)) {
				AcceleroIrq = true;
				TD_WakeMainLoop();
			}
		} else {
			TD_SENSOR_MonitorSwitchIrq(false, USR4_PORT, USR4_BIT, 0, 0, 0, 0, 0);
			TD_ACCELERO_PowerDown();
			TD_LIS3DH_ClearIRQ();
			TD_Accelero.monitoring = TD_ACCELERO_NO_MONITORING;
		}
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
}

/***************************************************************************//**
 * @brief
 *   Setup accelerometer data monitoring. This function will give you access to
 *   raw xyz data from the accelerometer.
 *
 * @param[in] enable
 *    Enable accelerometer data monitoring if true, disable if false.
 *
 * @param[in] low_power
 *     Enable low-power mode if true, disable if false.
 *
 * @param[in] rate
 *    The accelerometer sampling rate taken from the available TD_ACCELERO_Rates.
 *
 * @param[in] axis
 *    The accelerometer axis mask.
 *
 * @param[in] scale
 *   The accelerometer measurement scale taken from the available
 *   TD_ACCELERO_Scales.
 *
 * @param[in] filter
 *   Accelerometer high-pass filter enable flag: enabled if true, disabled if false.
 *
 * @param[in] fifo
 *   The FIFO mode to use for the data.
 *
 * @param[in] watermark
 *   The number of bytes to wait before triggering an IRQ.
 *
 * @param[in] callback
 *   Pointer to the function to call back whenever new data arrives. This function
 *   will receive a xyz array with up to 32 accelerometer values as well as data
 *   count and an overrun flag as arguments.
 ******************************************************************************/
void TD_ACCELERO_MonitorData(bool enable, bool low_power, TD_ACCELERO_Rates_t rate, uint8_t axis, TD_ACCELERO_Scales_t scale, int8_t filter, TD_ACCELERO_FifoModes_t fifo, uint8_t watermark, void (*callback)(TD_ACCELERO_Data_t data[32], uint8_t count, bool overrun))
{
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		if (callback != 0 && enable) {
			TD_Accelero.monitoring = TD_ACCELERO_MONITOR_DATA;
			TD_Accelero.config.low_power = low_power;
			TD_Accelero.config.rate = rate;
			TD_Accelero.config.scale = scale;
			TD_Accelero.config.filter = filter;
			TD_Accelero.data_config.axis = axis;
			TD_Accelero.data_config.user_callback = callback;
			DataCallback = callback;
			// TD_LIS3DH_ClearIRQ();
			TD_SENSOR_MonitorSwitchIrq(true, USR4_PORT, USR4_BIT, 0, 1, 0, 0, TD_ACCELERO_Irq);
			// TD_LIS3DH_ConfigureIRQ(LIS3DH_IRQ_DRDY1,0,0,0);

			// Use stream mode and set IRQ as soon as a sample is available
			TD_LIS3DH_SetFIFOMode(fifo, watermark);
			Filter = (filter > 0 ? true : false);
			if (low_power) {
				TD_ACCELERO_LowPower(rate, axis, scale);
			} else {
				TD_ACCELERO_NormalPower(rate, axis, scale);
			}

			// Usually the pin is up before monitoring the switch so the IRQ is missed
			if (GPIO_PinInGet(USR4_PORT, USR4_BIT)) {
				AcceleroIrq = true;
				TD_WakeMainLoop();
			}
		} else if (!enable) {
			TD_SENSOR_MonitorSwitchIrq(false, USR4_PORT, USR4_BIT, 0, 0, 0, 0, 0);
			TD_ACCELERO_PowerDown();
			TD_LIS3DH_ClearIRQ();
			TD_Accelero.monitoring = TD_ACCELERO_NO_MONITORING;
		}
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
}

/***************************************************************************//**
 * @brief
 *   Allow reading one register of the accelerometer.
 *
 * @param[in] address
 *  Register address to read the value from.
 *
 * @return[in] value
 *  Returns the read value.
 ******************************************************************************/
uint8_t TD_ACCELERO_ReadRegister(uint8_t address)
{
	uint8_t value = 0;

	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		value = TD_LIS3DH_ReadRegister(address);
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
	return value;
}

/***************************************************************************//**
 * @brief
 *   Allow writing to the accelerometer register
 *
 * @param[in] address
 *  Register address to write to.
 *
 * @param[in] value
 *  Value to write.
 ******************************************************************************/
void TD_ACCELERO_WriteRegister(uint8_t address, uint8_t value)
{
	if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
		TD_LIS3DH_WriteRegister(address, value);
		TD_SPI_UnLock(ACCELERO_SPI_ID);
	} else {
		TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
	}
}

/***************************************************************************//**
 * @brief
 *   Accelero process. Will read data from accelerometer if interrupt flag
 *   is set for current monitoring. Will call user callback. Must be called
 *   in User_Loop for proper accelerometer operation.
 ******************************************************************************/
void TD_ACCELERO_Process(void)
{
	TD_ACCELERO_Data_t data[ACCELERO_BUF_SIZE];
	int count = 0;
	bool overrun = false;
	uint16_t x, y, z;
	uint8_t source = 0;
	uint8_t st;

	if (AcceleroIrq) {
		AcceleroIrq = false;
		if (TD_Accelero.monitoring == TD_ACCELERO_MONITOR_EVENT) {
			//tfp_printf("ac irq pin checked %d %d\r\n",GPIO_PinInGet(USR4_PORT,USR4_BIT),source);
			if (EventCallback != 0) {

				// If there is no callback IRQ handling is made somewhere else
				// so don't kill IRQ source unless we have a callback
				if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {

					// Clear event flag on accelerometer and allow IRQ renewal
					source = TD_LIS3DH_GetEventSource();
					TD_SPI_UnLock(ACCELERO_SPI_ID);
				} else {
					TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
				}
				(*EventCallback)(source&TD_Accelero.event_config.event);
			}
			if (Filter) {
				if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
					TD_LIS3DH_SetFilterRef();
					TD_SPI_UnLock(ACCELERO_SPI_ID);
				} else {
					TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
				}
			}
		} else if (TD_Accelero.monitoring == TD_ACCELERO_MONITOR_DATA) {
			AcceleroIrq = false;
			if (DataCallback != 0) {

				// Empty FIFO by reading all data
				if (TD_SPI_Lock(ACCELERO_SPI_ID, NULL)) {
					overrun = TD_LIS3DH_GetFIFOStatus() & 40;

					//Discards first -> output buffer is only changed when FIFO
					// has new data so first value always contains same value
					// as last sample of last reading.
					TD_LIS3DH_GetXYZ(&x, &y, &z);

					// While not empty
					while (count < ACCELERO_BUF_SIZE) {
						TD_LIS3DH_GetXYZ(&x, &y, &z);
						data[count].x = ((short int) x) /** CurrentScale * 1000) >> 15*/;
						data[count].y = ((short int) y) /** CurrentScale * 1000) >> 15*/;
						data[count].z = ((short int) z) /** CurrentScale * 1000) >> 15*/;
						count++;
						st = TD_LIS3DH_GetFIFOStatus();

						// If (!(TD_LIS3DH_GetFIFOStatus() & 0x20))
						// As of 07/11/2013 Empty flag seem broken and sometimes appear when FIFO is not empty
						// At 400Hz (slow read of FIFO : 22ms for full fifo) seem to appear after 10 to 30 s
						// We must go below watermark
						// If we absolutely want to make sure all samples are read, FIFO level can also be checked
						// "Not Watermark" And "Empty"
						if ((st & 0xA0) == 0x20) {
							break;
						}
					}

					// There are still samples but buffer overflows: need an other turn
					if (GPIO_PinInGet(USR4_PORT, USR4_BIT) && count != 32) {
						DEBUG_PRINTF("IT not clear CNT:%d st:0x%02X!!\r\n", count, st);
					}
					if (count >= ACCELERO_BUF_SIZE) {
						AcceleroIrq = true;
						TD_WakeMainLoop();
					}

					// If filter is activated
					if (Filter) {
						//TD_LIS3DH_SetFilterRef();
						TD_ACCELERO_HighPassFilter(data, count);
					}
					TD_SPI_UnLock(ACCELERO_SPI_ID);

					// Push all data to callback
					(*DataCallback)(data, count, overrun);
				} else {
					TD_Trap(TRAP_SPI_NOT_AVAILABLE, __LINE__);
				}
			}
		}
	}
}

/** @} */

/** @} (end addtogroup TD_ACCELERO) */
