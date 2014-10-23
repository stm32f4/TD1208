/***************************************************************************//**
 * @file
 * @brief Sensor Monitoring
 * @author Telecom Design S.A.
 * @version 1.1.1
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

#include <stdint.h>
#include <stdbool.h>

#include <em_rmu.h>

#include <td_rtc.h>
#include <td_flash.h>
#include <td_measure.h>
#include <td_gpio.h>
#include <td_utils.h>
#include <td_trap.h>
#include <td_scheduler.h>
#include <td_core.h>
#include <td_printf.h>
#include <td_sigfox.h>

#include "sensor_private.h"
#include "sensor_keepalive.h"
#include "sensor_event.h"
#include "td_sensor.h"
#include "td_sensor_lan.h"
#include "td_sensor_device.h"
#include "td_sensor_gateway.h"
#include "td_sensor_transmitter.h"

/***************************************************************************//**
 * @addtogroup TD_SENSOR Sensor Monitoring
 * @brief Sensor initialization and monitoring functions.
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************  PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_Private_Variables Private Variables
 * @{ */

/** Local module configuration */
static TD_SENSOR_Configuration_t Module;

/** Boot cause given by RMU */
static int32_t BootCause = -1;

/** Custom boot cause loaded from flash */
static uint8_t CustomBootCause;

/** Buffer in which to store configuration that must be persistent (boot monitoring, battery state) */
uint32_t CurrentState[2];

/** Module SIGFOX ID */
uint32_t SigfoxID;

/** Flag to know whether the configuration is already initialized or not */
static bool ConfigInit = false;

/** Flag to know whether the class is already initialized or not */
static bool ConfigClass = false;

/* Unexpected brown-out reboot counter */
static uint32_t BrownOutReboot = 0;

/** Switch state list top index */
static uint8_t TD_SENSOR_SwitchStateList_top = 0;

/** Switch state list bottom index */
static uint8_t TD_SENSOR_SwitchStateList_bottom = 0;

/* External declarations to allow code removal and optimization */
extern TD_SENSOR_SwitchState_t *TD_SENSOR_SwitchStateList;
extern TD_SENSOR_SwitchConfiguration_t *TD_SENSOR_SwitchConfig;
extern TD_SENSOR_TRANSMITTER_Init_t const TD_SENSOR_TRANSMITTER_Init_;
extern TD_SENSOR_TRANSMITTER_Process_t const TD_SENSOR_TRANSMITTER_Process_;
extern TD_SENSOR_MonitorInit_t const TD_SENSOR_MonitorInit_;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Initialize the module local variables.
 ******************************************************************************/
void TD_SENSOR_InternalInit(void)
{
	memset(&Module, 0, sizeof(TD_SENSOR_Configuration_t));

	// Initialize  states
	Module.temperature.state = TEMPERATURE_OK;

	//Initialize timers
	Module.temperature.timer = 0xFF;
	Module.connection.timer = 0xFF;
	Module.keepalive.timer = 0xFF;

	// Initialize dynamic switch data
	Module.switches = TD_SENSOR_SwitchConfig;
	memset(TD_SENSOR_SwitchConfig,0,CONFIG_TD_SENSOR_MAX_SWITCH *
		sizeof (TD_SENSOR_SwitchConfiguration_t));
}

/***************************************************************************//**
 * @brief
 *   Save the module state to Flash memory.
 ****************************************************************************/
static void SaveCurrentStateToFlash(void)
{
	uint32_t temp;

	CurrentState[0] = Module.battery.state & 0x01;
	temp = Module.boot.monitor & 0x01;
	CurrentState[0] |= temp << 1;
	temp = CustomBootCause;
	CurrentState[0] |= temp << 2;
	CurrentState[1] = (uint32_t) Module.boot.user_callback;
	TD_FLASH_WriteVariables();
}

/***************************************************************************//**
 * @brief
 *   Restore the module state from Flash memory.
 ****************************************************************************/
static void LoadCurrentStateFromFlash(void)
{
	if (!TD_FLASH_DeclareVariable((uint8_t *) &CurrentState, sizeof (CurrentState), 0)) {
		Module.battery.state = true;
		Module.boot.monitor = false;
		Module.boot.user_callback = 0;
		CustomBootCause = 0;
	} else {
		Module.battery.state = CurrentState[0] & 0x01;
		Module.boot.monitor = (CurrentState[0] >> 1) & 0x01;
		Module.boot.user_callback = (bool ( *)(void)) CurrentState[1];
		CustomBootCause = (CurrentState[0] >> 2) & 0xFF;
	}
}

/***************************************************************************//**
 * @brief
 *   Read and clear boot cause from RMU.
 ******************************************************************************/
static void ReadBootCause(void)
{
	if (BootCause == -1) {
		BootCause = RMU_ResetCauseGet();
		RMU_ResetCauseClear();
	}
}

/***************************************************************************//**
 * @brief
 *   Process a Sensor state event.
 *
 * @param[in] event
 *	 The Sensor event to process.
 *
 * @param[in] arg
 *   The event-specific argument. Currently, only used for Switch events for
 *   holding switch index and state.
 *
 * @return
 *   Returns true if the event was properly processed (i.e. the frame was
 *   acknowledged by the gateway), false if the Gateway never acknowledged the
 *   frame.
 ******************************************************************************/
static bool TD_SENSOR_EventProcess(TD_SENSOR_Event_t event, uint8_t arg)
{
	bool send_sigfox = true;
	bool acked = false;
	uint8_t switch_index;
	bool switch_state;

	switch (event) {
	case SENSOR_SEND_BATTERY:
		if (Module.battery.user_callback != 0) {
			send_sigfox = (*Module.battery.user_callback)(Module.battery.state, Module.battery.level);
		}
		if (send_sigfox) {
			acked = TD_SENSOR_SendEventBattery(Module.battery.state, Module.battery.level);
		}
		break;

	case SENSOR_SEND_BOOT:
		if (Module.boot.user_callback != 0) {
			send_sigfox = (*Module.boot.user_callback)();
		}
		if (send_sigfox) {
			acked = TD_SENSOR_SendEventBootExt(BootCause & 0xFF, CustomBootCause, 0, 0);
		}
		break;

	case SENSOR_SEND_TEMPERATURE:
		if (Module.temperature.user_callback != 0) {
			send_sigfox = (*Module.temperature.user_callback)(Module.temperature.state, Module.temperature.level);
		}
		if (send_sigfox) {
			acked = TD_SENSOR_SendEventTemperature(Module.temperature.state);
		}
		break;

	// Device only
	case SENSOR_SEND_LOCAL_KEEPALIVE:
		if (Module.type == SENSOR_DEVICE) {
			acked = TD_SENSOR_DEVICE_KeepAlive(true,
				Module.connection.interval,
				Module.rssi.monitor,
				Module.rssi.level_low,
				Module.rssi.level_ok);
		} else if (Module.type == SENSOR_GATEWAY) {
			TD_SENSOR_GATEWAY_SendKeepAliveBroadcast(true,
				Module.connection.interval,
				0,
				0);
		}
		break;

	case SENSOR_SEND_SENSOR_KEEPALIVE:
		acked = TD_SENSOR_SendKeepAlive();
		break;

	case SENSOR_SEND_SWITCH:
		switch_index = (arg & 0x7F);
		switch_state = (arg >> 7);
		if (Module.switches[switch_index].user_callback != 0) {
			send_sigfox =
				(*Module.switches[switch_index].user_callback)(Module.switches[switch_index].port,
						Module.switches[switch_index].bit,
						switch_state);
		}
		if (send_sigfox) {
			acked =
				TD_SENSOR_SendEventSwitch(Module.switches[switch_index].port,
					Module.switches[switch_index].bit,
					switch_state);
		}
		break;

	default:
		break;
	}
	return acked;
}

/***************************************************************************//**
 * @brief
 *   Keep-alive callback function, called by the Scheduler.
 *
 * @param[in] arg
 *	 Timer argument. Not used.
 *
 * @param[in] repetitions
 *	 Timer argument. Not used.
 ******************************************************************************/
static void TD_SENSOR_KeepAliveCallback(uint32_t arg, uint8_t repetitions)
{
	TD_SENSOR_EventProcess(SENSOR_SEND_SENSOR_KEEPALIVE, 0);
}

/***************************************************************************//**
 * @brief
 *   Connection callback function, called by the Scheduler when connection
 *   monitoring is enabled.
 *
 * @param[in] arg
 *	 Timer argument. Not used.
 *
 * @param[in] repetitions
 *	 Timer argument. Not used.
 ******************************************************************************/
static void TD_SENSOR_ConnectionCallBack(uint32_t arg, uint8_t repetitions)
{
	TD_SENSOR_EventProcess(SENSOR_SEND_LOCAL_KEEPALIVE, 0);
}

/***************************************************************************//**
 * @brief
 *   Apply the module configuration and start the related monitoring functions.
 ******************************************************************************/
static void TD_SENSOR_ApplyMonitoringConfiguration(void)
{
	int i;

	if (Module.boot.monitor) {
		TD_SENSOR_EventProcess(SENSOR_SEND_BOOT, 0);
	}
	if (Module.battery.monitor) {
		TD_SENSOR_MonitorBattery(1,
			Module.battery.level_low,
			Module.battery.level_ok,
			Module.battery.user_callback);
	}
	if (Module.temperature.monitor) {
		Module.temperature.monitor = false;
		TD_SENSOR_MonitorTemperature(1,
			Module.temperature.interval,
			Module.temperature.level_low,
			Module.temperature.level_high,
			Module.temperature.user_callback);
	}
	if (Module.connection.monitor) {
		Module.connection.monitor = false;
		TD_SENSOR_MonitorConnection(true, Module.connection.interval);
	}
	if (Module.keepalive.monitor) {
		Module.keepalive.monitor = false;
		TD_SENSOR_MonitorKeepAlive(true, Module.keepalive.interval);
	}
	for (i = 0; i < CONFIG_TD_SENSOR_MAX_SWITCH; i++) {
		if (Module.switches[i].monitor) {
			if (!Module.switches[i].on_irq) {
				TD_SENSOR_MonitorSwitch(true,
					Module.switches[i].port,
					Module.switches[i].bit,
					Module.switches[i].falling,
					Module.switches[i].rising,
					Module.switches[i].pull,
					Module.switches[i].pull_state,
					Module.switches[i].user_callback);
			} else {
				TD_SENSOR_MonitorSwitchIrq(true,
					Module.switches[i].port,
					Module.switches[i].bit,
					Module.switches[i].falling,
					Module.switches[i].rising,
					Module.switches[i].pull,
					Module.switches[i].pull_state,
					Module.switches[i].user_callback);
			}
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Temperature callback function, called by the Scheduler.
 *
 * @param[in] arg
 *	 Timer argument. Not used.
 *
 * @param[in] repetitions
 *	 Timer argument. Not used.
 ******************************************************************************/
static void TD_SENSOR_TemperatureCallBack(uint32_t arg, uint8_t repetitions)
{
	uint8_t prev_state;

	// Measure temperature and save previous state
	Module.temperature.level = (int16_t)TD_MEASURE_TemperatureExtended();
	prev_state = Module.temperature.state;

	// Check state
	if (Module.temperature.level <= Module.temperature.level_low) {
		Module.temperature.state = TEMPERATURE_LOW;
	} else if (Module.temperature.level >= Module.temperature.level_high) {
		Module.temperature.state = TEMPERATURE_HIGH;
	} else {
		Module.temperature.state = TEMPERATURE_OK;
	}

	// If state changed, add an event
	if (Module.temperature.state != prev_state) {
		TD_SENSOR_EventProcess(SENSOR_SEND_TEMPERATURE, 0);
	}
}

/***************************************************************************//**
 * @brief
 *   Switch callback function, called from GPIO IRQ processing.
 *
 * @param mask
 *   Contains the IRQ mask.
 ******************************************************************************/
static void TD_SENSOR_SwitchCallBack(uint32_t mask)
{
	int i;
	bool state;
	int done = 0;
	uint8_t index;
	uint32_t temp_mask = 0;

	// Check IRQ mask for all switches
	for (i = 0; i < CONFIG_TD_SENSOR_MAX_SWITCH; i++) {
		if (done >= Module.switch_count) {
			break;
		}
		if (Module.switches[i].monitor) {
			done++;
			temp_mask = 1 << Module.switches[i].bit;

			//TODO: debounce
			if (temp_mask & mask) {
				state = GPIO_PinInGet(Module.switches[i].port, Module.switches[i].bit);
				if (!Module.switches[i].on_irq) {
					index = TD_SENSOR_SwitchStateList_top + 1;
					if (index >= CONFIG_TD_SENSOR_MAX_SWITCH_EVENT) {
						index = 0;
					}

					// If too many pending, discards current event
					if (TD_SENSOR_SwitchStateList_bottom != index) {

						// Append to circular buffer
						TD_SENSOR_SwitchStateList[TD_SENSOR_SwitchStateList_top].param = (state << 7) | i;
						TD_SENSOR_SwitchStateList_top = index;
					}
				} else {
					(*Module.switches[i].user_callback)(Module.switches[i].port, Module.switches[i].bit, state);
				}
			}
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Process non-IRQ switch events.
 ******************************************************************************/
static void TD_SENSOR_SwitchEventProcess(void)
{
	uint32_t mask;
	TD_SENSOR_SwitchState_t tmp;

	while (TD_SENSOR_SwitchStateList_bottom != TD_SENSOR_SwitchStateList_top){
		mask=__get_PRIMASK();
		__set_PRIMASK(1);
		tmp.param = TD_SENSOR_SwitchStateList[TD_SENSOR_SwitchStateList_bottom].param;
		TD_SENSOR_SwitchStateList_bottom++;
		if (TD_SENSOR_SwitchStateList_bottom >= CONFIG_TD_SENSOR_MAX_SWITCH_EVENT){
			TD_SENSOR_SwitchStateList_bottom = 0;
		}
		__set_PRIMASK(mask);

		// If event is 0xFF it was canceled
		if (tmp.param != 0xFF){
			TD_SENSOR_EventProcess(SENSOR_SEND_SWITCH, tmp.param);
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Setup IRQ handlers for a switch monitoring.
 *
 * @param[in] port
 *	 Port on which the switch is connected.
 *
 * @param[in] bit
 *	 Bit on which the switch is connected.
 *
 * @param[in] falling
 *	 If true an IRQ will be triggered on falling edge.
 *
 * @param[in] rising
 *	 If true an IRQ will be triggered on rising edge.
 *
 * @param[in] pull
 *	 Enable pull-up or pull-down on the specified port, bit.
 *
 * @param[in] pull_state
 *	 If pull is set to true, pull_state determines if the pin is pulled high
 *	 (true) or pull-down (false).
 *
 * @param[in] switch_callback
 *   Pointer to the callback function that will be called upon GPIO IRQ with the
 *   following arguments:
 *     - port: the Switch GPIO port that generated the IRQ
 *     - bit: the Switch GPIO bit that generated the IRQ
 *     - state: the Switch GPIO state that generated the IRQ
 ******************************************************************************/
static void TD_SENSOR_SetSwitchMonitoring(GPIO_Port_TypeDef port, unsigned int bit, bool falling, bool rising, bool pull, bool pull_state, bool (*switch_callback)(GPIO_Port_TypeDef port, unsigned int bit, bool state))
{
	if (((1 << bit) & 0xAAAAAAAA) != 0) {

		// Odd
		TD_GPIO_SetCallback(TD_GPIO_USER_ODD,
				TD_SENSOR_SwitchCallBack,
				Module.switch_mask & TD_GPIO_ODD_MASK);
	} else {

		// Even
		TD_GPIO_SetCallback(TD_GPIO_USER_EVEN,
				TD_SENSOR_SwitchCallBack,
				Module.switch_mask & TD_GPIO_EVEN_MASK);
	}
	if (pull) {
		GPIO_PinModeSet(port, bit, gpioModeInputPull, pull_state);
	} else {
		GPIO_PinModeSet(port, bit, gpioModeInput, 0);
	}
	GPIO_IntConfig(port, bit, rising, falling, true);
}

/***************************************************************************//**
 * @brief
 *	Configure Switch Monitoring.
 *
 * @param[in] enable
 *	 Enable (true) or disable (false) switch monitoring.
 *
 * @param[in] port
 *	 Port on which the switch is connected.
 *
 * @param[in] bit
 *	 Bit on which the switch is connected.
 *
 * @param[in] falling
 *	 If true an IRQ will be triggered on falling edge.
 *
 * @param[in] rising
 *	 If true an IRQ will be triggered on rising edge.
 *
 * @param[in] pull
 *	 Enable pull-up or pull-down on the specified port, bit.
 *
 * @param[in] pull_state
 *	 If pull is set to true, pull_state determines if the pin is pulled high
 *	 (true) or pull-down (false), otherwise it has no effect.
 *
 * @param[in] switch_callback
 *   Pointer to the callback function that will be called upon GPIO IRQ with the
 *   following arguments:
 *     - port: the Switch GPIO port that generated the IRQ
 *     - bit: the Switch GPIO bit that generated the IRQ
 *     - state: the Switch GPIO state that generated the IRQ
 *
 * @param[in] irq
 *   Set to true to enable IRQ switch monitoring, set to false to enable polled
 *   switch monitoring.
 *
 * @details
 *	 A Switch ON Sensor message will be sent on a falling edge and a Switch OFF
 *	 message will be sent on a rising edge. It is therefore highly recommended
 *	 to connect one side of your switch to the ground and activate pull-up on
 *	 the corresponding GPIO.
 ******************************************************************************/
static bool TD_SENSOR_PrivateMonitorSwitch(bool enable, GPIO_Port_TypeDef port, unsigned int bit, bool falling, bool rising, bool pull, bool pull_state, bool (*switch_callback)(GPIO_Port_TypeDef port, unsigned int bit, bool state), bool irq)
{
	int i;
	bool monitored = false;
	int index = -1;
	int alt_index = -1;
	uint32_t mask_irq;
	uint8_t j;

	// If enable, append the GPIO to the list if not already there
	if (enable) {
		if (Module.switch_count >= CONFIG_TD_SENSOR_MAX_SWITCH) {
			return false;
		} else if (Module.switch_count == 0) {

			// Enable GPIO IRQs
			NVIC_EnableIRQ(GPIO_EVEN_IRQn);
			NVIC_EnableIRQ(GPIO_ODD_IRQn);
		}
		monitored = false;
		index = -1;
		alt_index = -1;

		// Make sure the bit/port is not already in the list
		for (i = 0; i < CONFIG_TD_SENSOR_MAX_SWITCH; i++) {
			if (Module.switches[i].bit == bit && Module.switches[i].port == port) {

				// The best thing to dot is to reuse the previous index
				index = i;
				monitored = Module.switches[i].monitor;
			}

			// If it does not exist, use the first available slot
			if (alt_index == -1 && !Module.switches[i].monitor) {
				alt_index = i;
			}
		}
		if (index == -1) {
			if (alt_index == -1) {

				//TODO: generate a trap
			} else {
				index = alt_index;
			}
		}

		// If already running, stop IRQs for this pin
		if (monitored) {
			GPIO_IntConfig(port, bit, false, false, false);
		}
		Module.switches[index].monitor = true;
		Module.switches[index].bit = bit;
		Module.switches[index].port = port;
		Module.switches[index].falling = falling;
		Module.switches[index].rising = rising;
		Module.switches[index].pull = pull;
		Module.switches[index].pull_state = pull_state;
		Module.switches[index].state = GPIO_PinInGet(port, bit);
		Module.switches[index].user_callback = switch_callback;
		Module.switches[index].on_irq = irq;
		Module.switch_mask |= 1 << bit;
		if (!monitored) {
			Module.switch_count++;
		}
		TD_SENSOR_SetSwitchMonitoring(port,
			bit,
			falling,
			rising,
			pull,
			pull_state,
			switch_callback);
		return true;
	} else {

		// Remove the GPIO from the list
		for (i = 0; i < CONFIG_TD_SENSOR_MAX_SWITCH; i++) {
			if (Module.switches[i].bit == bit
					&& Module.switches[i].port == port) {
				uint32_t mask = 1 << bit;
				mask = ~mask;
				Module.switch_mask &= mask;

				// Disable IRQs
				GPIO_IntConfig(port, bit, false, false, false);

				// Then remove the mask
				if ((bit & 0xAAAAAAAA) != 0) {

					// Odd IRQ
					TD_GPIO_SetCallback(TD_GPIO_USER_ODD,
						TD_SENSOR_SwitchCallBack,
						Module.switch_mask & TD_GPIO_ODD_MASK);
				} else {

					// Even IRQ
					TD_GPIO_SetCallback(TD_GPIO_USER_EVEN,
						TD_SENSOR_SwitchCallBack,
						Module.switch_mask & TD_GPIO_EVEN_MASK);
				}

				// Now IRQ is removed. Remove also dereferenced call
				mask_irq = __get_PRIMASK();
				__set_PRIMASK(1);
				j = TD_SENSOR_SwitchStateList_bottom;
				while (j != TD_SENSOR_SwitchStateList_top) {

					// There is an event we are removing in queue
					if ((TD_SENSOR_SwitchStateList[j].param & 0x7F) == i) {

						// Cancel it
						TD_SENSOR_SwitchStateList[j].param = 0xFF;
					}
					j++;
					if (j >= CONFIG_TD_SENSOR_MAX_SWITCH_EVENT) {
						j = 0;
					}
				}
				__set_PRIMASK(mask_irq);
				Module.switches[i].monitor = false;
				Module.switches[i].bit = 0xFF;
				Module.switches[i].port = (GPIO_Port_TypeDef) 0xFF;
				Module.switches[i].user_callback = 0;
				Module.switch_count--;
				return true;
			}
		}
	}
	return false;
}

/** @} */

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_GLOBAL_FUNCTIONS Global Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *  Battery monitoring callback function, called after a SIGFOX transmission.
 ******************************************************************************/
void TD_SENSOR_BatteryCallBack(void)
{
	uint16_t battery_level_mv;

	if (Module.battery.monitor) {
		battery_level_mv = (uint16_t) TD_SIGFOX_PowerVoltageExtended();

		// If there was a previous SIGFOX transmission
		if (battery_level_mv != 0) {
			Module.battery.level = battery_level_mv;
			if (Module.battery.state) {
				if (battery_level_mv <= Module.battery.level_low) {
					Module.battery.state = false;
					SaveCurrentStateToFlash();
					TD_SENSOR_EventProcess(SENSOR_SEND_BATTERY, 0);
				}
			} else {
				if (battery_level_mv >= Module.battery.level_ok) {
					Module.battery.state = true;
					SaveCurrentStateToFlash();
					TD_SENSOR_EventProcess(SENSOR_SEND_BATTERY, 0);
				}
			}
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Dump the Sensor configuration.
 *******************************************************************************/
void TD_SENSOR_Dump(void)
{
	int i;

	tfp_printf("Boot %d call=%08x\r\n",
		Module.boot.monitor,
		Module.boot.user_callback);
	tfp_printf("Bat %d, low=%d ok=%d call=%08x\r\n",
		Module.battery.monitor,
		Module.battery.level_low,
		Module.battery.level_ok,
		Module.battery.user_callback);
	tfp_printf("Temp %d, int=%d low=%d high=%d call=%08x\r\n",
		Module.temperature.monitor,
		Module.temperature.interval,
		Module.temperature.level_low,
		Module.temperature.level_high,
		Module.temperature.user_callback);
	tfp_printf("Con %d, int=%d\r\n",
		Module.connection.monitor,
		Module.connection.interval);
	tfp_printf("KA %d, int=%d\r\n",
		Module.keepalive.monitor,
		Module.keepalive.interval);
	for (i = 0; i < CONFIG_TD_SENSOR_MAX_SWITCH; i++) {
		tfp_printf("Switch %d Monitor: %d Port %d Bit %d\r\n",
			i,
			Module.switches[i].monitor,
			Module.switches[i].port,
			Module.switches[i].bit);
	}
}

/***************************************************************************//**
 * @brief
 *   Sensor monitoring initialization.
 ******************************************************************************/
void TD_SENSOR_MonitorInit(void)
{
	TD_SENSOR_ApplyMonitoringConfiguration();
}

/** @} */

/** @addtogroup TD_SENSOR_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *	Configure a polled Switch Monitoring on a module pin.
 *
 * @param[in] enable
 *	Enable (true) or disable (false) switch monitoring.
 *
 * @param[in] port
 *	Port on which the switch is connected.
 *
 * @param[in] bit
 *	Bit on which the switch is connected.
 *
 * @param[in] falling
 *	If true an IRQ will be triggered on falling edge.
 *
 * @param[in] rising
 *	If true an IRQ will be triggered on rising edge.
 *
 * @param[in] pull
 *	Enable pull-up or pull-down on the specified port, bit.
 *
 * @param[in] pull_state
 *	If pull is set to true, pull_state determines if the pin is pulled high
 *	(true) or pull-down (false), otherwise it has no effect.
 *
 * @param[in] switch_callback
 *  Pointer to the callback function that will be called upon GPIO IRQ with the
 *  following arguments:
 *    - port: the Switch GPIO port that generated the IRQ
 *    - bit: the Switch GPIO bit that generated the IRQ
 *    - state: the Switch GPIO state that generated the IRQ
 *
 * @details
 *	A Switch ON Sensor message will be sent on a falling edge and a Switch OFF
 *	message will be sent on a rising edge. It is therefore highly recommended to
 *	connect one side of your switch to the ground and activate pull-up on the
 *	corresponding GPIO.
 *  The callback will be called within TD_SENSOR_Process().
 ******************************************************************************/
bool TD_SENSOR_MonitorSwitch(bool enable, GPIO_Port_TypeDef port, unsigned int bit, bool falling, bool rising, bool pull, bool pull_state, bool (*switch_callback)(GPIO_Port_TypeDef port, unsigned int bit, bool state))
{
	return TD_SENSOR_PrivateMonitorSwitch(enable, port, bit, falling, rising, pull, pull_state, switch_callback, false);
}

/***************************************************************************//**
 * @brief
 *	Configure an IRQ-based Switch Monitoring on a module pin.
 *
 * @param[in] enable
 *	Enable (true) or disable (false) switch monitoring.
 *
 * @param[in] port
 *	Port on which the switch is connected.
 *
 * @param[in] bit
 *	Bit on which the switch is connected.
 *
 * @param[in] falling
 *	If true an IRQ will be triggered on falling edge.
 *
 * @param[in] rising
 *	If true an IRQ will be triggered on rising edge.
 *
 * @param[in] pull
 *	Enable pull-up or pull-down on the specified port, bit.
 *
 * @param[in] pull_state
 *	If pull is set to true, pull_state determines if the pin is pulled high
 *	(true) or pull-down (false), otherwise it has no effect.
 *
 * @param[in] switch_callback
 *  Pointer to the callback function that will be called upon GPIO IRQ with the
 *  following arguments:
 *    - port: the Switch GPIO port that generated the IRQ
 *    - bit: the Switch GPIO bit that generated the IRQ
 *    - state: the Switch GPIO state that generated the IRQ
 *
 * @details
 *	A Switch ON Sensor message will be sent on a falling edge and a Switch OFF
 *	message will be sent on a rising edge. It is therefore highly recommended to
 *	connect one side of your switch to the ground and activate pull-up on the
 *	corresponding GPIO.
 *  The callback will be called within TD_SENSOR_Process().
 ******************************************************************************/
bool TD_SENSOR_MonitorSwitchIrq(bool enable, GPIO_Port_TypeDef port, unsigned int bit, bool falling, bool rising, bool pull, bool pull_state, bool (*switch_callback)(GPIO_Port_TypeDef port, unsigned int bit, bool state))
{
	return TD_SENSOR_PrivateMonitorSwitch(enable, port, bit, falling, rising, pull, pull_state, switch_callback, true);
}

/***************************************************************************//**
 * @brief
 *	Configure Temperature Monitoring on the module.
 *
 * @param[in] enable
 *	Enable (true) or disable (false) temperature monitoring.
 *
 * @param[in] interval
 *	Interval in seconds at which the temperature should be checked.
 *
 * @param[in] level_low
 *	Temperature level in 1/10 Celsius degrees at which a "temperature low" event
 *	is sent to Sensor.
 *
 * @param[in] level_high
 *	Temperature level in 1/10 Celsius degrees at which a "temperature high" event
 *	is sent to Sensor.
 *
 * @param[in] callback
 *  Pointer to a function that will be called on any temperature event with the
 *  following parameters:
 *    - state: the temperature state that trigggered the event
 *    - level: the current temperature level in 1/10 Celisus degrees.
 *
 * @details
 *	A "temperature OK" event is sent once to Sensor if the current temperature is
 *	back within the allowed range after a temperature high/low event has been
 *	previously sent.
 ******************************************************************************/
void TD_SENSOR_MonitorTemperature(bool enable, uint32_t interval, int16_t level_low, int16_t level_high, bool (*callback)(TD_SENSOR_TemperatureState_t state, int16_t level))
{
	if (enable) {
		if (!Module.temperature.monitor) {

			// Start timer
			Module.temperature.timer = TD_SCHEDULER_Append(interval,
				0,
				0,
				0xFF,
				TD_SENSOR_TemperatureCallBack,
				0);
			if (Module.temperature.timer != 0xFF) {
				Module.temperature.monitor = true;
				Module.temperature.interval = interval;
				Module.temperature.level_low = level_low;
				Module.temperature.level_high = level_high;
				Module.temperature.user_callback = callback;

				// Check the temperature right now
				TD_SENSOR_TemperatureCallBack(0, 0);
			}
		} else {

			// Modify the settings
			TD_SCHEDULER_SetInterval(Module.temperature.timer, interval, 0, 0);
			Module.temperature.interval = interval;
			Module.temperature.level_low = level_low;
			Module.temperature.level_high = level_high;
			Module.temperature.user_callback = callback;

			// Check the temperature right now
			TD_SENSOR_TemperatureCallBack(0, 0);
		}
	} else if (!enable && Module.temperature.monitor) {
		TD_SCHEDULER_Remove(Module.temperature.timer);
		Module.temperature.timer = 0xFF;
		Module.temperature.monitor = false;
	}
}

/***************************************************************************//**
 * @brief
 *   Configure RSSI Monitoring on gateway side. Keep-alive monitoring
 *   must be enabled to enable/disable RSSI.
 *
 *  @param[in] enable
 *   Enable (true) or disable (false) RSSI monitoring.
 *
 *  @param[in] level_low
 *   RSSI level at which an "RSSI low" event is sent to Sensor.
 *
 *  @param[in] level_ok
 *   RSSI level at which an "RSSI OK" event is sent to Sensor (this
 *   only happens once after an "RSSI low/high" event occurred).
 *
 *  @return
 *  returns true if the module is a device, keep-alive is enabled and if the
 *  gateway acknowledged the keep-alive frame, false otherwise.
 ******************************************************************************/
bool TD_SENSOR_MonitorRSSI(bool enable, int8_t level_low, int8_t level_ok)
{
	// Only a Device can request RSSI monitoring
	if (Module.type == SENSOR_DEVICE) {
		if (Module.connection.monitor) {
			if (enable) {

				// If there is something to update
				if (!Module.rssi.monitor ||
					Module.rssi.level_low != level_low ||
					Module.rssi.level_ok != level_ok) {
					if (TD_SENSOR_DEVICE_KeepAlive(true,
						Module.connection.interval,
						true,
						level_low,
						level_ok) == ACK_OK) {
						Module.rssi.level_low = level_low;
						Module.rssi.level_ok = level_ok;
						Module.rssi.monitor = true;
						return true;
					}
				}
			} else if (Module.rssi.monitor) {
				if (TD_SENSOR_DEVICE_KeepAlive(true,
						Module.connection.interval,
						false,
						0,
						0) == ACK_OK) {
					Module.rssi.monitor = false;
					return true;
				}
			}
		}
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *   Configure Battery Monitoring. Battery level is checked after a SIGFOX
 *   transmission while the battery is still fully loaded by a charge.
 *
 *  @param[in] enable
 *   Enable (true) or disable (false) Battery monitoring.
 *
 *  @param[in] level_low
 *   Battery level at which a "battery low" event is sent to Sensor.
 *
 *  @param[in] level_ok
 *   Battery level at which a "battery OK" event is sent to Sensor (this
 *   only happens once after a "battery low" event occurred).
 *
 *  @param[in] callback
 *   Pointer to a callback function that will be called on any Battery Event
 *   with the following arguments:
 *     - state: the battery state
 *     - level: the battery level in mV
 ******************************************************************************/
void TD_SENSOR_MonitorBattery(bool enable, uint16_t level_low, uint16_t level_ok, bool (*callback)(bool state, uint16_t level))
{
	 // TODO: Measure battery level during LAN transmissions as well
	Module.battery.monitor = enable;
	if (enable) {
		Module.battery.level_low = level_low;
		Module.battery.level_ok = level_ok;
		Module.battery.user_callback = callback;
	} else {
		Module.battery.user_callback = 0;
	}
}

/***************************************************************************//**
 * @brief
 *   Configure Boot Monitoring. Monitoring will only take effect on next reboot.
 *
 *  @param[in] enable
 *   Enable (true) or disable (false) boot monitoring.
 *
 *  @param[in] callback
 *   Pointer to a callback function that will be called on boot events.
 ******************************************************************************/
void TD_SENSOR_MonitorBoot(bool enable, bool (*callback)(void))
{
	if (enable != Module.boot.monitor || Module.boot.user_callback != callback) {
		Module.boot.monitor = enable;
		Module.boot.user_callback = callback;
		SaveCurrentStateToFlash();
	}
}

/***************************************************************************//**
 * @brief
 *   Save a custom boot cause value into Flash memory. As this value will be sent
 *   in a boot event, custom boot cause can bee used alongside boot monitoring
 *   to provide a mean to know exactly why a reboot occurred.
 *
 *  @param[in] cause
 *   The custom boot cause to be saved into Flash memory.
 ******************************************************************************/
void TD_SENSOR_SetCustomBootCause(uint8_t cause)
{
	CustomBootCause = cause;
	SaveCurrentStateToFlash();
}

/***************************************************************************//**
 * @brief
 *   Configure Connection Monitoring. This requires the device and the gateway to
 *   be already paired.
 *
 *  @param[in] enable
 *   Enable (true) or disable (false) connection monitoring.
 *
 *  @param[in] interval
 *   Interval in seconds at which connection is being checked.
 *
 * @return
 *   Returns true if the module is a device or a gateway and if a timer could be
 *   started, false otherwise.
 *
 * @details:
 *    - If the module is a device: at every interval, a LAN frame will be sent to
 *      the gateway. Each time the gateway receives it, it will arm a timer which
 *      will expect the next keep-alive frame to be received from this device. If
 *      the next keep-alive is not received on time, then the gateway will
 *      transmit a connection lost event. If the gateway further receives a
 *      keep-alive frame, it will send a connection OK event.
 *
 *    - If the module is a gateway: the gateway will broadcast a keep-alive
 *      request to all devices at every given interval. Keep-alive handling must
 *      be implemented on the device side by using the
 *      TD_SENSOR_DEVICE_StartBroadcastReception and
 *      TD_SENSOR_DEVICE_SetKeepAliveCallback API function calls.
 ******************************************************************************/
bool TD_SENSOR_MonitorConnection(bool enable, uint32_t interval)
{
	if (Module.type == SENSOR_DEVICE || Module.type == SENSOR_GATEWAY) {
		if (enable) {

			// If not already enabled
			if (!Module.connection.monitor) {

				// Enable keep-alive monitoring even if the gateway does not reply!
				Module.connection.timer = TD_SCHEDULER_Append(interval,
					0,
					0,
					0xFF,
					TD_SENSOR_ConnectionCallBack,
					0);
				if (Module.connection.timer != 0xFF) {
					if (Module.type == SENSOR_DEVICE) {
						TD_SENSOR_DEVICE_KeepAlive(true,
							interval,
							Module.rssi.monitor,
							Module.rssi.level_low,
							Module.rssi.level_ok);
					} else {
						TD_SENSOR_GATEWAY_SendKeepAliveBroadcast(true,
							interval,
							0,
							0);
					}
					Module.connection.monitor = enable;
					Module.connection.interval = interval;
					return true;
				} else {
					return false;
				}
			} else {

				// Just update the interval
				TD_SCHEDULER_SetInterval(Module.connection.timer,
					interval,
					0,
					0);
				Module.connection.interval = interval;
				return true;
			}
		} else if (!enable && Module.connection.monitor) {
			if (Module.type == SENSOR_DEVICE) {
				if (TD_SENSOR_DEVICE_KeepAlive(false, 0, false, 0, 0) == ACK_OK) {
					TD_SCHEDULER_Remove(Module.connection.timer);
					Module.connection.monitor = false;
					Module.rssi.monitor = false;
					Module.connection.interval = 0;
					Module.connection.timer = 0xFF;
					return true;
				} else {
					return false;
				}
			} else if (Module.type == SENSOR_GATEWAY) {
				TD_SCHEDULER_Remove(Module.connection.timer);
				Module.connection.monitor = false;
				Module.rssi.monitor = false;
				Module.connection.interval = 0;
				Module.connection.timer = 0xFF;
			} else {
				return false;
			}
		}
		return true;
	}
	return false;
}
/***************************************************************************//**
 * @brief
 *   Sensor Keepalive Monitoring. A keep-alive frame will be sent to SIGFOX
 *   at fixed intervals.
 *
 *  @param[in] enable
 *   Enable (true) or disable (false) keep-alive monitoring.
 *
 *  @param[in] interval_hour
 *   Interval in hours at which keep-alive frame are being sent.
 *
 * @return
 *   Returns true if a keep-alive timer has been successfully programmed or
 *   removed, false otherwise.
 ******************************************************************************/
bool TD_SENSOR_MonitorKeepAlive(bool enable, uint8_t interval_hour)
{
	if (enable) {

		// Enable timer if not already done
		if (!Module.keepalive.monitor) {

			// Send a keep-alive frame once to let Sensor know about the interval
			TD_SENSOR_KeepAliveCallback(0, 0);
			Module.keepalive.timer = TD_SCHEDULER_Append(interval_hour * 3600,
				0,
				0,
				0xFF,
				TD_SENSOR_KeepAliveCallback,
				0);
			if (Module.keepalive.timer != 0xFF) {
				Module.keepalive.monitor = true;
				Module.keepalive.interval = interval_hour;
				return true;
			}
		} else {

			// Only change interval
			TD_SCHEDULER_SetInterval(Module.keepalive.timer,
									 interval_hour * 3600,
									 0,
									 0);
			return true;
		}
	} else if (!enable && Module.keepalive.monitor) {

		// Remove timer
		Module.keepalive.monitor = false;
		Module.keepalive.interval = 0;
		TD_SENSOR_KeepAliveCallback(0, 0);
		TD_SCHEDULER_Remove(Module.keepalive.timer);
		return true;
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *   Sensor reset. All monitoring, LAN configuration and managed devices are
 *   deleted. Only the module type is unchanged.
 ******************************************************************************/
void TD_SENSOR_Reset(void)
{
	int i;

	// Remove all active timers
	if (Module.keepalive.timer != 0xFF) {
		TD_SCHEDULER_Remove(Module.keepalive.timer);
		Module.keepalive.timer = 0xFF;
	}
	if (Module.connection.timer != 0xFF) {
		TD_SCHEDULER_Remove(Module.connection.timer);
		Module.connection.timer = 0xFF;
	}
	if (Module.temperature.timer != 0xFF) {
		TD_SCHEDULER_Remove(Module.temperature.timer);
		Module.temperature.timer = 0xFF;
	}

	// Stop switch monitoring
	for (i = 0; i < CONFIG_TD_SENSOR_MAX_SWITCH; i++) {
		if (Module.switches[i].monitor) {
			//disable irq
			GPIO_IntConfig(Module.switches[i].port,
						   Module.switches[i].bit,
						   false,
						   false,
						   false);
		}
	}
	TD_GPIO_SetCallback(TD_GPIO_USER_ODD, TD_SENSOR_SwitchCallBack, 0);
	TD_GPIO_SetCallback(TD_GPIO_USER_EVEN, TD_SENSOR_SwitchCallBack, 0);

	// Reset LAN
	if (Module.type != SENSOR_TRANSMITTER) {
		if (Module.type == SENSOR_GATEWAY) {
			TD_SENSOR_GATEWAY_DeleteAllDevices();
		} else if (Module.type == SENSOR_DEVICE) {
			TD_SENSOR_DEVICE_Reset();
		}
	}

	// Reinitialize all local variables except the module type
	TD_SENSOR_InternalInit();
}

/***************************************************************************//**
 * @brief
 *   Set the device class.
 *
 *  @param[in] class
 *   The new device class to use.
 ******************************************************************************/
void TD_SENSOR_SetDeviceClass(uint16_t class)
{
	Module.class = class;
	ConfigClass = true;
}

/***************************************************************************//**
 * @brief
 *   Get the module type.
 *
 * @return
 *  Returns the module type (device, gateway or transmitter).
 ******************************************************************************/
TD_SENSOR_ModuleType_t TD_SENSOR_GetModuleType(void)
{
	return Module.type;
}

/***************************************************************************//**
 * @brief
 *   Set the Module type but does NOT apply it. It should be called before
 *   initialization to be taken into account.
 ******************************************************************************/
void TD_SENSOR_SetModuleType(TD_SENSOR_ModuleType_t type)
{
	Module.type = type;
}

/***************************************************************************//**
 * @brief
 *  Get the module configuration.
 *
 * @return
 *  Returns a poitner to the module configuration.
 ******************************************************************************/
TD_SENSOR_Configuration_t *TD_SENSOR_GetModuleConfiguration(void)
{
	return &Module;
}

/***************************************************************************//**
 * @brief
 *  Set the Mhodule configuration but does NOT apply it. It should be called
 *  before initialization to be taken into account.
 ******************************************************************************/
void TD_SENSOR_SetModuleConfiguration(TD_SENSOR_Configuration_t *config)
{
	if (config != 0) {
		ConfigInit = true;
		memcpy(&Module, config, sizeof(TD_SENSOR_Configuration_t));
		Module.switches = TD_SENSOR_SwitchConfig;
		memset(TD_SENSOR_SwitchConfig,
			0,
			CONFIG_TD_SENSOR_MAX_SWITCH * sizeof (TD_SENSOR_SwitchConfiguration_t));
		Module.switch_count = 0;
		Module.switch_mask = 0;
	}
}

/***************************************************************************//**
 * @brief
 *  Get the module SIGFOX ID.
 *
 * @return
 *  Returns the module's SIGFOX ID.
 ******************************************************************************/
uint32_t TD_SENSOR_GetSigfoxID(void)
{
	return SigfoxID;
}

/***************************************************************************//**
 * @brief
 * 	Get the boot cause according to the RMU (Reset Management Unit).
 *
 * @return
 *  Returns a bit field containing the (possibly multiple) boot reasons.
 *  See "ef32g_rmu.h" for details.
 *    - POR (Power On Reset): 1
 *    - Brown-out unregulated: 2
 *    - Brown-out regulated: 4
 *    - External reset: 8
 *    - Watchdog timeout: 16
 *    - Lock-up: 32
 *    - Software reset: 64
 ******************************************************************************/
uint32_t TD_SENSOR_GetBootCause(void)
{
	return BootCause;
}

/***************************************************************************//**
 * @brief
 * 	 Get the custom boot cause as saved in Flash memory using the
 * 	 TD_SENSOR_SetCustomBootCause() function.
 *
 * @return
 *   Returns the custom boot cause.
 ******************************************************************************/
uint8_t TD_SENSOR_GetCustomBootCause(void)
{
	return CustomBootCause;
}

/***************************************************************************//**
 * @brief
 *   Check if the battery is dead or not.
 *
 * @details
 *   The battery is considered dead if the the module rebooted twice
 *   consecutively with a brown-out reboot cause.
 *
 *  @return
 * 	 Returns true is the battery is considered dead, false otherwise.
 ******************************************************************************/
bool TD_SENSOR_IsBatteryDead(void)
{
	ReadBootCause();
	if (!TD_FLASH_DeclareVariable((uint8_t *) &BrownOutReboot, sizeof(BrownOutReboot), 0)) {
		BrownOutReboot = 0;
	}
	switch (BootCause) {
	case RMU_RSTCAUSE_BODREGRST:
	case RMU_RSTCAUSE_BODUNREGRST:
		if (BrownOutReboot <= 1) {

			// First brown-out reboot: give it a go but battery may be dead
			// Second consecutive brown-out reboot: battery is declared dead
			if (++BrownOutReboot == 2) {
			}
			TD_FLASH_WriteVariables();
		}
		break;

	default:
		if (BrownOutReboot >= 1) {

			// Rebooting from another cause resets the brown-out counter
			BrownOutReboot = 0;
			TD_FLASH_WriteVariables();
		}
		break;
	}
	return BrownOutReboot >= 2 ? true : false;
}

/***************************************************************************//**
 * @brief
 *   Initialize the module as either a gateway, a device or a transmitter with
 *   a given configuration.
 *
 * @param[in] type
 *	The module type:
 *		- SENSOR_DEVICE: the module will mainly communicate with a gateway which
 *		will forward all messages to SIGFOX.
 *		Note however that it may also send message directly to SIGFOX without
 *		a gateway forward
 *		- SENSOR_GATEWAY: the module is managing up to 15 devices. It can
 *		communicate with the devices, forward frames from them and send frames
 *		to SIGFOX
 *		- SENSOR_TRANSMITTER: the module only send frames to SIGFOX and does not
 *		use any LAN communication
 *
 * @param[in] lan_frequency
 *	The frequency in Hz to use for LAN communications. The valid range is
 *	868000000..869700000 Hz, default is 869312500 Hz.
 *
 * @param[in] lan_power_level
 *	The power level in dBm to use for LAN communications. The valid range is
 *	-35..14 dBm, default is 10 dBm.
 *
 * @return
 *   Always returns true for a transmitter. Returns false for a gateway or a
 *   device if the LAN could not be initialized properly.
 ******************************************************************************/
bool TD_SENSOR_Init(TD_SENSOR_ModuleType_t type, uint32_t lan_frequency, int16_t lan_power_level)
{
	TD_DEVICE device;
	TD_SENSOR_DeviceClass_t temp_class;
	bool ret = true;

	ReadBootCause();

	// Read the SIGFOX ID
	if (TD_FLASH_DeviceRead(&device)) {
		SigfoxID = device.Serial;
	}
	if (!ConfigInit) {

		// If a class is provided, save it
		if (ConfigClass) {
			temp_class = Module.class;
		}

		// If the configuration is null, initialize the current configuration (setting class to 0)
		TD_SENSOR_InternalInit();

		// Apply back the class
		if(ConfigClass) {
			Module.class = temp_class;
		}
	}

	// Load current state which must be persistent (boot monitoring, battery state...)
	// from Flash memory
	LoadCurrentStateFromFlash();

	// Set the module type
	Module.type = type;

	// Provide default values for LAN parameters
	if (lan_frequency == 0) {
		lan_frequency = 869312500;
		lan_power_level = 10;
	}

	// LAN and/or Transmitter initialization depending on module type
	if (Module.type == SENSOR_DEVICE) {
		if (TD_SENSOR_LAN_Init(false, lan_frequency, lan_power_level) == false) {
			ret = false;
		}
	} else if (Module.type == SENSOR_GATEWAY) {
		if (TD_SENSOR_LAN_Init(true, lan_frequency, lan_power_level) == false) {
			ret = false;
		}
	}
	if (Module.type == SENSOR_GATEWAY || Module.type == SENSOR_TRANSMITTER) {
		TD_SENSOR_TRANSMITTER_Init_();
	}
	if ((void *) TD_SENSOR_MonitorInit_ != (void *) TD_TrapHere) {
		TD_SENSOR_MonitorInit_();
	}
	return ret;
}

/***************************************************************************//**
 * @brief
 *   Sensor state machine processing, to be called from main user loop.
 *
 * @details
 *   This function processes pending events and pending transmitter SIGFOX
 *   frames.
 ******************************************************************************/
void TD_SENSOR_Process(void)
{
	// Process device events in queue
	if (Module.type == SENSOR_DEVICE) {
		TD_SENSOR_DEVICE_Process();
	}

	// Process frames in queue
	TD_SENSOR_TRANSMITTER_Process();

	// Process switch event in queue
	TD_SENSOR_SwitchEventProcess();
}

/** @} */

/** @} (end addtogroup TD_SENSOR) */
