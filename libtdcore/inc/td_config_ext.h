/** @cond TD_CONFIG */
/***************************************************************************//**
 * @file
 * @brief Configuration file for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.0.0
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

#ifndef __TD_CONFIG_EXT_H
#define __TD_CONFIG_EXT_H

#include <stdint.h>
#include <stdbool.h>

#include <em_gpio.h>

/* To define custom board, td_config.h default value must be overloaded
 * Here we defined useful helpers to do this overloading
 * See td_config.h documentation for more in depth informations
 */

#define PI_DISABLED		0
#define PI_INPUT		1
#define PI_INPUT_PULL	2
#define PI_OUTPUT		4

#define SIZEOF(x) ((char*)(&(x) + 1) - (char*)&(x))

#ifndef NULL
#define NULL ((void*)0)
#endif

#define IS_EMPTY(x)		1-x-1==2

#define PIP(p,b,m,v) 	(((p) << 12) | ((b) << 8) | ((v) << 4) | (m))
#define PIS(p,s) 		(((p) << 12) | ((s) <<8 ) | ((2) << 4))
#define PINULL 			(((0xF) << 4))

/* Theses are global constants value that can be used in projects */

/*
	 *    #define                                 | Level        | Type      |      Usage     | Description
	 *    :---------------------------------------|:------------:|:---------:|:--------------:|:--------------------------------------------------------------------
	 *    MANUFACTURER                            | Applicative  | string    |  AT commands   | Manufacturer name
	 *    HARDWARE_VERSION                        | Applicative  | string    |  AT commands   | Hardware version
	 *    SOFTWARE_VERSION                        | Applicative  | string    |  AT commands   | Software version
	 *    RELEASE_DATE                            | Applicative  | string    |  AT commands   | Release date
	 *    SERIAL_NUMBER                           | Applicative  | string    |  AT commands   | Serial number
	 *    PRODUCT                                 | Applicative  | string    |  AT commands   | Product name
*/
//#define PRE_SDK4_COMPATIBILITY
#ifdef PRE_SDK4_COMPATIBILITY
#define ProductType 							CONFIG_PRODUCT_TYPE
#define ProductLedPolarity						CONFIG_PRODUCT_LED_POLARITY
#define ProductLedPort							CONFIG_PRODUCT_LED_PORT
#define ProductLedBit							CONFIG_PRODUCT_LED_BIT
#define ProductLedBlink							CONFIG_PRODUCT_LED_BLINK
#define ProductBootloaderChannel				CONFIG_PRODUCT_BOOTLOADER_CHANNEL
#define ProductBootloaderSkip					CONFIG_PRODUCT_BOOTLOADER_SKIP
#define ProductInitData							CONFIG_PRODUCT_INIT_DATA
#define ProductInitDataSize						CONFIG_PRODUCT_INIT_DATA_SIZE
#define TD_SPI_MaxConf							CONFIG_MAX_SPI_ID
#define TD_FLASH_MaxDataPointer					CONFIG_TD_FLASH_MAX_DATA_POINTER
#define TD_BSP_StackSize						CONFIG_STACK_SIZE
#define TD_SENSOR_TRANSMITTER_MaxTransmission	CONFIG_TD_SENSOR_TRANSMITTER_MAX_TRANSMIT
#define TD_SENSOR_TRANSMITTER_MaxRetransmission	CONFIG_TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT
#define TD_SENSOR_GATEWAY_MaxDevice				CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE
#define TD_SENSOR_MaxSwitch						CONFIG_TD_SENSOR_MAX_SWITCH
#define TD_SENSOR_MaxSwitchEvent				CONFIG_TD_SENSOR_MAX_SWITCH_EVENT
#define TD_SCHEDULER_MaxTimer					CONFIG_TD_SCHEDULER_MAX_TIMER
#define TD_SCHEDULER_MaxQueue					CONFIG_TD_SCHEDULER_MAX_QUEUE
#define LanPeriod								CONFIG_LAN_PERIOD
#define LanThreshold							CONFIG_LAN_THRESHOLD
#define TD_BSP_GPS_CS_Port						CONFIG_GPS_CS_PORT
#define TD_BSP_GPS_CS_Bit						CONFIG_GPS_CS_BIT
#define TD_BSP_GPS_IRQ_Port						CONFIG_GPS_IRQ_PORT
#define TD_BSP_GPS_IRQ_Bit						CONFIG_GPS_IRQ_BIT
#define TD_BSP_GPS_RESET_Port					CONFIG_GPS_RESET_PORT
#define TD_BSP_GPS_RESET_Bit					CONFIG_GPS_RESET_BIT
#define TD_BSP_GPS_VBCKP_Port					CONFIG_GPS_VBCKP_PORT
#define TD_BSP_GPS_VBCKP_Bit					CONFIG_GPS_VBCKP_BIT
#define TD_BSP_GPS_VIO_Port						CONFIG_GPS_VIO_PORT
#define TD_BSP_GPS_VIO_Bit						CONFIG_GPS_VIO_BIT
#define TD_BSP_PowerCrystalPort					CONFIG_POWER_CRYSTAL_PORT
#define TD_BSP_PowerCrystalBit					CONFIG_POWER_CRYSTAL_BIT
#define TD_BSP_ShutdownPort						CONFIG_SHTD_PORT
#define TD_BSP_ShutdownBit						CONFIG_SHTD_BIT
#define TD_BSP_RadioInfoPin						CONFIG_RADIO_INFO_PIN
#define TD_BSP_ForceRadioReset					CONFIG_FORCE_RADIO_RESET
#define TD_BSP_RadioPAPower						CONFIG_RADIO_PA_POWER
#define RadioInitData							CONFIG_RADIO_INIT_DATA
#endif

#define AT_manufacturer							CONFIG_MANUFACTURER
#define AT_hardwareRevision						CONFIG_HARDWARE_VERSION
#define AT_softwareRevision						CONFIG_SOFTWARE_VERSION
#define AT_releaseDate							CONFIG_RELEASE_DATE
#define AT_serial								CONFIG_SERIAL_NUMBER

/***************************
 *  Application definition
 ****************************/
extern char const *CONFIG_MANUFACTURER;
extern char const *CONFIG_HARDWARE_VERSION;
extern char const *CONFIG_SOFTWARE_VERSION;
extern char const *CONFIG_RELEASE_DATE;
extern char *CONFIG_SERIAL_NUMBER;

/***************************
 *  Customer Board definition
 ****************************/
extern char const CONFIG_PRODUCT_TYPE;
extern char const CONFIG_PRODUCT_LED_POLARITY;
extern GPIO_Port_TypeDef const CONFIG_PRODUCT_LED_PORT;
extern char const CONFIG_PRODUCT_LED_BIT;
extern char const CONFIG_PRODUCT_LED_BLINK;
extern unsigned char const CONFIG_PRODUCT_BOOTLOADER_CHANNEL;
extern unsigned char const CONFIG_PRODUCT_BOOTLOADER_SKIP;
extern unsigned short const CONFIG_PRODUCT_INIT_DATA[];
extern unsigned char const CONFIG_PRODUCT_INIT_DATA_SIZE;

/***************************
 *  SYSTEM and LIBRARY definition
 ****************************/
extern unsigned char const CONFIG_MAX_SPI_ID;
extern uint8_t const CONFIG_TD_FLASH_MAX_DATA_POINTER;
extern uint16_t const CONFIG_STACK_SIZE;

/* TD_SENSOR limits */
extern uint8_t const CONFIG_TD_SENSOR_TRANSMITTER_MAX_TRANSMIT;
extern uint8_t const CONFIG_TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT;
extern uint8_t const CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE;
extern uint8_t const CONFIG_TD_SENSOR_MAX_SWITCH;
extern uint8_t const CONFIG_TD_SENSOR_MAX_SWITCH_EVENT;
extern uint8_t const CONFIG_TD_SCHEDULER_MAX_TIMER;
extern uint8_t const CONFIG_TD_SCHEDULER_MAX_QUEUE;
extern unsigned short const CONFIG_LAN_PERIOD;
extern unsigned char CONFIG_LAN_THRESHOLD;
/***************************
 *  TD12xxx Module definition
 ****************************/
/* GPS Chip ports definitions on EFM32 */
extern uint8_t const CONFIG_GPS_CS_PORT;
extern uint8_t const CONFIG_GPS_CS_BIT;
extern uint8_t const CONFIG_GPS_IRQ_PORT;
extern uint8_t const CONFIG_GPS_IRQ_BIT;
extern uint8_t const CONFIG_GPS_RESET_PORT;
extern uint8_t const CONFIG_GPS_RESET_BIT;
extern uint8_t const CONFIG_GPS_VBCKP_PORT;
extern uint8_t const CONFIG_GPS_VBCKP_BIT;
extern uint8_t const CONFIG_GPS_VIO_PORT;
extern uint8_t const CONFIG_GPS_VIO_BIT;

/* RF chip port definitions on EFM32 CPU side */
extern GPIO_Port_TypeDef const CONFIG_POWER_CRYSTAL_PORT;
extern uint8_t const CONFIG_POWER_CRYSTAL_BIT;
extern GPIO_Port_TypeDef const CONFIG_SHTD_PORT;
extern uint8_t const CONFIG_SHTD_BIT;

/* RF chip port definitions on RF Chip side */
	extern uint8_t const CONFIG_RADIO_INFO_PIN;

	/* RF chip ports behavior */
	extern uint8_t const CONFIG_FORCE_RADIO_RESET;
	extern uint8_t const CONFIG_RADIO_PA_POWER;
	extern unsigned char const CONFIG_RADIO_INIT_DATA[];

#ifndef __TD_TRAP_H
#include <td_trap.h>
#endif

/** SIGFOX functions redirection
 * These definitions are done here because :
 * - we know only in library if we are using SIGFOX V1 or V0
 * - SIGFOX V0 is only supported for backward compatibility and V1 must be used now
 * - SIGFOX V0 must be made as a subset of V1 and lead to a unified SIGFOX */
typedef bool (*TD_SIGFOX_Init_t)(bool init);
typedef bool (*TD_SIGFOX_Send_t)(uint8_t *message, uint8_t size, uint8_t retry);
extern TD_SIGFOX_Init_t const TD_SIGFOX_Init;
extern TD_SIGFOX_Send_t const TD_SIGFOX_Send;
bool TD_SIGFOX_Init_(bool init);
bool TD_SIGFOX_Send_(uint8_t *message, uint8_t size, uint8_t retry);

/* Dynamic dump function */
extern uint8_t const TD_TRAP_MaxSystemDump;
extern TD_Dump_Func_t const TD_TRAP_SystemDumpFunc[];

/* Trap callback functions */
extern TD_TRAP_callback_t TD_Trap_Callback;
extern TD_TRAP_callback_t const TD_Trap_Callback2;

#endif // __TD_CONFIG_EXT_H
/** @endcond */
