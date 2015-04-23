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
#ifndef __TD_CONFIG_H
#define __TD_CONFIG_H

#include <stdint.h>

/* Note : this file td_config.h is always included in USER code, so .h file are library include '<xxx.h>' */
#include <td_config_ext.h>
#include <td_trap.h>
#include <td_rtc.h>
#include <td_scheduler.h>
#include <td_spi.h>
#include <td_gpio.h>
#include <td_utils.h>
#include <td_flash.h>
#include <td_boot.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup DYNAMIC_OPTIONS Dynamic Options
	 * @brief Link-time dynamic options for binary inter-modules/libraries parameters.
	 * @{
	 * @details
	 *
	 *   # Introduction
	 *
	 *   The Telecom Design RF SDK API is organized into subsystems, providing a
	 *   consistent interface to both hardware and software components.
	 *
	 *   Related subsystems are grouped into different static libraries , each library
	 *   taking care of a given layer in the overall Telecom Design RF SDK API.
	 *
	 *   However, there is often the need for an API user to provide build-time
	 *   options to these libraries, and from within a library, to be able to access
	 *   the build-time chosen option values.
	 *
	 *   Unfortunately, it is not possible to use standard preprocessor definitions,
	 *   as some of the libraries (such as the libtdrf) or just some of the modules
	 *   within a library (such as the UBLOX module within the libtgeoloc library) are
	 *   only provided in binary object format, mainly for Intellectual Property
	 *   and/or patent issues.
	 *
	 *   But despite the fact that you do not have the sources for these objects, the
	 *   DYNAMIC_OPTION module provides a simple mechanism allowing you to:
	 *
	 *     - define user option values once at link-time
	 *     - reference these user option values from anywhere within the firmware at
	 *       run-time, including within the binary-only static libraries
	 *
	 *   # Usage
	 *
	 *   In your project main source file (where TD_USER_Setup() is located) <b>and only here</b>, you must:
	 *   - optionally overload the definitions of library parameters (see <i>Parameters</i> below)
	 *   - include <td_config.h> at the end of your include list.
	 *
	 *   # Example
	 * @code
	 *   #include "config.h"
	 *   #include <efm32.h>
	 *   #include <td_core.h>
	 *   #include <td_rtc.h>
	 *   [...]
	 *
	 *   // Define 'manufacturer' for AT command
	 *   #define MANUFACTURER "Telecom Design"
	 *
	 *   // Define 'product name' for AT command
	 *   #define PRODUCT "Sample include"
	 *
	 *   // We need 14 timer for our application
	 *   #define TD_SCHEDULER_MAX_TIMER 14
	 *
	 *   // But we will never have more than 5 timers ready at a time
	 *   #define TD_SCHEDULER_MAX_QUEUE 5
	 *
	 *   #include <td_config.h>
	 *
	 * @endcode
	 *
	 * @addtogroup EVB_MODULES Standard EVB Modules
	 * @brief Standard EVB Module definitions
	 * @{
	 * @details
	 *  # Standard EVB Modules
	 *
	 *   When you are using standard EVB TD MODULE, only one define is mandatory
	 *
	 * @code
	 *	#define MODULE_REVISION REVISION_TD1202
	 *	#define MODULE_REVISION REVISION_TD1204
	 *	#define MODULE_REVISION REVISION_TD1208
	 * @endcode
	 *
	 *  It will define all needed parameters. All of theses can be overloaded by custom defines if needed.
	 *
	 * @} (end addtogroup EVB_MODULES)
	 *
	 * @addtogroup CUSTOM_BOARD Custom Board
	 * @brief Custom Board with standard TD Chips
	 * @{
	 * @details
	 *	# Custom Board with standard TD Chips
	 *
	 *	If your are using a custom board with a standard TD12xx chip, you can used these #define
	 *	to automatically setup configuration for your chip, and only add configuration needed for your board
	 *	(PRODUCT_INIT_DATA,PRODUCT_LED_PORT, ...)
	 *
	 * @code
	 *	#define CHIP_TD1202
	 *	#define CHIP_TD1204
	 *	#define CHIP_TD1208
	 * @endcode
	 * @} (end addtogroup CUSTOM_BOARD)
	 *
	 * @addtogroup PARAMETERS Dynamic Parameters
	 * @brief Dynamic parameters that can be overloaded for libraries
	 * @{
	 * @details
	 *
	 *   # Parameters
	 *
	 *   You can define each of these parameters before including <td_config.h>
	 *
	 *   Level can be:
	 *   - Chip       : these parameters control internal Chip configuration. Don't modify them if you use standard TDxxx chip/module products
	 *   - Board      : these parameters control Board configuration. Don't modify them if you use standard TD EVB board. If you use your own custom board, you must customize them
	 *   - Applicative: these parameters are merely software related ones (AT Configuration) and can be modified to fit your needs
	 *
	 *   Anywhere in your application all of theses parameters are accessible (read only) via constant CONFIG_xxxx
	 *
	 *   If you want to force a parameter in your main source file:
	 *
	 * @code
	 *   #define TD_SCHEDULER_MAX_TIMER 12
	 *   #include <td_config.h>
	 * @endcode
	 *
	 *   On all other sources files, all parameters are available :
	 *   tfp_printf("Max timer : %d, MaxQueue : %d\r\n",CONFIG_TD_SCHEDULER_MAX_TIMER,CONFIGTD_SCHEDULER_MAX_QUEUE);
	 *
	 *    Define                                  | Level        | Type      |      Usage     | Description
	 *    :---------------------------------------|:------------:|:---------:|:--------------:|:--------------------------------------------------------------------
	 *    MANUFACTURER                            | Applicative  | string    |  AT commands   | Manufacturer name
	 *    PRODUCT                                 | Applicative  | string    |  AT commands   | Product name
	 *    HARDWARE_VERSION                        | Applicative  | string    |  AT commands   | Hardware version
	 *    SOFTWARE_VERSION                        | Applicative  | string    |  AT commands   | Software version
	 *    RELEASE_DATE                            | Applicative  | string    |  AT commands   | Release date
	 *    SERIAL_NUMBER                           | Applicative  | string    |  AT commands   | Serial number
	 *    PRODUCT_LED_BLINK		 	              | Applicative  | boolean   |  Bootloader    | Set to true, blink led during flash of each sector (default : false)
	 *    PRODUCT_BOOTLOADER_CHANNEL	 	      | Applicative  | integer   |  Bootloader    | Bootloader channel (def : 255), other not actually handled by TD_Loader
	 *    PRODUCT_BOOTLOADER_SKIP  	 	          | Applicative  | integer   |  Bootloader    | Set to true, skip bootloader
	 *    TD_SCHEDULER_MAX_TIMER                  | Applicative  | integer   |  Scheduler     | Total number of Scheduler Timers
	 *    TD_SCHEDULER_MAX_QUEUE                  | Applicative  | integer   |  Scheduler     | Total number of timer call-back (not IRQ) in pending queue
	 *    TD_SENSOR_TRANSMITTER_MAX_TRANSMIT      | Applicative  | integer   |  Sensor        | Max pending SIGFOX message transmission count
	 *    TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT    | Applicative  | integer   |  Sensor        | Max pending SIGFOX message retransmission count
	 *    TD_SENSOR_GATEWAY_MAX_DEVICE            | Applicative  | integer   |  Sensor        | Max attached devices to Gateway (Gateway itself count for 1 device)
	 *    TD_SENSOR_MAX_SWITCH                    | Applicative  | integer   |  Sensor        | Max number of handled switches
	 *    TD_SENSOR_MAX_SWITCH_EVENT			  | Applicative  | integer   |  Sensor        | Max number of switches event queue (for non IRQ processing)
	 *    TD_FLASH_MAX_DATA_POINTER               | Applicative  | integer   |  Flash Variable| Max number of flash variable handled
	 *    STACK_SIZE                              | Applicative  | integer   |  system wide   | System stack size in bytes
	 *    MODULE_REVISION                         | Board        | symbol    |  system wide   | REVISION_TD1202, REVISION_TD1204, REVISION_TD1208 (often defined in project defines)
	 *    PRODUCT_LED_POLARITY		              | Board        | integer   |  Bootloader    | 0:led lit when TIM2 set to VDD, 1:led lit when TIM2 set to GND
	 *    PRODUCT_LED_PORT		 	              | Board        | gpioPortx |  Bootloader    | Set port of Led (default : TIM2_PORT), 0xFF : no led
	 *    PRODUCT_LED_BIT		 	              | Board        | integer   |  Bootloader    | Set bit of Led (default : TIM2_BIT)
	 *    PRODUCT_INIT_DATA 	 	              | Board        | array     |  Bootloader    | Others port to initialize (see Init Data)
	 *    MAX_USER_SPI_ID						  | Board        | integer   |  Spi           | Max number of SPI users
	 *    PRODUCT_TYPE		                      | Chip         | integer   |  Bootloader    | 0:TD1202, 8:TD1204, 9:TD1205, 10:TD1208 (automatically set if not set and MODULE_REVISION defined)
	 *
	 *   These parameters are RF SYSTEM parameters do not MESS with them!
	 *
	 *   Define                                  | Level        | Type      |      Usage          | Description
	 *   :---------------------------------------|:------------:|:---------:|:-------------------:|:------------------------------------------------------------------------------------
	 *   TD_LOADER_TRANSMITTER				     | Applicative  | define    |  Bootloader         | if this define present, code will stop in bootloader for loader/transmit function
	 *   POWER_CRYSTAL_PORT		 	             | Chip         | gpioPortx |  Bootloader         | Set port of RF TCXO
	 *   POWER_CRYSTAL_BIT		                 | Chip         | integer   |  Bootloader         | Set bit of RF TCXO
	 *   SHTD_PORT		 	                     | Chip         | gpioPortx |  Bootloader         | Set port of RF Shutdown
	 *   SHTD_BIT		               			 | Chip         | integer   |  Bootloader         | Set bit of RF Shutdown
	 *   RADIO_INIT_DATA					     | Chip         | array     |  Bootloader         | Initial configuration of RF PIN
	 *   RADIO_INFO_PIN					         | Chip         | integer   |  td_rf              | RF PIN number connected to EFM32
	 *   FORCE_RADIO_RESET					     | Chip         | integer   |  td_rf              | Force RF reset at each reset
	 *   RADIO_PA_POWER					         | Chip         | integer   |  td_sigfox          | Default RF power (dBm)
	 *   LAN_PERIOD						         | Chip         | integer   |  td_sensor_gateway  | Receive sampling period (timer unit)
	 *   LAN_THRESHOLD						     | Chip         | integer   |  td_rf  			  | RSSI Threshold in LAN reception mode in 0.5dB step from -126dB max sensitivity (default : 32 aka -110dB)
	 *
	 *   These parameters are GPS SYSTEM parameters do not MESS with them!
	 *
	 *   Define                                  | Level        | Type      |      Usage     | Description
	 *   :---------------------------------------|:------------:|:---------:|:--------------:|-------------------------------
	 *   GPS_CS_PORT		 	                 | Chip         | gpioPortx |  Bootloader    | set port of GPS CS
	 *   GPS_CS_BIT		                         | Chip         | number    |  Bootloader    | set bit of GPS CS
	 *   GPS_IRQ_PORT		 	                 | Chip         | gpioPortx |  Bootloader    | set port of GPS IRQ
	 *   GPS_IRQ_BIT		                     | Chip         | number    |  Bootloader    | set bit of GPS IRQ
	 *   GPS_RESET_PORT		 	                 | Chip         | gpioPortx |  Bootloader    | set port of GPS RESET
	 *   GPS_RESET_BIT		                     | Chip         | number    |  Bootloader    | set bit of GPS RESET
	 *   GPS_VBCKP_PORT		 	                 | Chip         | gpioPortx |  Bootloader    | set port of GPS VBCKP
	 *   GPS_VBCKP_BIT		                     | Chip         | number    |  Bootloader    | set bit of GPS VBCKP
	 *   GPS_VIO_PORT		 	                 | Chip         | gpioPortx |  Bootloader    | set port of GPS VIO
	 *   GPS_VIO_BIT		                     | Chip         | number    |  Bootloader    | set bit of GPS VIO
	 *
	 *   These parameters are for General code use. It will force to include (or remove) blocks of code.
	 *   In general use case theses parameters are not useful, they are just here for special use case
	 *
	 *   Define                                  | Description
	 *   :---------------------------------------|-------------------------------
	 *   TD_GEOLOC_USE_CODE		 	 			 |Force use/not use the libtdgeoloc dynamic data
	 *   TD_SENSOR_USE_CODE		 	 			 |Force use/not use the libtdsensor dynamic data

	 *   These parameters are Code Size reduction parameters. If you remove code and use it after that, it will throw a Trap
	 *
	 *   Define                                  | Description
	 *   :---------------------------------------|-------------------------------
	 *   TD_SENSOR_GATEWAY_REMOVE_CODE		 	 |Remove Sensor Gateway code
	 *   TD_SENSOR_TRANSMITTER_REMOVE_CODE		 |Remove Sensor Transmitter code
	 *   TD_SENSOR_MONITOR_REMOVE_CODE		 	 |Remove Sensor Switch Monitor code
	 *   TD_GEOLOC_LOGGER_REMOVE_CODE		     |Remove Geolocation Flash logger code
	 *   TD_SIGFOX_REMOVE_CODE		 	         |Remove all SIGFOX code
	 *   TD_TRAP_RESET_CODE		                 |Reset only trap
	 *   TD_TRAP_MINI_CODE		                 |Remove standard printf trap handler, replace with minimal trap
	 *   TD_TRAP_PRINTF_CODE		 	         |Standard printf trap handler
	 *   TD_TRAP_FLASH_CODE		                 |Remove standard printf trap handler, replace with flash trap
	 *   TD_TRAP_FLASH_PRINTF_CODE		 	     |Add flash handler with standard printf handler.
	 *   TD_ALL_DUMP_REMOVE_CODE		 	     |Remove all xxx_DUMP code (same as enable one by one)
	 *   TD_GPIO_DUMP_REMOVE_CODE		 	     |Remove TD_GPIO_Dump() code
	 *   TD_RF_DUMP_REMOVE_CODE		 	 	     |Remove TD_RF_Dump() code
	 *   TD_IRQ_DUMP_REMOVE_CODE		 	 	 |Remove TD_GPIO_Dump() code
	 *   TD_SCHEDULER_DUMP_REMOVE_CODE		 	 |Remove TD_SCHEDULER_Dump() code
	 *   TD_UBX7_DUMP_REMOVE_CODE		 	 	 |Remove TD_UBX7_Dump() code
	 *   TD_SPILOCK_DUMP_REMOVE_CODE		 	 |Remove TD_SPILock_Dump() code
	 *
	 *   Note : for flash trap handler, remember to call TD_TRAP_DirectToFlash function in your code
	 *   don't forget to remove TD_TRAP_DirectToFlash to reclaim space if you not use it
	 *
	 * @} (end addtogroup PARAMETERS)
	 *
	 * @addtogroup INIT_DATA Initialization Data
	 * @brief Initialization Data
	 * @{
	 * @details
	 *   # Initialization Data
	 *
	 *   Data is an array of unsigned shorts, each entry set one pin state
	 *   For each entry, use
	 *
	 *   PIP(p,b,m,v)	: initialize one port
	 *      p : port (gpioPortx)
	 *      b : bit number (0=>15)
	 *      m : port mode (PI_DISABLED:disabled, PI_INPUT:input, PI_INPUT_PULL:input, PI_OUTPUT:push pull)
	 *      v : init value (0:low/pull-down, 1:high/pull-up)
	 *
	 *   Note :
	 *   	PI_DISABLED, initial value of 0 : no pull-down, initial value of 1 : pull-up
	 *   	PI_INPUT, no pull-up/pull-down
	 *   	PI_INPUT_PULL, initial value of 0 : pull-down, initial value of 1 : pull-up
	 *
	 *   PIS(p,s)	: initial port strength
	 *      p : port (gpioPortx)
	 *	  s : drive mode set (gpioDriveModexx)
	 *    example:
	 * @code
	 *    #define PRODUCT_INIT_DATA {PIP(gpioPortB, 1, PI_OUTPUT, 0), PIP(gpioPortF, 7, PI_OUTPUT, 1)}
	 * @endcode
	 *		set Port B1 to 0 and Port F7 to 1
	 *
	 *	PINULL : dummy entry, Do nothing. Initialization array length must be > 0 so, used for empty array
	 *
	 *	Note : PRODUCT_LED_PORT, PRODUCT_LED_BIT are initialized before Init Data.
	 *	if PRODUCT_LED_PORT is different from 0xFF, it set this port to push pull
	 *
	 * @} (end addtogroup INIT_DATA)
	 * @} (end addtogroup DYNAMIC_OPTIONS)
	 *
	 ******************************************************************************/

	/** @cond TD_CONFIG */

	/* Cook standard AT variables */
	/* If MANUFACTURER is not defined, we suppose no AT command are included */
#ifdef MANUFACTURER
#ifndef MANUFACTURER
#error("To use AT command, MANUFACTURER must be defined")
#endif
#ifndef PRODUCT
#error("To use AT command, PRODUCT must be defined")
#endif
#ifndef HARDWARE_VERSION
#error("To use AT command, HARDWARE_VERSION must be defined")
#endif
#ifndef SOFTWARE_VERSION
#error("To use AT command, SOFTWARE_VERSION must be defined")
#endif
#ifndef RELEASE_DATE
#error("To use AT command, RELEASE_DATE must be defined")
#endif
#ifndef SERIAL_NUMBER
#error("To use AT command, SERIAL_NUMBER must be defined")
#endif
#define TD_STDLIB_DATA_AT	\
    /** AT device Manufacturer */\
    char const *CONFIG_MANUFACTURER = MANUFACTURER" "PRODUCT;\
    \
    /** AT hardware revision */\
    char const *CONFIG_HARDWARE_VERSION = HARDWARE_VERSION;\
    \
    /** AT software revision */\
    char const *CONFIG_SOFTWARE_VERSION = SOFTWARE_VERSION;\
    \
    /** AT firmware release date */\
    char const *CONFIG_RELEASE_DATE = RELEASE_DATE;\
    \
    /** AT device serial number */
	char *CONFIG_SERIAL_NUMBER = SERIAL_NUMBER;
#else /*MANUFACTURER*/
#define TD_STDLIB_DATA_AT
#endif /*MANUFACTURER*/

	/* Check MANDATORY variables and set others to default values */
#ifdef MODULE_REVISION	/* Do we have standard board? */
#ifndef PRODUCT_TYPE	/* Do we want to define for us? */

#if MODULE_REVISION == REVISION_TD1202
#define PRODUCT_TYPE			0
#define CHIP_TD1202
#endif

#if MODULE_REVISION == REVISION_TD1204
#define PRODUCT_TYPE			8
#define CHIP_TD1204
#endif

#if MODULE_REVISION == REVISION_TD1205
#define PRODUCT_TYPE			9
#define PRODUCT_LED_POLARITY 	1
#define CHIP_TD1205
#endif

#if MODULE_REVISION == REVISION_TD1208
#define PRODUCT_TYPE			10
#define CHIP_TD1208
#endif

#endif /* PRODUCT_TYPE*/
#endif /* MODULE_REVISION*/

	/* Definition of standard TD Chips */
#ifdef CHIP_TD1202
#define POWER_CRYSTAL_PORT      gpioPortF           /**< Power RF Crystal port */
#define POWER_CRYSTAL_BIT       2					/**< Power RF Crystal bit */
#define	SHTD_PORT               gpioPortC           /**< RF GPIO 1 port */
#define	SHTD_BIT                13					/**< RF GPIO 1 bit */
#define RADIO_INIT_DATA			{0x01, 0x00, 21, 0x00, 0x00}
#define RADIO_INFO_PIN			2
#define TD_GEOLOC_LOGGER_REMOVE_CODE
#ifndef	TD_GEOLOC_USE_CODE
#define TD_GEOLOC_USE_CODE				0
#endif
#endif

#ifdef CHIP_TD1204
#define POWER_CRYSTAL_PORT      gpioPortF           /**< Power RF Crystal port */
#define POWER_CRYSTAL_BIT       2					/**< Power RF Crystal bit */
#define	SHTD_PORT               gpioPortC           /**< RF GPIO 1 port */
#define	SHTD_BIT                13					/**< RF GPIO 1 bit */
#define GPS_CS_PORT 			USR0_PORT
#define GPS_CS_BIT 				USR0_BIT
#define GPS_IRQ_PORT 			USR3_PORT
#define GPS_IRQ_BIT 			USR3_BIT
#define GPS_RESET_PORT 			0x10
#define GPS_RESET_BIT 			2
#define GPS_VBCKP_PORT 			0x10 				/**< port 0x10 is Silabs */
#define GPS_VBCKP_BIT 			3
#define GPS_VIO_PORT 			USR2_PORT
#define GPS_VIO_BIT 			USR2_BIT
#define RADIO_INFO_PIN			0

#ifndef PRODUCT_INIT_DATA
#define PRODUCT_INIT_DATA\
    {PIP(gpioPortB,13,PI_DISABLED,1),	/* CS pull-up to 1 for GPS and accelerometer */ \
    PIP(gpioPortC,15,PI_OUTPUT,1)}
#define RADIO_INIT_DATA			{21, 0x00, 0x01, 0x02, 0x00}
#endif
#ifndef	TD_GEOLOC_USE_CODE
#define TD_GEOLOC_USE_CODE				1
#endif
#endif /* CHIPTD1204 */

#ifdef CHIP_TD1205
#define POWER_CRYSTAL_PORT      gpioPortA           /**< Power RF Crystal port */
#define POWER_CRYSTAL_BIT       0					/**< Power RF Crystal bit */
#define	SHTD_PORT               gpioPortC           /**< RF GPIO 1 port */
#define	SHTD_BIT                13					/**< RF GPIO 1 bit */
#define GPS_CS_PORT 			USR0_PORT
#define GPS_CS_BIT 				USR0_BIT
#define GPS_IRQ_PORT 			USR3_PORT
#define GPS_IRQ_BIT 			USR3_BIT
#define GPS_RESET_PORT 			gpioPortB
#define GPS_RESET_BIT 			11
#define GPS_VBCKP_PORT 			gpioPortF
#define GPS_VBCKP_BIT 			2
#define GPS_VIO_PORT 			USR2_PORT
#define GPS_VIO_BIT 			USR2_BIT
#define RADIO_INFO_PIN			1

#ifndef PRODUCT_INIT_DATA
#define PRODUCT_INIT_DATA\
    {PIP(gpioPortB,13,PI_DISABLED, 1),	/* CS pull-up to 1 for GPS and accelerometer */ \
    PIP(gpioPortC,15,PI_DISABLED, 1)}
#endif
#define RADIO_INIT_DATA			{33, 21, 32, 1, 0x00}
#define RADIO_PA_POWER			20
#ifndef	TD_GEOLOC_USE_CODE
#define TD_GEOLOC_USE_CODE				1
#endif
#endif /* CHIPTD1205 */

#ifdef CHIP_TD1208
#define POWER_CRYSTAL_PORT      gpioPortF           /**< Power RF Crystal port */
#define POWER_CRYSTAL_BIT       2					/**< Power RF Crystal bit */
#define	SHTD_PORT               gpioPortC           /**< RF GPIO 1 port */
#define	SHTD_BIT                13					/**< RF GPIO 1 bit */
#define RADIO_INIT_DATA			{0x01, 0x00, 21, 0x00, 0x00}
#define RADIO_INFO_PIN			2
#define TD_GEOLOC_LOGGER_REMOVE_CODE
#ifndef	TD_GEOLOC_USE_CODE
#define TD_GEOLOC_USE_CODE				0
#endif
#endif /* CHIPTD1208 */

#if IS_EMPTY(TD_SENSOR_USE_CODE)
#error("TD_SENSOR_USE_CODE must be set to 0 or 1, or removed")
#endif

#if IS_EMPTY(TD_GEOLOC_USE_CODE)
#error("TD_GEOLOC_USE_CODE must be set to 0 or 1, or removed")
#endif

	/* For all chip, by default we would use sensor */
#ifndef	TD_SENSOR_USE_CODE
#define TD_SENSOR_USE_CODE				1
#endif

	/* PRODUCT_TYPE is mandatory, see error message for standard value or ask Telecom Design for custom design
	 * This type is used by the integrated RF loader to prevent bricking products with wrong firmware updates.
	 */
#ifndef PRODUCT_TYPE
#error("You must specify a product type : PRODUCT_TYPE=x. TD1202_EVB:0 TD1204_EVB:8 TD1205_EVB:9 TD1208_EVB:10")
#endif

	/* PRODUCT_LED_POLARITY : value to set on PIN TIM2(or PRODUCT_LED_PORT/PRODUCT_LED_BIT) to turn OFF information led (used by the integrated bootloader)
	 * default value : led active high
	 */
#ifndef PRODUCT_LED_POLARITY
#define PRODUCT_LED_POLARITY 0
#endif

#if IS_EMPTY(PRODUCT_LED_POLARITY)
#error("PRODUCT_LED_POLARITY is empty, must be set to 0 or 1")
#else

#if ((PRODUCT_LED_POLARITY != 0) && (PRODUCT_LED_POLARITY != 1))
#error("PRODUCT_LED_POLARITY must be set to 0 or 1")
#endif

#endif /* PRODUCT_LED_POLARITY */

	/* PRODUCT_LED_PORT : port used to turn on information led (used by the integrated bootloader)
	 * default value : TIM2_PORT (TDxxxx modules)
	 */
#ifndef PRODUCT_LED_PORT
#define PRODUCT_LED_PORT TIM2_PORT
#endif

	/* PRODUCT_LED_BIT : bit used to turn on information led (used by the integrated bootloader)
	 * default value : TIM2_BIT (TDxxxx modules)
	 */
#ifndef PRODUCT_LED_BIT
/***********************************************************************************
 *      _                                                                   _      *
 *     / \                                                                 / \     *
 *    / ! \   THA alternate LED port due to chip issue on port TIM2_BIT   / ! \    *
 *   /_____\                                                             /_____\   *
 *                                                                                 *
 ***********************************************************************************/
#define PRODUCT_LED_BIT TIM2_BIT
//#define PRODUCT_LED_BIT ADC0_BIT
#endif

	/* PRODUCT_LED_BLINK : set to 1, blink led during each flash sector write
	 * default value : false
	 * Note : remember if you want to change this default value for TDxxxx module to update ALL user bootloader documentations ...
	 */
#ifndef PRODUCT_LED_BLINK
#define PRODUCT_LED_BLINK 0
#endif

#if (IS_EMPTY(PRODUCT_LED_BLINK))
#error("PRODUCT_LED_BLINK is empty, must be set to 0 or 1")
#else

#if ((PRODUCT_LED_BLINK != 0) && (PRODUCT_LED_BLINK != 1))
#error("PRODUCT_LED_BLINK must be set to 0 or 1")
#endif

#endif /* PRODUCT_LED_BLINK */

	/* PRODUCT_BOOTLOADER_CHANNEL : channel to use for bootloader feature
	 * default value : 255
	 * Note :
	 * 	channel 25 (868.4MHz) and 26 (869.80MHz) are allowed in France
	 * 	channel 254 is on 856.798185
	 * 	channel 255 is on 855.173185
	 */
#ifndef PRODUCT_BOOTLOADER_CHANNEL
#define PRODUCT_BOOTLOADER_CHANNEL	255
#endif

	/* PRODUCT_BOOTLOADER_SKIP : set to true, do not execute bootloader
	 * default value : 0
	 */
#ifndef PRODUCT_BOOTLOADER_SKIP
#define PRODUCT_BOOTLOADER_SKIP	0
#endif

#if IS_EMPTY(PRODUCT_BOOTLOADER_SKIP)
#error("PRODUCT_BOOTLOADER_SKIP is empty, must be set to 0 or 1")
#endif

	/* RADIO_INIT_DATA : RF IO port initialized in bootloader
	* default value : empty
	*/
#ifndef RADIO_INIT_DATA
#error("RADIO_INIT_DATA must be defined !\r\n"\
"Please remove your PRODUCT_TYPE definition and keep only MODULE_REVISION definition.\r\n"\
"if PRODUCT_TYPE is externally defined, all SYSTEM define must also occurs")
#endif

	/* PRODUCT_INIT_DATA : CPU IO port optionally initialized in bootloader
	 * default value : 0x00F0. This value is a "do nothing" value
	 */
#ifndef PRODUCT_INIT_DATA
#define PRODUCT_INIT_DATA {PINULL}
#endif

	/* RADIO_INFO_PIN : RF IO port link to EFM32 CPU
	 * default value : empty
	 */
#ifndef RADIO_INFO_PIN
#error("RADIO_INFO_PIN must be defined ! (if PRODUCT_TYPE is externally defined all SYSTEM define must also occurs)")
#endif

	/* FORCE_RADIO_RESET : force RF reset at each reset
	 * default value : 0
	 */
#ifndef FORCE_RADIO_RESET
#define FORCE_RADIO_RESET	0
#endif

	/* RADIO_PA_POWER : default RF PA power for SIGFOX
	 * default value : 14
	 */
#ifndef RADIO_PA_POWER
#define RADIO_PA_POWER	14
#endif

#ifndef MAX_USER_SPI_ID
#define MAX_SPI_ID MAX_SYSTEM_SPI_ID
#else
#define MAX_SPI_ID (MAX_SYSTEM_SPI_ID + MAX_USER_SPI_ID)
#endif

#ifdef GPS_CS_PORT
#ifndef GPS_CS_BIT
#error("GPS_CS_BIT is undefined")
#endif
#ifndef GPS_IRQ_PORT
#error("GPS_IRQ_PORT is undefined")
#endif
#ifndef GPS_IRQ_BIT
#error("GPS_IRQ_BIT is undefined")
#endif
#ifndef GPS_VIO_PORT
#error("GPS_VIO_PORT is undefined")
#endif
#ifndef GPS_RESET_PORT
#error("GPS_RESET_PORT is undefined")
#endif
#ifndef GPS_RESET_BIT
#error("GPS_RESET_BIT is undefined")
#endif
#ifndef GPS_VBCKP_PORT
#error("GPS_VBCKP_PORT is undefined")
#endif
#ifndef GPS_VBCKP_BIT
#error("GPS_VBCKP_BIT is undefined")
#endif
#endif /* GPS_CS_PORT (Do we want GPS?) */

	/* Board Support Package definition */
#ifdef GPS_CS_PORT
#define TD_BSP_GPS\
     uint8_t const CONFIG_GPS_CS_PORT = GPS_CS_PORT;\
     uint8_t const CONFIG_GPS_CS_BIT = GPS_CS_BIT;\
     uint8_t const CONFIG_GPS_IRQ_PORT = GPS_IRQ_PORT;\
     uint8_t const CONFIG_GPS_IRQ_BIT = GPS_IRQ_BIT;\
     uint8_t const CONFIG_GPS_RESET_PORT = GPS_RESET_PORT;\
     uint8_t const CONFIG_GPS_RESET_BIT = GPS_RESET_BIT;\
     uint8_t const CONFIG_GPS_VBCKP_PORT = GPS_VBCKP_PORT;\
     uint8_t const CONFIG_GPS_VBCKP_BIT = GPS_VBCKP_BIT;\
     uint8_t const CONFIG_GPS_VIO_PORT = GPS_VIO_PORT;\
     uint8_t const CONFIG_GPS_VIO_BIT = GPS_VIO_BIT;
#else
#define TD_BSP_GPS
#endif

#define TD_BSP\
    TD_BSP_GPS\
    GPIO_Port_TypeDef const CONFIG_POWER_CRYSTAL_PORT = POWER_CRYSTAL_PORT;\
    uint8_t const CONFIG_POWER_CRYSTAL_BIT = POWER_CRYSTAL_BIT;\
    GPIO_Port_TypeDef const CONFIG_SHTD_PORT = SHTD_PORT;\
    uint8_t const CONFIG_SHTD_BIT = SHTD_BIT;\
    uint8_t const CONFIG_RADIO_INFO_PIN = RADIO_INFO_PIN;\
    uint8_t const CONFIG_FORCE_RADIO_RESET = FORCE_RADIO_RESET;\
    uint8_t const CONFIG_RADIO_PA_POWER = RADIO_PA_POWER;

#if !defined(POWER_CRYSTAL_PORT) && IS_EMPTY(POWER_CRYSTAL_PORT)
#error("PRODUCT_CRYSTAL_PORT must be set (if PRODUCT_TYPE is externally defined all SYSTEM define must also occurs)")
#endif

#if !defined(POWER_CRYSTAL_BIT) && IS_EMPTY(POWER_CRYSTAL_BIT)
#error("PRODUCT_CRYSTAL_BIT must be set (if PRODUCT_TYPE is externally defined all SYSTEM define must also occurs)")
#endif

#if !defined(SHTD_PORT) && IS_EMPTY(SHTD_PORT)
#error("SHTD_PORT must be set (if PRODUCT_TYPE is externally defined all SYSTEM define must also occurs)")
#endif

#if !defined(SHTD_BIT) && IS_EMPTY(SHTD_BIT)
#error("SHTD_BIT must be set (if PRODUCT_TYPE is externally defined all SYSTEM define must also occurs)")
#endif

#ifdef TD_LOADER_TRANSMITTER
	void LoaderTransmitterFunc(void);
#define TD_LOADER_TRANSMITTER_FUNC  void (*LoaderTransmitter)(void) = LoaderTransmitterFunc;
#else
#define TD_LOADER_TRANSMITTER_FUNC  void (*LoaderTransmitter)(void) = NULL;
#endif

	/* TD_SCHEDULER_MAX_TIMER : total number of Scheduler Timer. Cost gain : ram usage, system performance.
	 * Be careful, standard libraries use some timers ...
	 * default value : TD1202 = 10 others = 100 */
#ifndef TD_SCHEDULER_MAX_TIMER
#ifdef EFM32TG210F32
#define TD_SCHEDULER_MAX_TIMER	10
#else
#define TD_SCHEDULER_MAX_TIMER	100
#endif
#endif /* TD_SCHEDULER_MAX_TIMER */

	/* TD_SCHEDULER_MAX_QUEUE : size of timer execution queue. Cost gain : ram usage.
	 * Be careful, if timer queue is too short, some timer events will get lost!
	 * default value : TD1202 = 10 others = 100 */
#ifndef TD_SCHEDULER_MAX_QUEUE
#ifdef EFM32TG210F32
#define TD_SCHEDULER_MAX_QUEUE	10
#else
#define TD_SCHEDULER_MAX_QUEUE	100
#endif
#endif /* TD_SCHEDULER_MAX_QUEUE */

#if TD_SENSOR_USE_CODE

#include <td_sensor_transmitter.h>

#ifndef TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT
#ifdef EFM32TG210F32
#define TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT 10
#else
#define TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT 10
#endif
#endif

#ifndef TD_SENSOR_TRANSMITTER_MAX_TRANSMIT
#define TD_SENSOR_TRANSMITTER_MAX_TRANSMIT 1
#endif

#if TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT > 0
#define TD_STDLIB_DATA_SENSOR_TRANSMITTER_RETRANSMISSION\
      /** Transmitter retransmit queue */\
      uint8_t const CONFIG_TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT = TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT;\
      static TD_SENSOR_TRANSMITTER_Retransmission_t TransmitterRetransmissionList[TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT];\
      TD_SENSOR_TRANSMITTER_Retransmission_t * TD_SENSOR_TRANSMITTER_RetransmissionList = TransmitterRetransmissionList;
#else
#define TD_STDLIB_DATA_SENSOR_TRANSMITTER_RETRANSMISSION\
      uint8_t const CONFIG_TD_SENSOR_TRANSMITTER_MAX_RETRANSMIT = 0;\
      TD_SENSOR_TRANSMITTER_Retransmission_t * TD_SENSOR_TRANSMITTER_RetransmissionList = NULL;
#endif

#if TD_SENSOR_TRANSMITTER_MAX_TRANSMIT > 0
#define TD_STDLIB_DATA_SENSOR_TRANSMITTER_TRANSMISSION\
      /** Transmitter transmit queue */\
      uint8_t const CONFIG_TD_SENSOR_TRANSMITTER_MAX_TRANSMIT = TD_SENSOR_TRANSMITTER_MAX_TRANSMIT;\
      static TD_SENSOR_TRANSMITTER_Transmission_t TransmitterTransmissionQueue[TD_SENSOR_TRANSMITTER_MAX_TRANSMIT];\
      TD_SENSOR_TRANSMITTER_Transmission_t * TD_SENSOR_TRANSMITTER_TransmissionQueue = TransmitterTransmissionQueue;
#else
#define TD_STDLIB_DATA_SENSOR_TRANSMITTER_TRANSMISSION\
      uint8_t const CONFIG_TD_SENSOR_TRANSMITTER_MAX_TRANSMIT = 0;\
      TD_SENSOR_TRANSMITTER_Transmission_t * TD_SENSOR_TRANSMITTER_TransmissionQueue = NULL;
#endif

#define TD_STDLIB_DATA_SENSOR_TRANSMITTER\
    TD_STDLIB_DATA_SENSOR_TRANSMITTER_RETRANSMISSION\
    TD_STDLIB_DATA_SENSOR_TRANSMITTER_TRANSMISSION
#else	/* No td_sensor_transmitter.h included */
#define TD_STDLIB_DATA_SENSOR_TRANSMITTER
#endif /* TD_SENSOR_USE_CODE */

#if TD_SENSOR_USE_CODE

#include <td_sensor_gateway.h>

#ifndef TD_SENSOR_GATEWAY_MAX_DEVICE
#define TD_SENSOR_GATEWAY_MAX_DEVICE 15
#endif

#if TD_SENSOR_GATEWAY_MAX_DEVICE > 0
#define TD_STDLIB_DATA_SENSOR_GATEWAY\
    /**Registered Devices List and count*/\
    uint8_t const CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE = TD_SENSOR_GATEWAY_MAX_DEVICE;\
    static TD_SENSOR_GATEWAY_Device_t GatewayDeviceList[TD_SENSOR_GATEWAY_MAX_DEVICE];\
    TD_SENSOR_GATEWAY_Device_t * DeviceList = GatewayDeviceList;
#else
#define TD_STDLIB_DATA_SENSOR_GATEWAY\
    /**Registered Devices List and count*/\
    uint8_t const CONFIG_TD_SENSOR_GATEWAY_MAX_DEVICE = TD_SENSOR_GATEWAY_MAX_DEVICE;\
    TD_SENSOR_GATEWAY_Device_t * DeviceList = NULL;
#endif

#else	/* No td_sensor_gateway.h included */
#define TD_STDLIB_DATA_SENSOR_GATEWAY
#endif /* TD_SENSOR_USE_CODE */

	/* If we have td_data_sensor.h included, implement transmitter data */
#if TD_SENSOR_USE_CODE
#include <td_sensor.h>

#ifndef TD_SENSOR_MAX_SWITCH
#define TD_SENSOR_MAX_SWITCH 8
#endif

#ifndef TD_SENSOR_MAX_SWITCH_EVENT
#define TD_SENSOR_MAX_SWITCH_EVENT 5
#endif

#if TD_SENSOR_MAX_SWITCH > 0
#define TD_STDLIB_DATA_SENSOR_SWITCH\
    /** Switch monitoring */\
    uint8_t const CONFIG_TD_SENSOR_MAX_SWITCH = TD_SENSOR_MAX_SWITCH;\
    static TD_SENSOR_SwitchConfiguration_t SensorSwitchConfig[TD_SENSOR_MAX_SWITCH];\
    TD_SENSOR_SwitchConfiguration_t * TD_SENSOR_SwitchConfig = SensorSwitchConfig;\
    /** Switch monitoring - non irq processing (add 1 for empty buffer stage needed) */\
    uint8_t const CONFIG_TD_SENSOR_MAX_SWITCH_EVENT = TD_SENSOR_MAX_SWITCH_EVENT+1;\
    static TD_SENSOR_SwitchState_t SensorSwitchStateList[TD_SENSOR_MAX_SWITCH_EVENT];\
    TD_SENSOR_SwitchState_t * TD_SENSOR_SwitchStateList = SensorSwitchStateList;
#else
#define TD_STDLIB_DATA_SENSOR_SWITCH\
    /** Switch monitoring */\
    uint8_t const CONFIG_TD_SENSOR_MAX_SWITCH = TD_SENSOR_MAX_SWITCH;\
    TD_SENSOR_SwitchConfiguration_t * TD_SENSOR_SwitchConfig = NULL;\
    /** Switch monitoring - non irq processing */\
    uint8_t const CONFIG_TD_SENSOR_MAX_SWITCH_EVENT = TD_SENSOR_MAX_SWITCH_EVENT;\
    TD_SENSOR_SwitchState_t * TD_SENSOR_SwitchStateList = NULL;
#endif
#else	/* No td_data_sensor.h included */
#define TD_STDLIB_DATA_SENSOR_SWITCH
#endif /* TD_SENSOR_USE_CODE */

#if PRODUCT_BOOTLOADER_SKIP
	TD_BOOT_Handler_t const TD_BOOT_Handler = NULL;
#endif

#ifndef LAN_PERIOD
#define LAN_PERIOD T1S
#endif

#ifndef LAN_THRESHOLD
#define LAN_THRESHOLD 32
#endif

#define TD_STDLIB_SCHEDULER\
  /** Timer list */\
  uint8_t const CONFIG_TD_SCHEDULER_MAX_TIMER = TD_SCHEDULER_MAX_TIMER;\
  static TD_SCHEDULER_timer_t SchedulerTimer[TD_SCHEDULER_MAX_TIMER];\
  TD_SCHEDULER_timer_t * TD_SCHEDULER_Timer = SchedulerTimer;\
  \
  /** Scheduler callback queue */\
  uint8_t const CONFIG_TD_SCHEDULER_MAX_QUEUE = TD_SCHEDULER_MAX_QUEUE;\
  static TD_SCHEDULER_callback_t SchedulerCallbackQueue[TD_SCHEDULER_MAX_QUEUE];\
  TD_SCHEDULER_callback_t * TD_SCHEDULER_CallbackQueue  = SchedulerCallbackQueue;

#ifdef TD_FLASH_MAX_DATA_POINTER
#define MAX_FLASH_DATA_POINTER TD_FLASH_MAX_DATA_POINTER
#else
	/** Maximum number of data pointers */
#ifdef EFM32TG210F32
#define MAX_FLASH_DATA_POINTER 10
#else
#define MAX_FLASH_DATA_POINTER 25
#endif
#endif /* TD_FLASH_MAX_DATA_POINTER */

	uint8_t const CONFIG_TD_FLASH_MAX_DATA_POINTER = MAX_FLASH_DATA_POINTER;
	static TD_FLASH_variable_t FlashDataList[MAX_FLASH_DATA_POINTER];
	TD_FLASH_variable_t *TD_FLASH_DataList = FlashDataList;

#define _PRODUCT_TYPE PRODUCT_TYPE
	/* These definitions are the core "dynamic" library data
	 * They must be called outside all function on only one file of each project
	 * PRODUCT_TYPE must always be defined
	 */
	TD_STDLIB_DATA_AT

	/** Product type */
	char const CONFIG_PRODUCT_TYPE = PRODUCT_TYPE;

	/** Bootloader Led Polarity */
	char const CONFIG_PRODUCT_LED_POLARITY = PRODUCT_LED_POLARITY;

	/** Bootloader Led Port */
	GPIO_Port_TypeDef const CONFIG_PRODUCT_LED_PORT = PRODUCT_LED_PORT;

	/** Bootloader Led Bit */
	char const CONFIG_PRODUCT_LED_BIT = PRODUCT_LED_BIT;

	/** Bootloader Led Blink */
	char const CONFIG_PRODUCT_LED_BLINK = PRODUCT_LED_BLINK;

	/** Bootloader Channel */
	unsigned char const CONFIG_PRODUCT_BOOTLOADER_CHANNEL = PRODUCT_BOOTLOADER_CHANNEL;

	/** Bootloader Init Data */
	unsigned short const CONFIG_PRODUCT_INIT_DATA[] = PRODUCT_INIT_DATA;

	/** RF Init Data */
	unsigned char const CONFIG_RADIO_INIT_DATA[] = RADIO_INIT_DATA;

	/** LAN sampling period */
	unsigned short const CONFIG_LAN_PERIOD = LAN_PERIOD;

	/** LAN threshold level */
	unsigned char CONFIG_LAN_THRESHOLD = LAN_THRESHOLD;

	/** SPI Max user and data */
	unsigned char const CONFIG_MAX_SPI_ID = MAX_SPI_ID;
	TD_SPI_Conf_t TD_SPI_Conf_[MAX_SPI_ID + 1];
	TD_SPI_Conf_t *TD_SPI_Conf = TD_SPI_Conf_;

	TD_LOADER_TRANSMITTER_FUNC
	unsigned char const CONFIG_PRODUCT_INIT_DATA_SIZE = sizeof(CONFIG_PRODUCT_INIT_DATA) / sizeof(unsigned short);
	TD_STDLIB_SCHEDULER
	TD_STDLIB_DATA_SENSOR_TRANSMITTER
	TD_STDLIB_DATA_SENSOR_GATEWAY
	TD_STDLIB_DATA_SENSOR_SWITCH
	TD_BSP
	/* This line must remain empty - end of definition */

	/* Handle code reduction system */
#if defined(TD_TRAP_MINI_CODE)
	TD_TRAP_callback_t	TD_Trap_Callback = (TD_TRAP_callback_t)TD_TRAP_Mini_Callback;
	TD_TRAP_callback_t	const TD_Trap_Callback2 = (TD_TRAP_callback_t)TD_TRAP_Mini_Callback;
#elif defined(TD_TRAP_FLASH_CODE)
	TD_TRAP_callback_t	TD_Trap_Callback = (TD_TRAP_callback_t)TD_TRAP_Flash_Callback;
	TD_TRAP_callback_t	const TD_Trap_Callback2 = (TD_TRAP_callback_t)TD_TRAP_Mini_Callback;
#elif defined(TD_TRAP_FLASH_PRINTF_CODE)
	TD_TRAP_callback_t	TD_Trap_Callback = (TD_TRAP_callback_t)TD_TRAP_Mini_Callback;
	TD_TRAP_callback_t	const TD_Trap_Callback2 = (TD_TRAP_callback_t)TD_TRAP_Mini_Callback;
#elif defined(TD_TRAP_PRINTF_CODE)
	TD_TRAP_callback_t	TD_Trap_Callback = TD_TRAP_Printf_Callback;
	TD_TRAP_callback_t const TD_Trap_Callback2 = TD_TRAP_Printf_Callback;
#elif defined(TD_TRAP_RESET_CODE)
	TD_TRAP_callback_t	TD_Trap_Callback = TD_TRAP_Reset_Callback;
	TD_TRAP_callback_t const TD_Trap_Callback2 = TD_TRAP_Reset_Callback;
#else
	TD_TRAP_callback_t	TD_Trap_Callback = TD_TRAP_Printf_Callback;
	TD_TRAP_callback_t const TD_Trap_Callback2 = TD_TRAP_Printf_Callback;
#endif

#if TD_SENSOR_USE_CODE
#include <td_sensor.h>

#ifdef TD_SENSOR_GATEWAY_REMOVE_CODE
	TD_LAN_callback_t const TD_SENSOR_GATEWAY_FrameReceived_ = (TD_LAN_callback_t)TD_TRAP_HERE;
#else
	TD_LAN_callback_t const TD_SENSOR_GATEWAY_FrameReceived_ = TD_SENSOR_GATEWAY_FrameReceived;
#endif
#ifdef TD_SENSOR_MONITOR_REMOVE_CODE
	TD_SENSOR_MonitorInit_t const TD_SENSOR_MonitorInit_ = (TD_SENSOR_MonitorInit_t)TD_TRAP_HERE;
#else
	TD_SENSOR_MonitorInit_t const TD_SENSOR_MonitorInit_ = TD_SENSOR_MonitorInit;
#endif

#include <td_sensor_transmitter.h>
#ifdef TD_SENSOR_TRANSMITTER_REMOVE_CODE
	TD_SENSOR_TRANSMITTER_Init_t const TD_SENSOR_TRANSMITTER_Init_ = (TD_SENSOR_TRANSMITTER_Init_t)TD_TRAP_HERE;
	TD_SENSOR_TRANSMITTER_Process_t const TD_SENSOR_TRANSMITTER_Process_ = (TD_SENSOR_TRANSMITTER_Init_t)TD_TRAP_HERE;
#else
	TD_SENSOR_TRANSMITTER_Init_t const TD_SENSOR_TRANSMITTER_Init_ = TD_SENSOR_TRANSMITTER_Init;
	TD_SENSOR_TRANSMITTER_Process_t const TD_SENSOR_TRANSMITTER_Process_ = TD_SENSOR_TRANSMITTER_Process;
#endif
#endif /* TD_SENSOR_USE_CODE */

#if TD_GEOLOC_USE_CODE
#include <td_geoloc.h>
#ifdef TD_GEOLOC_LOGGER_REMOVE_CODE
	TD_FLASH_InitLogger_t const TD_FLASH_InitLogger_ = (TD_FLASH_InitLogger_t)TD_TRAP_HERE;
	TD_GEOLOC_Log_t const TD_GEOLOC_Log_ = (TD_GEOLOC_Log_t)TD_TRAP_HERE;
#else
	TD_FLASH_InitLogger_t const TD_FLASH_InitLogger_ = TD_FLASH_InitLogger;
	TD_GEOLOC_Log_t const TD_GEOLOC_Log_ = TD_GEOLOC_Log;
#endif
#endif /* TD_GEOLOC_USE_CODE */

#ifdef TD_SIGFOX_REMOVE_CODE
	TD_SIGFOX_Init_t const TD_SIGFOX_Init = (TD_SIGFOX_Init_t)NULL;
	TD_SIGFOX_Send_t const TD_SIGFOX_Send = (TD_SIGFOX_Send_t)TD_TRAP_HERE;
#else
	TD_SIGFOX_Init_t const TD_SIGFOX_Init = TD_SIGFOX_Init_;
	TD_SIGFOX_Send_t const TD_SIGFOX_Send = TD_SIGFOX_Send_;
#endif

#ifdef 	TD_ALL_DUMP_REMOVE_CODE
#define TD_GPIO_DUMP_REMOVE_CODE
#define TD_RF_DUMP_REMOVE_CODE
#define TD_IRQ_DUMP_REMOVE_CODE
#define TD_SCHEDULER_DUMP_REMOVE_CODE
#define TD_GEOLOC_DUMP_REMOVE_CODE
#define TD_SPILOCK_DUMP_REMOVE_CODE
#define TD_ACCELERO_DUMP_REMOVE_CODE
#define TD_SENSOR_DUMP_REMOVE_CODE
#endif

#if TD_GEOLOC_USE_CODE
/* If there is a compilation error at this point
 * -if you don't want to use libtdgeoloc : add #define TD_GEOLOC_USE_CODE 0
 * - if you want to use libtdgeoloc : add libtdgeoloc/inc include path and the
 *    path to the libtdgeoloc library corresponding to your build configuration
 *    in your project settings
 */
#include <td_accelero.h>

#endif

void TD_RF_Dump(void);

TD_Dump_Func_t const TD_TRAP_SystemDumpFunc [] = {
#ifdef TD_GPIO_DUMP_REMOVE_CODE
			NULL,
#else
			TD_GPIO_Dump,
#endif
#if defined(TD_RF_DUMP_REMOVE_CODE)
			NULL,
#else
			TD_RF_Dump,
#endif
#ifdef TD_IRQ_DUMP_REMOVE_CODE
			NULL,
#else
			TD_IRQ_Dump,
#endif
#ifdef TD_SCHEDULER_DUMP_REMOVE_CODE
			NULL,
#else
			TD_SCHEDULER_Dump,
#endif
#if defined(TD_GEOLOC_DUMP_REMOVE_CODE) || TD_GEOLOC_USE_CODE==0
			NULL,
#else
			TD_GEOLOC_Dump,
#endif
#ifdef TD_SPILOCK_DUMP_REMOVE_CODE
			NULL,
#else
			TD_SPI_LockDump,
#endif
#if defined(TD_ACCELERO_DUMP_REMOVE_CODE) || TD_GEOLOC_USE_CODE==0
			NULL,
#else
			TD_ACCELERO_Dump,
#endif
#if defined(TD_ACCELERO_DUMP_REMOVE_CODE) || TD_SENSOR_USE_CODE==0
			NULL,
#else
			TD_SENSOR_Dump
#endif

		};

uint8_t const TD_TRAP_MaxSystemDump = sizeof (TD_TRAP_SystemDumpFunc) / sizeof (TD_Dump_Func_t);


#if defined(__ICCARM__)
#ifdef STACK_SIZE
#error("Stack size must be modified directly in .ICF file with IAR. If you want TD1202/1208 source code compatibility use, #if defined(__GNUC__) around #define STACK_SIZE")
#endif
#endif
#if defined(__GNUC__)
#ifndef STACK_SIZE
#define STACK_SIZE	0x400
#endif

	/* Create a custom stack with name SYMBOL, aligned to ALIGNMENT bytes, sized by
	   SIZE bytes, but possibly shortened such that the initial stack pointer
	   (symbol __cs3_stack) that points to the block's last extent is aligned to
	   ALIGNMENT bytes, too.
	   HACK : we must add 'used' attributed to not have stack removed by gentle GCC compiler in
	   its size optimization process ...
	   */
#define TD_CS3_STACK_SYMBOL(symbol, size, alignment) \
  static char __attribute__ ((aligned (alignment),used)) \
    symbol[(size - ((size) % (alignment)))]; \
  asm (".global __cs3_stack"); \
  asm ("__cs3_stack = " #symbol " + (" #size ") - (" #alignment ")" \
       " - ((" #size ") % (" #alignment "))")
#define TD_STACK(size)\
    TD_CS3_STACK_SYMBOL(__cs3_stack_block, (size), 16);
	TD_STACK(STACK_SIZE)
#endif

#ifndef STACK_SIZE
	uint16_t const CONFIG_STACK_SIZE = 0x400;
#else
	uint16_t const CONFIG_STACK_SIZE = STACK_SIZE;
#endif

#ifdef __cplusplus
}
#endif

#endif // __TD_CONFIG_H
/** @endcond */
