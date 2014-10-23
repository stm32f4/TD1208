/******************************************************************************
 * @file
 * @brief Configuration file for the TD12xx RF module firmware
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
#ifndef CONFIG_H_
#define CONFIG_H_

#include <td_module.h>

/** Manufacturer */
#define MANUFACTURER			"Telecom Design/Sopra"

#if MODULE_REVISION == REVISION_TD1202

/** Product */
#define PRODUCT					"TD1202"

#elif MODULE_REVISION == REVISION_TD1208

/** Product */
#define PRODUCT					"TD1208"

#elif MODULE_REVISION == REVISION_TD1204

/** Product */
#define PRODUCT					"TD1204"

#endif

/** Hardware revision */
#define HARDWARE_VERSION		"0F"

#if MODULE_REVISION == REVISION_TD1202

/** TD1202 Software release and version */
#define SOFTWARE_RELEASE		1290

#elif MODULE_REVISION == REVISION_TD1208

/** TD1208 Software release and version */
#define SOFTWARE_RELEASE		1291

#elif MODULE_REVISION == REVISION_TD1204

/** TD1204 Software release and version */
#define SOFTWARE_RELEASE		1292

#endif

#define CONCAT(a,b)				a ## b
#define CONCAT2(a,b)			CONCAT(a,b)
#define STRING(x)				#x
#define STRING2(x)				STRING(x)
#define SOFTWARE_VERSION		STRING2(CONCAT2(SOFT,SOFTWARE_RELEASE))

/** Release data */
#define RELEASE_DATE			"M02+2014"

/** Telecom Design 12-digit serial number */
#define SERIAL_NUMBER			"123456789012"

/** TD CORE AT parser extension */
#define AT_CORE					1

#if MODULE_REVISION == REVISION_TD1202

/** TD LAN RF AT parser extension */
#define AT_LAN_RF				0

#else

/** TD LAN RF AT parser extension */
#define AT_LAN_RF				0

#endif

/** RF chip info AT parser extension */
#define AT_RADIO_INFO			1

/** SigFox AT parser extension */
#define AT_SIGFOX				1


#if MODULE_REVISION == REVISION_TD1202

/** TD Sensor AT parser extension */
#define AT_SENSOR				0

#else

/** TD Sensor AT parser extension */
#define AT_SENSOR				0

#endif


#if MODULE_REVISION == REVISION_TD1204

/** Geoloc AT parser extension */
#define AT_GEOLOC				1
#else

/** Geoloc AT parser extension */
#define AT_GEOLOC				0

#endif


/* Flash variables version */
#define FLASH_VARIABLES_VERSION	SOFTWARE_RELEASE

#if AT_LAN_RF
#include <at_lan.h>
#else
#define AT_LAN_EXTENSION
#endif

#if AT_RADIO_INFO
#include <at_radio.h>
#else
#define AT_RADIO_EXTENSION
#endif

#if AT_SIGFOX
#include <at_sigfox.h>
#else
#define AT_SIGFOX_EXTENSION
#endif

#if AT_CORE
#include <at_core.h>
#else
#define AT_CORE_EXTENSION
#endif

#if AT_SENSOR
#include <at_sensor.h>
#include <at_sensor_lan.h>
#include <at_sensor_send.h>
#else
#define AT_SENSOR_EXTENSION
#endif

#endif // CONFIG_H_
