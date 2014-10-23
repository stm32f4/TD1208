/***************************************************************************//**
 * @file
 * @brief UART/RF bootloader API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 2.1.0
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

#ifndef __TD_BOOT_H
#define __TD_BOOT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup BOOT Bootloader
	 * @brief RF bootloader API for the TDxxxx RF modules
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 ***********************   ENUMERATIONS   **************************************
	 ******************************************************************************/

	/** @addtogroup BOOT_ENUMERATIONS Enumerations
	 * @{ */

	/** Bootloader operation modes */
	typedef enum {
		MODE_BOOT,					/**< Used in bootloader mode */
		MODE_BOOT_EXIT,				/**< Used at exit bootloader mode */
		MODE_POWER_ON,				/**< Power on TCXO and RF chip */
		MODE_FULL,					/**< Full reconfigure mode */
		MODE_FULL_APP,				/**< Full reconfigure in app mode (no ramfunc use) */
		MODE_GPIO					/**< Reconfigure only GPIO */
	}
	TD_BOOT_radio_mode_t;

	/** @} */

	/*******************************************************************************
	 *************************   TYPEDEFS   ****************************************
	 ******************************************************************************/

	/** @addtogroup BOOT_USER_TYPEDEFS Typedefs
	 * @{ */

	/** Bootloader handling function pointer */
	typedef void (*TD_BOOT_Handler_t)(bool LedPolarity, uint8_t ProductType);

	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup BOOT_USER_FUNCTIONS User Functions
	 * @{ */

	void TD_BOOT_Init(bool LedPolarity, uint8_t ProductType);
	void TD_BOOT_ConfigureRadio(TD_BOOT_radio_mode_t Mode);
	void TD_BOOT_Exec_InitData(unsigned short const *data, unsigned char len);

	/** @} */

	/** @} (end addtogroup BOOT) */

#ifdef __cplusplus
}
#endif

#endif // __TD_BOOT_H
