/***************************************************************************//**
 * @file
 * @brief Flash controller (MSC) peripheral API for the TDxxx RF modules.
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

#ifndef __TD_FLASH_H
#define __TD_FLASH_H

#include <stdint.h>
#include <stdbool.h>

#include "td_module.h"

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup FLASH
	 * @brief Flash controller (MSC) Peripheral API for the TDxxxx RF modules
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 *******************************   DEFINES   ***********************************
	 ******************************************************************************/

	/** @addtogroup FLASH_DEFINES Defines
	 * @{ */

	/** Number of words in a flash page */
#define FLASH_E2P_SIZE_WORD        	(FLASH_PAGE_SIZE / sizeof (uint32_t))

	/** Deprecated names, please use TD_FLASH_ReadVariable() instead, compatibility only */
#define TD_FLASH_ReadDataBuffer TD_FLASH_ReadVariable

	/** Deprecated names, please use TD_FLASH_DeclareVariable() instead, compatibility only */
#define TD_FLASH_DeclareFlashVariable TD_FLASH_DeclareVariable

	/** Deprecated names, please use TD_FLASH_WriteVariables() instead, compatibility only */
#define TD_FLASH_UpdateFlashVariables TD_FLASH_WriteVariables

	/** Deprecated names, please use TD_FLASH_DeleteVariables() instead, compatibility only */
#define TD_FLASH_DeleteFlashVariables TD_FLASH_DeleteVariables

	/** @} */

	/*******************************************************************************
	 *************************   TYPEDEFS   ****************************************
	 ******************************************************************************/

	/** @addtogroup FLASH_TYPEDEFS Typedefs
	 * @{ */

	/** TDxxxx RF module device descriptor */
	typedef struct _TD_DEVICE {
		uint32_t    Serial;            	/**< Descriptor version 0 or 1 serial number (SIGFOX ID). */
		uint32_t    Key1;				/**< Reserved. */
		uint64_t    Key2;				/**< Reserved. */
		uint8_t     ModResult;			/**< Descriptor version 0 or 1 module manufacturing test result. */
		uint8_t     ProdResult;			/**< Descriptor version 0 or 1 final product manufacturing test result. */
		uint8_t     LedMask1;			/**< Obsolete. */
		uint8_t     LedMask2;			/**< Obsolete. */
	} TD_DEVICE;

	/** TDxxxx RF module extended device descriptor */
	typedef struct _TD_DEVICE_EXT {
		uint16_t	DeviceVersion;		/**< Device descriptor version */
		uint8_t		TDSerial[12];		/**< Descriptor version 1 or 2 Serial number (Standard TD ID). */
		uint8_t		PK[16];				/**< Descriptor version 2 SIGFOX 128-bit private key */
		uint32_t    Serial;            	/**< Descriptor version 2 serial number (SIGFOX ID). */
		uint8_t     ModResult;			/**< Descriptor version 2 module manufacturing test result. */
		uint8_t     ProdResult;			/**< Descriptor version 2 final product manufacturing test result. */
	} TD_DEVICE_EXT;

	/** Flash Logger function typedef to permit "dynamic linking" */
	typedef void (*TD_FLASH_InitLogger_t)(bool reset,
										  uint8_t id,
										  uint8_t data_size,
										  uint32_t first_page_adress,
										  uint32_t last_page_adress);

	/**
	 * Flash variable structure
	 *
	 * @note
	 *  Each variable is assigned a unique virtual address automatically when first
	 *  written to, or when using the declare function.
	 */
	typedef struct {
		uint8_t *data_pointer;			/**< Variable data pointer. */
		uint16_t data_size;				/**< Variable data size. */
	} TD_FLASH_variable_t;

	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup FLASH_USER_FUNCTIONS User Functions
	 * @{ */

	void TD_FLASH_Write(void *buffer, uint32_t count);
	bool TD_FLASH_Read(void *buffer, uint32_t count);
	bool TD_FLASH_DeviceRead(TD_DEVICE *device);
	bool TD_FLASH_DeviceReadExtended(TD_DEVICE *device, TD_DEVICE_EXT *device_ext);
	uint16_t TD_FLASH_ReadVariable(uint8_t index, uint8_t *buffer);
	bool TD_FLASH_DeclareVariable(uint8_t *variable, uint16_t size, uint8_t *index);
	void TD_FLASH_WriteVariables(void);
	void TD_FLASH_DeleteVariables(void);
	void TD_FLASH_SetVariablesVersion(uint32_t version);
	void TD_FLASH_DumpVariables(uint8_t *data);
	__RAMFUNCTION void TD_FLASH_ErasePage(uint32_t *blockStart);
	__RAMFUNCTION void TD_FLASH_WriteWord(uint32_t *address, uint32_t data);
	void TD_FLASH_Init(void);
	void TD_FLASH_Deinit(void);
	void TD_FLASH_InitLogger(bool reset,
							 uint8_t id,
							 uint8_t data_size,
							 uint32_t first_page_adress,
							 uint32_t last_page_adress);
	void TD_FLASH_LoggerWrite(uint8_t id, uint32_t *data);
	void TD_FLASH_LoggerResetRead(uint8_t id);
	bool TD_FLASH_LoggerReadNext(uint8_t id, uint32_t *data);

	/** @} */

	/** @} (end addtogroup FLASH) */

#ifdef __cplusplus
}
#endif

#endif // __TD_FLASH_H
