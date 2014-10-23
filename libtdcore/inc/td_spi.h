/***************************************************************************//**
 * @file
 * @brief Serial Peripheral Interface (SPI) peripheral API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.0.0
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

#ifndef __TD_SPI_H
#define __TD_SPI_H

#include <em_usart.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup SPI SPI
	 * @brief Serial Peripheral Interface (SPI) peripheral API for the TDxxxx RF modules
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 *************************   DEFINES   *****************************************
	 ******************************************************************************/

	/** @addtogroup SPI_DEFINES Defines
	 * @{ */

/** SPI IDs for SPI bus locking/unlocking */
#define RF_SPI_ID 			1		///< SPI lock ID for RF chip
#define ACCELERO_SPI_ID 	2		///< SPI lock ID for accelerometer chip
#define GPS_SPI_ID 			3		///< SPI lock ID for GPS chip
#define RFIO_SPI_ID 		4		///< SPI lock ID for RF chip GPIOs

/** Maximum SPI lock ID, MUST be set to the maximum SPI ID defined */
#define	MAX_SYSTEM_SPI_ID	RFIO_SPI_ID

/** Retrieve a user-defined SPI lock ID.
 *
 * @note
 * It is possible to reserve additional SPI lock IDs by defining the
 * MAX_SPI_ID parameter in td_config.h, and retrieve the corresponding
 * lock IDs using this macro. */
#define USER_SPI_ID(x)		((x) + MAX_SYSTEM_SPI_ID)

	/** @} */

	/*******************************************************************************
	 *************************   TYPEDEFS   *****************************************
	 ******************************************************************************/

	/** @addtogroup SPI_TYPEDEFS Typedefs
	* @{ */

	/** User SPI callback function that will be called when The SPI bus
	 * is unlocked. */
	typedef void (*TD_SPI_LockedCallback)(void);

	/** SPI USER */
	typedef struct {
		uint8_t bus;						///< The used SPI bus
		uint8_t friend_id;					///< We can gracefully take ownership on this other id
		uint32_t freq;						///< The SPI bus frequency
		USART_ClockMode_TypeDef	mode;		///< The SPI bus clock mode
		USART_TypeDef *usart;        		///< The USART base pointer
	} TD_SPI_Conf_t;

	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup SPI_GLOBAL_FUNCTIONS Global Functions
	 * @{ */

	void TD_SPI_WriteBuffer(uint8_t id, uint8_t count, uint8_t *buffer);
	void TD_SPI_WriteBuffer_PN9(uint8_t id, uint8_t count, uint8_t *buffer, bool reset);
	void TD_SPI_ReadBuffer(uint8_t id, uint8_t count, uint8_t *buffer);
	void TD_SPI_WriteByte(uint8_t id, uint8_t c);
	uint16_t TD_SPI_BackToBack(uint8_t id, uint8_t *write, uint8_t *read, uint16_t count);
	void TD_SPI_StartBackToBack(uint8_t id);
	void TD_SPI_EndBackToBack(uint8_t id);
	uint32_t TD_SPI_WriteReadDouble(uint8_t id, uint32_t value);
	void TD_SPI_WriteDouble(uint8_t id, uint32_t value);
	bool TD_SPI_Lock(uint8_t id, TD_SPI_LockedCallback Callback);
	void TD_SPI_UnLock(uint8_t id);
	void TD_SPI_Register(uint8_t id, uint8_t friend_id, uint8_t bus, uint32_t freq, USART_ClockMode_TypeDef mode);
	void TD_SPI_InitBus(uint8_t bus);
	void TD_SPI_LockDump(void);

	/** @} */

	/** @} (end addtogroup SPI) */

#ifdef __cplusplus
}
#endif

#endif // __TD_SPI_H
