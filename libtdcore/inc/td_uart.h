/***************************************************************************//**
 * @file
 * @brief Un Asynchronous Receiver/Transmitter (UART) peripheral API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 2.0.2
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

#ifndef __TD_UART_H
#define __TD_UART_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup UART
	 * @brief Asynchronous Receiver/Transmitter (UART) peripheral API for the TDxxxx RF modules
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 *************************   TYPEDEFS   ****************************************
	 ******************************************************************************/

	/** @addtogroup UART_TYPEDEFS Typedefs
	 * @{ */

	/** UARt receive interrupt callback function pointer */
	typedef void (*TD_UART_CALLBACK)(char);

	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup UART_USER_FUNCTIONS User Functions
	 * @{ */

	/***************************************************************************//**
	 * @brief
	 *   Set up a callback function when receiving a character on UART.
	 *
	 * @param[in] cb
	 *   Pointer to the UART receive character callback function.
	 ******************************************************************************/
	static __INLINE void TD_UART_SetRxCallback(TD_UART_CALLBACK cb)
	{
		extern volatile TD_UART_CALLBACK TD_UART_RxCallback;
		TD_UART_RxCallback = cb;
	}

	void *TD_UART_Init(uint32_t speed, bool rxEnable, bool shared);
	void TD_UART_Putc(void *p, char c);
	void TD_UART_Start(void *p);
	void TD_UART_Stop(void *p);
	int TD_UART_GetChar(void);
	int TD_UART_AvailableChars(void);
	void TD_UART_Flush(void);
	void TD_UART_SendString(char *string);
	void TD_UART_Send(char *buffer, char length);

	/** @} */

	/** @} (end addtogroup UART) */

#ifdef __cplusplus
}
#endif

#endif // __TD_UART_H
