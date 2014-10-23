/***************************************************************************//**
 * @file
 * @brief General Purpose IO (GPIO) peripheral API for the TDxxxx RF modules.
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

#ifndef __TD_GPIO_H
#define __TD_GPIO_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup GPIO
	 * @brief General Purpose IO (GPIO) peripheral API for the TDxxxx RF modules
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 *************************   DEFINES   *****************************************
	 ******************************************************************************/

	/** @addtogroup GPIO_DEFINES Defines
	 * @{ */

#define TD_GPIO_USER		0								///< User GPIO IRQ hook
#define TD_GPIO_SYSTEM		1								///< System GPIO IRQ hook
#define TD_GPIO_EVEN		0								///< Even GPIO IRQ hook
#define TD_GPIO_ODD			2								///< Odd GPIO IRQ hook
#define TD_GPIO_SYSTEM_ODD	(TD_GPIO_SYSTEM | TD_GPIO_ODD)	///< Odd System GPIO IRQ hook
#define TD_GPIO_SYSTEM_EVEN	(TD_GPIO_SYSTEM | TD_GPIO_EVEN)	///< Even System GPIO IRQ hook
#define TD_GPIO_USER_ODD	(TD_GPIO_USER | TD_GPIO_ODD)	///< Odd User GPIO IRQ hook
#define TD_GPIO_USER_EVEN	(TD_GPIO_USER | TD_GPIO_EVEN)	///< Even User GPIO IRQ hook
#define TD_GPIO_MAX_HOOKS	4								///< Maximum number of GPIO IRQ hooks
#define TD_GPIO_ODD_MASK	0xAAAAAAAA						///< Mask for odd interrupts
#define TD_GPIO_EVEN_MASK	0x55555555						///< Mask for even interrupts

	/** @} */

	/*******************************************************************************
	 *************************   TYPEDEFS   ****************************************
	 ******************************************************************************/

	/** @addtogroup GPIO_TYPEDEFS Typedefs
	 * @{ */

	/** Interrupt process callback function. mask contains bitfield of one or multiple I/O that triggered IRQ */
	typedef void (*TD_GPIO_callback_t)(uint32_t mask);

	/** GPIO IRQ hook structure */
	typedef struct {
		TD_GPIO_callback_t callback;	///< callback function to handle this GPIO IRQ
		uint32_t mask;					///< mask to apply for matching this GPIO IRQ
	} TD_GPIO_hook_t;

	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup GPIO_USER_FUNCTIONS User Functions
	 * @{ */

	/***************************************************************************//**
	 * @brief
	 *   Default GPIO IRQ processing function.
	 ******************************************************************************/
	static __INLINE void TD_GPIO_DefaultHook(void)
	{
	}

	/***************************************************************************//**
	 * @brief
	 *   Register a callback function for processing interrupts matching a given bit mask.
	 *
	 * @ param[in] type
	 *   The type of callback to set up.
	 *
	 * @param[in] callback
	 *   Pointer to callback function called when an interrupt matching the mask is
	 *   received.
	 *
	 * @param[in] mask
	 *   Mask for testing a received even interrupts against.
	 ******************************************************************************/
	static __INLINE void TD_GPIO_SetCallback(int type, TD_GPIO_callback_t callback, uint32_t mask)
	{
		extern TD_GPIO_hook_t TD_GPIO_Hooks[];

		TD_GPIO_Hooks[type].callback = callback;
		TD_GPIO_Hooks[type].mask = mask;
	}

	/** @addtogroup GPIO_PROTOTYPES Prototypes
	 * @{ */

	void TD_GPIO_Init(void);
	void TD_GPIO_Dump(void);

	/** @} */
	/** @} */

	/*******************************************************************************
	 **************************   PUBLIC VARIABLES   *******************************
	 ******************************************************************************/

	/** @addtogroup GPIO_GLOBAL_VARIABLES Global Variables
	 * @{ */
	/** @addtogroup GPIO_EXTERN External Declarations
	 * @{ */

	/** Array of GPIO IRQ hooks for system/user/odd/even IRQs */
	extern TD_GPIO_hook_t TD_GPIO_Hooks[];

	/** @} */
	/** @} */

	/** @} (end addtogroup GPIO) */

#ifdef __cplusplus
}
#endif

#endif // __TD_GPIO_H
