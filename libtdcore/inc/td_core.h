/***************************************************************************//**
 * @file
 * @brief Public header for the TDxxxx RF modules.
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

#ifndef __TD_CORE_H
#define __TD_CORE_H

#include "td_module.h"

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup COMPILATION_OTPIONS Compilation Options
	 * @brief Public compilation options for the TDxxxx RF modules
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 *************************   DEFINES   *****************************************
	 ******************************************************************************/

	/** libtdcore v4.0.0 version number */
#define LIBTDCORE_V4_0_0			((4 * 0x10000) + (0 * 0x100) + 0)

	/** libtdcore current version number */
#define LIBTDCORE_VERSION			LIBTDCORE_V4_0_0

	/** libtdcore current version number string */
#define LIBTDCORE_VERSION_STRING	"v4.0.0"

	/** Flag to include printf() support */
#define USE_PRINTF

	/** Flag to include UART support */
#define USE_UART

	/** Flag to ensure we are not in interrupt handler */
#define TD_ASSERT_NOT_IN_INTERRUPT	EFM_ASSERT(!(SCB->ICSR&0xFF))

	/** @} (end addtogroup COMPILATION_OTPIONS) */

#ifdef __cplusplus
}
#endif

#endif // __TD_CORE_H

