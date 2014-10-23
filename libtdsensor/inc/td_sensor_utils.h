/***************************************************************************//**
 * @file
 * @brief Sensor Utils
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

#ifndef __TD_SENSOR_UTILS_H
#define __TD_SENSOR_UTILS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup TD_SENSOR_UTILS Sensor Utils
	 * @{ */

	/*******************************************************************************
	 *************************   DEFINES   **************************************
	 ******************************************************************************/

	/** @addtogroup TD_SENSOR_UTILS_DEFINES Defines
	 * @{ */

	/* Bits to byte conversion macro */
#define BITS_TO_BYTES(b) ((b + 7) / 8)

	/** Shift a value so that all their useful bits (n) are on left
	 * 00101011 -> 6 bits saved in 8 bits
	 * converted to:
	 * 10101100 -> all useful bits on left
	 */
#define BITS_TO_LEFT(v, n) (v << ((sizeof (v) * 8) - n))

	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup TD_SENSOR_UTILS_USER_FUNCTIONS User Functions
	 * @{ */

	void TD_SENSOR_UTILS_BitConcat(uint8_t *data,  uint8_t *len, uint8_t *data_append, uint8_t len_append);

	/** @} */

	/** @} (end addtogroup TD_SENSOR_UTILS) */

#ifdef __cplusplus
}
#endif

#endif /* TD_SENSOR_UTILS_H_ */
