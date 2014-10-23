/***************************************************************************//**
 * @file
 * @brief Utility functions for the TDxxxx RF modules.
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

#ifndef __TD_UTILS_H
#define __TD_UTILS_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup UTILS Utility Functions
	 * @brief Utility functions for the TDxxxx RF modules
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 *************************   DEFINES   **************************************
	 ******************************************************************************/

	/** @addtogroup UTILS_DEFINES Defines
	 * @{ */

	/** Macro to get a bit field */
#define GET_BITFIELD(b, f)				((b >> f ## _SHIFT) & GET_BITFIELD_MASK(f ## _WIDTH))

	/** Macro to set a bit field */
#define SET_BITFIELD(b, f, v)			b &= ~(GET_BITFIELD_MASK(f ## _WIDTH) << f ## _SHIFT); \
			b |= ((v & GET_BITFIELD_MASK(f ## _WIDTH))  << f ## _SHIFT)

	/** Macro to compute bit mask given its width */
#define GET_BITFIELD_MASK(w)			((1 << w) - 1)

#ifndef NULL

/** Define NULL if not already defined by someone else */
#define NULL	(void *) 0
#endif

	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup UTILS_USER_FUNCTIONS User Functions
	 * @{ */

	long long atoll(char *instr);
	long long atolli(char *instr, char ignore);
	int atoi(char *instr);
	char a2i(char ch, char **src, int base, int *nump);
	int a2d(char ch);
	void i2a(int num, char *bf);
	void ui2a(unsigned int num, unsigned int base, int uc, char *bf);
	void li2a(long int num, char *bf);
	void uli2a(unsigned long int num, unsigned int base, int uc, char *bf);

#ifndef __ICCARM__
	void *memset(void *s, int c, size_t n);
	void *memcpy(void *d, const void *s, size_t n);
	int memcmp(const void *s1, const void *s2, size_t n);
#endif

	int TD_STACK_Usage(void);
	void TD_IRQ_Dump(void);

	/** @} */

	/** @} (end addtogroup TD_UTILS) */

#ifdef __cplusplus
}
#endif

#endif // __TD_UTILS_H
