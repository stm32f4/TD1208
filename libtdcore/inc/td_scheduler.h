/***************************************************************************//**
 * @file
 * @brief Scheduler API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 2.1.0
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

#ifndef __TD_SCHEDULER_H_
#define __TD_SCHEDULER_H_

#include <stdbool.h>
#include <stdint.h>

#include <td_module.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup SCHEDULER
	 * @brief Scheduler for the TDxxxx RF modules
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 ***********************   ENUMERATIONS   **************************************
	 ******************************************************************************/

	/** @addtogroup SCHEDULER_ENUMERATIONS Enumerations
	 * @{ */

	/** Scheduler special repeat values */
	typedef enum  {
		TD_SCHEDULER_ONE_SHOT = 1,				///< One-shot timer
		TD_SCHEDULER_INFINITE = 0xFF				///< Infinite timer
	} TD_SCHEDULER_repeat_t;

	/** @} */

	/*******************************************************************************
	 *************************   TYPEDEFS   ****************************************
	 ******************************************************************************/

	/** @addtogroup SCHEDULER_TYPEDEFS Typedefs
	 * @{ */

#pragma pack(1)
	/** Scheduler timer structure */
	typedef struct {
		uint64_t interval;						///< Interval at which the callback is called
		uint64_t last_time;						///< Last time at which the callback was called
		void (*callback)(uint32_t, uint8_t);	///< Function to be called
		uint32_t arg;             				///< Argument to be passed to the callback when being called
		uint8_t repetition;						///< Timer repetition, 0xFF for infinite
		uint8_t irq;							///< 1, called in IRQ, 0 call in background process, 2 in adding
	} TD_SCHEDULER_timer_t;

	/** Scheduler callback structure */
	typedef struct {
		void (*callback)(uint32_t, uint8_t);	///< Function to be called
		uint32_t arg;							///< Argument to be passed to the callback when being called
		uint8_t flags;							///< Executions flags
		uint8_t index;							///< Timer ID
		uint8_t repetition;						///< Timer repetition, 0xFF for infinite
	} TD_SCHEDULER_callback_t;
#pragma pack()

	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup SCHEDULER_USER_FUNCTIONS User Functions
	 * @{ */

	void TD_SCHEDULER_Init(void);
	uint8_t TD_SCHEDULER_Append(uint32_t interval,
								uint16_t tick,
								uint32_t delay,
								uint8_t repetition,
								void (*callback)(uint32_t, uint8_t),
								uint32_t arg);
	uint8_t TD_SCHEDULER_AppendIrq(uint32_t interval,
								   uint16_t tick,
								   uint32_t delay,
								   uint8_t repetition,
								   void (*callback)(uint32_t, uint8_t),
								   uint32_t arg);
	void TD_SCHEDULER_Remove(uint8_t id);
	void TD_SCHEDULER_SetInterval(uint8_t id,
								  uint32_t interval,
								  uint16_t tick,
								  uint32_t delay);
	void TD_SCHEDULER_SetArg(uint8_t id, uint32_t arg);
	uint8_t TD_SCHEDULER_GetRepetition(uint8_t id);
	void TD_SCHEDULER_Dump(void);
	void TD_SCHEDULER_Process(void);
	uint32_t TD_ElapsedTime(void);
	uint64_t TD_SCHEDULER_GetTime(void);
	void TD_SCHEDULER_Restart(uint8_t id);

	/** @} */

	/** @} (end addtogroup SCHEDULER) */

#ifdef __cplusplus
}
#endif

#endif // __TD_SCHEDULER_H_
