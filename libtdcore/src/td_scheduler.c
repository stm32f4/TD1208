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

#include <time.h>
#include <stdint.h>
#include <stdbool.h>

#include "td_rtc.h"
#include "td_core.h"
#include "td_printf.h"
#include "td_scheduler.h"
#include "td_trap.h"
#include "td_utils.h"

/***************************************************************************//**
 * @addtogroup SCHEDULER
 * @brief Scheduler API for the TDxxxx RF modules
 * @{
 ******************************************************************************/
// TODO : add possibility to schedule a timer at delay|interval|interval|interval ...

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup SCHEDULER_DEFINES Defines
 * @{ */

//#define SCHEDULER_DEBUG
#ifdef SCHEDULER_DEBUG

/** Scheduler manager caller source argument */
#define TD_SCHEDULER_MANAGER_SOURCE			uint32_t src

/** Scheduler manager trace line argument */
#define TD_SCHEDULER_TR						__LINE__
#else

/** Scheduler manager caller source argument */
#define TD_SCHEDULER_MANAGER_SOURCE			void

/** Scheduler manager trace line argument */
#define TD_SCHEDULER_TR
#endif

/** Pending state for single-shot timers */
#define SCHEDULER_RESERVED_STATE			2

/** Null scheduler index */
#define SCHEDULER_NO_INDEX					0xFF

/** Flag to automatically remove a timer at end of processing */
#define SCHEDULER_FLAG_REMOVE_TIMER_AT_END	0x1

/** Flag to invalidate a timer */
#define SCHEDULER_FLAG_INVALIDATED_ITEM		0x2

/** @} */

/*******************************************************************************
 ***********************   ENUMERATIONS   **************************************
 ******************************************************************************/

/** @addtogroup SCHEDULER_ENUMERATIONS Enumerations
 * @{ */

/** Scheduler states */
typedef enum {
	SCHEDULER_IDLE = 0,
	SCHEDULER_IN_PROGRESS,
	SCHEDULER_RELAUNCH
} TD_SCHEDULER_state_t;

/** @} */

/*******************************************************************************
 *************************   PUBLIC VARIABLES   ********************************
 ******************************************************************************/

/** @addtogroup SCHEDULER_GLOBAL_VARIABLES Global Variables
 * @{ */

/** Array of timers */
extern TD_SCHEDULER_timer_t *TD_SCHEDULER_Timer;

/** Queue of callback waiting execution for non-IRQ timers */
extern TD_SCHEDULER_callback_t *TD_SCHEDULER_CallbackQueue;

#ifdef SCHEDULER_DEBUG
int32_t	TD_SCHEDULER_LastIRQ = 0;
uint32_t TD_SHEDULER_LastSet = 0;
#endif

/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup SCHEDULER_LOCAL_VARIABLES Local Variables
 * @{ */

/** Count of active timers */
static uint8_t TD_SCHEDULER_TimerCount = 0;

/** Active timer index */
static uint8_t TD_SCHEDULER_ActiveTimer = SCHEDULER_NO_INDEX;

/** Flag to reject undesired IRQ as comp1 always have a value */
static bool TD_SCHEDULER_WaitIrq = false;

/** Callback count in queue */
static uint8_t TD_SCHEDULER_CallbackQueueCount = 0;

/** Current callback index in queue */
static uint8_t TD_SCHEDULER_CallbackQueueIndex = 0;

/** Reference for elapsed time */
static uint64_t LastTime = 0;

/** @} */

/*******************************************************************************
 *************************   PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SCHEDULER_LOCAL_FUNCTIONS Local Functions
 * @{ */

static void TD_SCHEDULER_Manager(TD_SCHEDULER_MANAGER_SOURCE);

/***************************************************************************//**
 * @brief
 *   Remove a timer from the scheduler.
 *
 * @param[in] id
 *  Timer id.
 ******************************************************************************/
static void TD_SCHEDULER_RemovePrivate(uint8_t id, bool purge)
{
	int i;
	uint32_t msk;
	bool dirty = false;

	if (id < CONFIG_TD_SCHEDULER_MAX_TIMER) {

		// Atomic operation
		msk = __get_PRIMASK();
		__set_PRIMASK(1);

		// If someone else removed timer during this time, just exit
		// Note : if there is no purge, ignore this test (remove in execute)
		// Possible case:
		//  repetition != 0 => timer is running
		//  repetition = 0 && TD_SCHEDULER_Timer[id].irq == SCHEDULER_RESERVED_STATE => timer is just added or timer is mark to be removed
		if (purge && !TD_SCHEDULER_Timer[id].repetition && TD_SCHEDULER_Timer[id].irq != SCHEDULER_RESERVED_STATE) {
			__set_PRIMASK(msk);
			return;
		}

		// If we are waiting it
		if (id == TD_SCHEDULER_ActiveTimer) {
			TD_SCHEDULER_ActiveTimer = SCHEDULER_NO_INDEX;

			// We must reschedule
			dirty = true;
		}

		// Remove timer
		TD_SCHEDULER_Timer[id].repetition = 0;
		TD_SCHEDULER_Timer[id].irq = 0;

		// Decrement count, should always be > 0 but just in case...
		if (TD_SCHEDULER_TimerCount > 0) {
			TD_SCHEDULER_TimerCount--;

			// We are removing the last timer
			if (TD_SCHEDULER_TimerCount == 0) {
				TD_RTC_SetUserHandler(0);

				// No need to reschedule, we stop everything
				dirty = false;
			}
		}
		__set_PRIMASK(msk);
		if (purge) {

			// Remove pending callback in queue for a given removed timer
			uint8_t index = TD_SCHEDULER_CallbackQueueIndex;
			for (i = 0; i < TD_SCHEDULER_CallbackQueueCount; i++) {

				// Check for a match
				if (TD_SCHEDULER_CallbackQueue[index].index == id) {

					// Atomic operation
					msk = __get_PRIMASK();
					__set_PRIMASK(1);

					// We must be sure that memcpy is a direct copy (not forward) to use this
					// As long as td_utils.c version is used, there is no problem
					//memcpy(&TD_SCHEDULER_CallbackQueue[TD_SCHEDULER_CallbackQueueIndex + i], &TD_SCHEDULER_CallbackQueue[TD_SCHEDULER_CallbackQueueIndex + i + 1],
					//	   (TD_SCHEDULER_CallbackQueueCount - 1 - i) * sizeof (TD_SCHEDULER_callback_t));

					//TD_SCHEDULER_CallbackQueueCount--;

					// Flag this entry as invalid
					TD_SCHEDULER_CallbackQueue[index].flags |= SCHEDULER_FLAG_INVALIDATED_ITEM;
					__set_PRIMASK(msk);
				}
				index++;
				if (index >= CONFIG_TD_SCHEDULER_MAX_QUEUE){
					index = 0;
				}
			}
		}
		if (dirty) {
			TD_SCHEDULER_Manager(TD_SCHEDULER_TR);
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Execute the callback associated with a timer.
 *
 * @param[in] id
 *  Timer id.
 ******************************************************************************/
static void TD_SCHEDULER_ExecuteTimer(uint8_t id)
{
	void (*callback)(uint32_t, uint8_t);
	uint32_t arg, msk;
	uint8_t repetition, index;
	uint64_t time;

	// Grab current time, as soon as possible
	time = TD_SCHEDULER_GetTime();

	// Now go atomic
	msk = __get_PRIMASK();
	__set_PRIMASK(1);

	// Grab data from timer
	repetition = TD_SCHEDULER_Timer[id].repetition;

	// Someone else already execute this timer... (reentrency) => exit
	if (!repetition) {
		__set_PRIMASK(msk);
		return;
	}
	callback = TD_SCHEDULER_Timer[id].callback;
	arg = TD_SCHEDULER_Timer[id].arg;

	// Set data
	TD_SCHEDULER_Timer[id].last_time = time;

	// If called from outside of an IRQ, queue callback
	if (!TD_SCHEDULER_Timer[id].irq) {
		index = TD_SCHEDULER_CallbackQueueIndex + TD_SCHEDULER_CallbackQueueCount;
		if (index >= CONFIG_TD_SCHEDULER_MAX_QUEUE) {
			index -= CONFIG_TD_SCHEDULER_MAX_QUEUE;
		}
		if (TD_SCHEDULER_CallbackQueueCount < CONFIG_TD_SCHEDULER_MAX_QUEUE) {
			TD_SCHEDULER_CallbackQueueCount++;
			TD_WakeMainLoop();
		} else {
			TD_Trap(TRAP_SCHEDULER_QUEUE_OVF, (uint32_t) callback);
			__set_PRIMASK(msk);
			return;
		}
		TD_SCHEDULER_CallbackQueue[index].index = id;
		TD_SCHEDULER_CallbackQueue[index].flags = 0;
		if (TD_SCHEDULER_Timer[id].repetition == 1) {

			// We should:
			// - put this timer in a pending state (not used, but not allocatable too)
			// - remove it when we will process callback
			TD_SCHEDULER_CallbackQueue[index].flags = SCHEDULER_FLAG_REMOVE_TIMER_AT_END;
			TD_SCHEDULER_Timer[id].repetition = 0;
			TD_SCHEDULER_Timer[id].irq = SCHEDULER_RESERVED_STATE;
		} else {
			if (TD_SCHEDULER_Timer[id].repetition != 0xFF) {
				TD_SCHEDULER_Timer[id].repetition--;
			}
		}
		__set_PRIMASK(msk);

		// Now that the index is reserved for us, and source data are grabbed
		// Note: in case of a queue overrun, we will override the oldest event.
		// this event will be at each turn overrun
		TD_SCHEDULER_CallbackQueue[index].arg = arg;
		TD_SCHEDULER_CallbackQueue[index].callback = callback;
		TD_SCHEDULER_CallbackQueue[index].repetition = repetition;
		TD_SCHEDULER_Manager(TD_SCHEDULER_TR);
	} else {
		if (repetition != 0xFF) {
			TD_SCHEDULER_Timer[id].repetition--;
			if (TD_SCHEDULER_Timer[id].repetition == 0) {
				/* We must set RESERVED state to not have a false invalid timer trap */
				TD_SCHEDULER_Timer[id].irq = SCHEDULER_RESERVED_STATE;
				TD_SCHEDULER_RemovePrivate(id, false);
			}
		}
		__set_PRIMASK(msk);
		if (callback) {
			callback(arg, repetition - 1);
		}
		TD_WakeMainLoop();
	}
}

/***************************************************************************//**
 * @brief
 *   Arm the next timer IRQ according to all required timers.
 ******************************************************************************/
static void TD_SCHEDULER_Manager(TD_SCHEDULER_MANAGER_SOURCE)
{
	int i, j, tim_count;
	uint64_t delta, temp_delta, diff;
	volatile uint64_t time_now;
	bool set_alarm;
	bool bias;
	uint32_t msk;
	static TD_SCHEDULER_state_t in_progress = SCHEDULER_IDLE;

	// Atomic operation. If we 'reenter' remember dirty state at end
	msk = __get_PRIMASK();
	__set_PRIMASK(1);
	if ((in_progress == SCHEDULER_IN_PROGRESS) || (in_progress == SCHEDULER_RELAUNCH)) {
		in_progress = SCHEDULER_RELAUNCH;
		__set_PRIMASK(msk);
		return;
	}
	in_progress = SCHEDULER_IN_PROGRESS;

	// Stop IRQ processing
	TD_SCHEDULER_WaitIrq = false;
	__set_PRIMASK(msk);

	// Now we are in atomic condition
	while (1) {

		// Should stop processing timers right now...
		time_now = TD_SCHEDULER_GetTime();
		tim_count = TD_SCHEDULER_TimerCount;
		delta = 0xFFFFFFFFFFFFFFFF;
		set_alarm = false;
		j = 0;
		for (i = 0; i < CONFIG_TD_SCHEDULER_MAX_TIMER; i++) {

			// We should re-launch as soon as possible
			if (in_progress == SCHEDULER_RELAUNCH) {
				break;
			}
			temp_delta = 0xFFFFFFFFFFFFFFFF;

			// Stop when all active timers have been processed
			if (j >= tim_count) {
				break;
			}
			if (TD_SCHEDULER_Timer[i].repetition == 0) {
				continue;
			}

			// Count valid timer
			j++;
			bias = false;

			// In case of a bias
			if (TD_SCHEDULER_Timer[i].last_time > time_now) {
				diff = TD_SCHEDULER_Timer[i].last_time - time_now
					   + TD_SCHEDULER_Timer[i].interval;
				bias = true;
			} else {
				diff = time_now - TD_SCHEDULER_Timer[i].last_time;
			}

			// If we are late, go for it right now
			if (diff >= TD_SCHEDULER_Timer[i].interval && !bias) {

				// Does not keep order, latest should be started first!
				TD_SCHEDULER_ExecuteTimer(i);

				// After execution, we could have deleted timer, added timer, etc.
				if (in_progress == SCHEDULER_RELAUNCH) {
					break;
				}

				// Recompute new delta
				diff = time_now - TD_SCHEDULER_Timer[i].last_time;
				bias = false;
			}

			// Otherwise compute delta
			if (!bias) {
				temp_delta = TD_SCHEDULER_Timer[i].interval - diff;
			} else {
				temp_delta = diff;
			}

			// Wait until next overflow
			if (temp_delta >= 0x1000000) {
				continue;
			}

			// Get smallest delta
			if (temp_delta < delta) {
				delta = temp_delta;
				TD_SCHEDULER_ActiveTimer = i;
				set_alarm = true;
			}
		}

		// Now we know what to do. If no reschedule asked
		if (in_progress == SCHEDULER_IN_PROGRESS) {

			// Set alarm
			if (set_alarm) {

				// Must be atomic, we can start an old timer
				msk = __get_PRIMASK();
				__set_PRIMASK(1);
				TD_SCHEDULER_WaitIrq = true;
				TD_RTC_UserAlarmAfter(delta);
#ifdef SCHEDULER_DEBUG
				TD_SHEDULER_LastSet = RTC->CNT;
#endif
				__set_PRIMASK(msk);
			}
		}

		// Atomic operation. If we 'reenter' remember dirty state at end
		msk = __get_PRIMASK();
		__set_PRIMASK(1);
		if (in_progress == SCHEDULER_RELAUNCH) {
			in_progress = SCHEDULER_IN_PROGRESS;
			__set_PRIMASK(msk);
		} else {
			in_progress = SCHEDULER_IDLE;
			__set_PRIMASK(msk);
			break;
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Timer IRQ handler. Add the corresponding callback and argument in the queue
 *   so that it can be safely processed.
 ******************************************************************************/
static void TD_SCHEDULER_TimerIRQHandler(void)
{
#ifdef SCHEDULER_DEBUG
	TD_SCHEDULER_LastIRQ = RTC->CNT;
#endif

	// WaitIrq is used to throw away unwanted IRQs (during reschedule)
	if (TD_SCHEDULER_WaitIrq) {
		if (TD_SCHEDULER_ActiveTimer != 0xFF) {
			TD_SCHEDULER_WaitIrq = false;
			TD_SCHEDULER_ExecuteTimer(TD_SCHEDULER_ActiveTimer);
			TD_SCHEDULER_Manager(TD_SCHEDULER_TR);
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Append a timer to schedule. See TD_SchedulerAppend
 ******************************************************************************/
uint8_t TD_SCHEDULER_AppendPrivate(uint32_t interval, uint16_t tick, uint32_t delay, uint8_t repetition, bool irq, void (*callback)(uint32_t, uint8_t), uint32_t arg)
{
	uint8_t index;
	uint32_t msk;

	// Look for an available slot if at least one delay is specified
	if (repetition > 0 && TD_SCHEDULER_TimerCount < CONFIG_TD_SCHEDULER_MAX_TIMER
			&& (interval != 0 || tick != 0 || (delay != 0 && repetition == 1))) {
		for (index = 0; index < CONFIG_TD_SCHEDULER_MAX_TIMER; index++) {

			// Try to find a candidate (not atomic operation)
			if (TD_SCHEDULER_Timer[index].repetition || (TD_SCHEDULER_Timer[index].irq == SCHEDULER_RESERVED_STATE)) {

				// Lost, check next
				continue;
			}

			// Won, now go atomic
			msk = __get_PRIMASK();
			__set_PRIMASK(1);
			if (!TD_SCHEDULER_Timer[index].repetition && (TD_SCHEDULER_Timer[index].irq != SCHEDULER_RESERVED_STATE)) {

				// We won, reserve it with special value
				TD_SCHEDULER_Timer[index].irq = SCHEDULER_RESERVED_STATE;
			} else {

				// Slot not empty, argg someone stole us!!! Check next - not atomic
				__set_PRIMASK(msk);
				continue;
			}
			__set_PRIMASK(msk);

			// Now, index is reserved for us
			TD_SCHEDULER_Timer[index].interval = (((uint64_t) interval) << 15) | ((uint64_t) tick);
			TD_SCHEDULER_Timer[index].callback = callback;
			TD_SCHEDULER_Timer[index].last_time = TD_SCHEDULER_GetTime() + (delay << 15);
			TD_SCHEDULER_Timer[index].arg = arg;

			// Must be the last one, we validate timer with this
			msk = __get_PRIMASK();
			__set_PRIMASK(1);
			TD_SCHEDULER_Timer[index].irq = irq;
			TD_SCHEDULER_Timer[index].repetition = repetition;

			// We must update timer count, atomic too...
			if (TD_SCHEDULER_TimerCount == 0) {
				TD_SCHEDULER_WaitIrq = false;
				TD_RTC_SetUserHandler(&TD_SCHEDULER_TimerIRQHandler);
				TD_RTC_EnableUserInterrupts(true);
			}
			TD_SCHEDULER_TimerCount++;
			__set_PRIMASK(msk);
			TD_SCHEDULER_Manager(TD_SCHEDULER_TR);
			return index;
		}
	}
	return 0xFF;
}

/***************************************************************************//**
 * @brief
 *   Provide a full dump of scheduler system
 ******************************************************************************/
void TD_SCHEDULER_Dump(void)
{
	uint8_t index, n = 0;
	bool first = true;
	uint32_t t, msk;
	uint64_t cur_time = TD_SCHEDULER_GetTime();
	char *irq_s;
	uint32_t comp1;
	uint32_t _if;
	uint32_t ien;

	msk = __get_PRIMASK();
	__set_PRIMASK(1);
	tfp_printf("\r\n");
	for (index = 0; index < CONFIG_TD_SCHEDULER_MAX_TIMER; index++) {
		if (!TD_SCHEDULER_Timer[index].repetition && TD_SCHEDULER_Timer[index].irq != SCHEDULER_RESERVED_STATE) {
			continue;
		}
		if (first) {
			tfp_printf("---Timer list---\r\n");
			first = false;
		}
		t = TD_SCHEDULER_Timer[index].interval >> 15;
		irq_s="   ";
		if (TD_SCHEDULER_Timer[index].irq==SCHEDULER_RESERVED_STATE){
			irq_s="RSV";
		}
		tfp_printf("id:%d per:%02d:%02d:%02d.(%04d) rep:%3d irq:%d%s",
				   index,
				   t / 3600,
				   (t / 60) % 60,
				   t % 60,
				   (uint16_t) (TD_SCHEDULER_Timer[index].interval & 0x7FFF),
				   TD_SCHEDULER_Timer[index].repetition,
				   TD_SCHEDULER_Timer[index].irq,irq_s);
		tfp_printf(" cb:0x%08X arg:0x%08X last_time:0x%08X%08X\r\n",
				   TD_SCHEDULER_Timer[index].callback,
				   TD_SCHEDULER_Timer[index].arg,
				   (uint32_t) (TD_SCHEDULER_Timer[index].last_time >> 32),
				   (uint32_t) (TD_SCHEDULER_Timer[index].last_time & 0xFFFFFFFF));
		n++;
	}
	if (TD_SCHEDULER_CallbackQueueCount) {
		tfp_printf("---CB QUEUE---\r\n");
	}
	for (index = 0; index < TD_SCHEDULER_CallbackQueueCount; index++) {
		t = index + TD_SCHEDULER_CallbackQueueIndex;
		if (t >= CONFIG_TD_SCHEDULER_MAX_QUEUE) {
			t -= CONFIG_TD_SCHEDULER_MAX_QUEUE;
		}
		tfp_printf("flags:%d%s%s id:%d cb:0x%08X arg:0x%08X repet:%d\r\n",
			TD_SCHEDULER_CallbackQueue[t].flags,
			TD_SCHEDULER_CallbackQueue[t].flags & SCHEDULER_FLAG_REMOVE_TIMER_AT_END ? "REM" : "",
			TD_SCHEDULER_CallbackQueue[t].flags & SCHEDULER_FLAG_INVALIDATED_ITEM ? "INV" : "",
			TD_SCHEDULER_CallbackQueue[t].index,
			TD_SCHEDULER_CallbackQueue[t].callback,
			TD_SCHEDULER_CallbackQueue[t].arg,
			TD_SCHEDULER_CallbackQueue[t].repetition);
	}
	comp1 = RTC->COMP1;
	_if = RTC->IF & RTC_IF_COMP1;
	ien = RTC->IEN & RTC_IEN_COMP1;
	tfp_printf("==Wait IRQ:%d, Active:%d QC:%d QI:%d\r\n",
		TD_SCHEDULER_WaitIrq,
		TD_SCHEDULER_ActiveTimer,
		TD_SCHEDULER_CallbackQueueCount,
		TD_SCHEDULER_CallbackQueueIndex);
	tfp_printf("==%d tim., cur_time:0x%08X%08X CMP1:0x%08X IF:%d IEN:%d\r\n",
		n,
		(uint32_t) (cur_time >> 32),
		(uint32_t) (cur_time & 0xFFFFFFFF), comp1, _if, ien);
#ifdef SCHEDULER_DEBUG
	tfp_printf("==IT:%08X Set:%08X\r\n", TD_SCHEDULER_LastIRQ, TD_SHEDULER_LastSet);
#endif
	__set_PRIMASK(msk);
}

/** @} */

/*******************************************************************************
 *************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SCHEDULER_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Safely process all callbacks in queue.
 ******************************************************************************/
void TD_SCHEDULER_Process(void)
{
	uint32_t  msk;
	TD_SCHEDULER_callback_t tmp;

	// While in case a timer irq happens in between
	while (TD_SCHEDULER_CallbackQueueCount > 0) {
		memcpy(&tmp, &TD_SCHEDULER_CallbackQueue[TD_SCHEDULER_CallbackQueueIndex], sizeof (TD_SCHEDULER_callback_t));

		// Atomic operation (no interrupt during queue remove)
		msk = __get_PRIMASK();
		__set_PRIMASK(1);
		TD_SCHEDULER_CallbackQueueCount--;
		TD_SCHEDULER_CallbackQueueIndex++;
		if (TD_SCHEDULER_CallbackQueueIndex >= CONFIG_TD_SCHEDULER_MAX_QUEUE) {
			TD_SCHEDULER_CallbackQueueIndex = 0;
		}
		__set_PRIMASK(msk);

		// This item was flagged as invalidated (timer was removed, so callback must not be called
		if (tmp.flags&SCHEDULER_FLAG_INVALIDATED_ITEM) {
			continue;
		}

		// This item was flagged as last event of this timer and must be removed at end
		if (tmp.flags&SCHEDULER_FLAG_REMOVE_TIMER_AT_END) {
			TD_SCHEDULER_RemovePrivate(tmp.index, false);
		}
		if (tmp.callback) {
			(*tmp.callback)(tmp.arg, tmp.repetition - 1);
		}
	}
	if (TD_RTC_IsOverflowed() == true && TD_SCHEDULER_TimerCount > 0) {
		TD_RTC_ClearOverflow();
		TD_SCHEDULER_Manager(TD_SCHEDULER_TR);
	}
}

/***************************************************************************//**
 * @brief
 *   Initialize the scheduler.
 ******************************************************************************/
void TD_SCHEDULER_Init(void)
{
	memset(TD_SCHEDULER_Timer, 0, sizeof (TD_SCHEDULER_timer_t) * CONFIG_TD_SCHEDULER_MAX_TIMER);
}

/***************************************************************************//**
 * @brief
 *   Append a timer to schedule. Call outside irq.
 *
 * @param[in] interval
 *  Interval integer part in seconds at which the callback is called.
 *
 * @param[in] tick
 * Interval fractional part in ticks (1/32768 second) at which the callback is called
 *
 * @param[in] delay
 *  Time to wait in seconds before actually scheduling the timer. This time can be 0.
 *
 * @param[in] repetition
 * Exact number of timer callback count, 0xFF for infinite. Should be 1 or more.
 *
 * @param[in] callback
 *  Function to be called.
 *
 * @param[in] arg
 *  Argument to be passed to the callback when being called.
 *  void (*callback)(uint32_t arg, uint8_t count_remain)
 *
 * @return
 * 	Returns a timer id if successful or 0xFF if no more timer can be added in
 * 	the list.
 *
 * @note Timer appended will be called for first time at delay + interval time. Next
 *  (if repetition > 1) will be called with interval delay

 ******************************************************************************/
uint8_t TD_SCHEDULER_Append(uint32_t interval, uint16_t tick, uint32_t delay, uint8_t repetition, void (*callback)(uint32_t, uint8_t), uint32_t arg)
{
	return TD_SCHEDULER_AppendPrivate(interval, tick, delay, repetition, false, callback, arg);
}

/***************************************************************************//**
 * @brief
 *   Append a timer to schedule. Call within irq.
 *
 * @note : in most case timer callback will be called in IRQ context <b><i>but</i></b> it could be called
 * outside IRQ context too. If you absolutely need IRQ context, mask IRQ in callback
 *
 * @param[in] interval
 *  Interval integer part in seconds at which the callback is called.
 *
 * @param[in] tick
 * Interval fractional part in ticks (1/32768) at which the callback is called
 *
 * @param[in] delay
 *  Time to wait before actually scheduling the timer.
 *
 * @param[in] repetition
 * Timer repetition, 0xFF for infinite repetitions.
 *
 * @param[in] callback
 *  Function to be called.
 *
 * @param[in] arg
 *  Argument to be passed to the callback when being called.
 *
 * @return
 * 	Returns a timer id if successful or 0xFF if no more timer can be added in
 * 	the list.
 ******************************************************************************/
uint8_t TD_SCHEDULER_AppendIrq(uint32_t interval, uint16_t tick, uint32_t delay, uint8_t repetition, void (*callback)(uint32_t, uint8_t), uint32_t arg)
{
	return TD_SCHEDULER_AppendPrivate(interval, tick, delay, repetition, true, callback, arg);
}

/***************************************************************************//**
 * @brief
 *   Change a timer interval on the fly.
 *
 * @param[in] id
 *  Timer id.
 *
 * @param[in] interval
 *  Interval integer part in seconds at which the callback should now be called.
 *
 * @param[in] tick
 *  Interval fractional part in ticks at which the callback should now be called.
 *
 * @param[in] delay
 *  Time to wait in seconds before actually scheduling the timer.
 ******************************************************************************/
void TD_SCHEDULER_SetInterval(uint8_t id, uint32_t interval, uint16_t tick, uint32_t delay)
{
	uint32_t msk;

	msk = __get_PRIMASK();
	__set_PRIMASK(1);
	TD_SCHEDULER_Timer[id].interval = (((uint64_t) interval) << 15) | ((uint64_t) tick);
	TD_SCHEDULER_Timer[id].last_time = TD_SCHEDULER_GetTime() + (delay << 15);
	__set_PRIMASK(msk);
	TD_SCHEDULER_Manager(TD_SCHEDULER_TR);
}

/***************************************************************************//**
 * @brief
 *   Change a timer argument on the fly.
 *
 * @param[in] id
 *  Timer id.
 *
 * @param[in] arg
 *  New timer argument to be passed to the callback.
 ******************************************************************************/
void TD_SCHEDULER_SetArg(uint8_t id, uint32_t arg)
{
	TD_SCHEDULER_Timer[id].arg = arg;
}

/***************************************************************************//**
 * @brief
 *   Change a timer repetition on the fly.
 *
 * @param[in] id
 *  Timer id.
 *
 * @return
 * 	Returns the remaining repetitions for the given Timer id.
 ******************************************************************************/
uint8_t TD_SCHEDULER_GetRepetition(uint8_t id)
{
	return TD_SCHEDULER_Timer[id].repetition;
}

/***************************************************************************//**
 * @brief
 *   Restart a timer
 *
 * @param[in] id
 *  Timer id.
 *
 ******************************************************************************/
void TD_SCHEDULER_Restart(uint8_t id)
{
	TD_SCHEDULER_Timer[id].last_time = TD_SCHEDULER_GetTime();
	TD_SCHEDULER_Manager(TD_SCHEDULER_TR);
}

/***************************************************************************//**
 * @brief
 *   Remove a timer from the scheduler.
 *
 * @param[in] id
 *  Timer id.
 ******************************************************************************/
void TD_SCHEDULER_Remove(uint8_t id)
{
	TD_SCHEDULER_RemovePrivate(id, true);
}

/***************************************************************************//**
 * @brief
 *
 * @return
 *   Returns the current time in 1/32768 s ticks.
 ******************************************************************************/
uint64_t TD_SCHEDULER_GetTime(void)
{
	uint64_t time_now;
	uint32_t time_tick, overflow_counter;
	uint32_t msk;
	uint32_t pend;
	bool retry;

	//overflow_counter = TD_RTC_GetOverflowCounter();
	//time_tick = RTC_CounterGet();

	// This function can be called inside or outside IRQ context
	// We need to get TD_RTC_GetOverflowCounter and RTC_CounterGet() synchronously
	// Note : we state that RTC_OF is activated
	retry = true;
	while (retry) {
		retry = false;
		msk = __get_PRIMASK();
		__set_PRIMASK(1);

		// At this stage, IRQ will not trigger but RTC->IFC will be set if an RTC overflow occur

		// Is there an overflow pending before?
		pend = RTC->IF & RTC_IF_OF;

		// Get overflow and counter
		overflow_counter = TD_RTC_GetOverflowCounter();
		time_tick = RTC_CounterGet();

		// If an overflow is pending "locally", apply it. We can't rely on IRQ processing, as we may
		// already be in IRQ. To not duplicate IRQ processing, we only 'spoof' increase in overflow counter
		if (RTC->IF & RTC_IF_OF) {
			overflow_counter++;
		}

		//Is there a change in overflow pending?
		if (pend != (RTC->IF & RTC_IF_OF)) {

			// We faced a change, so retry
			retry = true;
		}
		__set_PRIMASK(msk);
	}
	time_now = overflow_counter;
	time_now = time_now << 24; // << 15 + 9
	time_now |= time_tick; // + tick
	return time_now;
}

/***************************************************************************//**
 * @brief
 *
 * @return
 *   Returns the elapsed time since last call in 1/32768 s ticks.
 ******************************************************************************/
uint32_t TD_ElapsedTime(void)
{
	uint64_t time_now, last;

	time_now = TD_SCHEDULER_GetTime();
	last = time_now - LastTime;
	LastTime = time_now;
	return last;
}

/** @} */

/** @} (end addtogroup SCHEDULER) */
