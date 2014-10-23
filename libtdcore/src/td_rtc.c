/***************************************************************************//**
 * @file
 * @brief Real-Time Clock (RTC) peripheral API for the TDxxxx RF modules.
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

#include <stdint.h>
#include <stdbool.h>

#include <em_cmu.h>
#include <em_rtc.h>
#include <em_emu.h>
#include <em_int.h>
#include <em_gpio.h>
#include <td_trap.h>

#include "td_core.h"
#include "td_rtc.h"

#if defined(_EFM32_GECKO_FAMILY)

/** Macro to wait for the RTC cross clock-domain synchronization */
#define RTC_WAIT_BUSY(cmp)\
    while (RTC->SYNCBUSY) {\
      ;\
    }
#else

/** Macro to wait for the RTC cross clock-domain synchronization */
#define RTC_WAIT_BUSY(cmp)
#endif

//#define DEBUG_RTC_DELAY
//#define DEBUG_RTC_DELAY_INFO
//#define DEBUG_RTC_TRAP
//#define DEBUG_RTC_DELAY_LOG
//#define DEBUG_RTC_EXT_CALL
//#define DEBUG_RTC_DELAY_IRQ

/***************************************************************************//**
 * @addtogroup RTC
 * @brief Real-Time Clock (RTC) peripheral API for the TDxxxx RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup RTC_DEFINES Defines
 * @{ */

/* Clock Defines */

/* These variables must reflect the RTC frequency */

/** Number of ticks per second */
#define TD_RTC_TICKS_PER_SECOND 32768

/** Overflow period based on counter width and frequency */
#define TD_RTC_OVERFLOW_PERIOD ((0x00FFFFFF + 1) / TD_RTC_TICKS_PER_SECOND)

/** Overflow flag based on counter width and frequency */
#define TD_RTC_OVERFLOWED ((0x00FFFFFF + 1) % TD_RTC_TICKS_PER_SECOND)

/** Use keepalive flag */
#define TD1202_KEEPALIVE

/** @} */

/*******************************************************************************
 **************************   PUBLIC VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup RTC_GLOBAL_VARIABLES Global Variables
 * @{ */

#ifdef TD1202_KEEPALIVE
/** Keep-alive flag */
volatile bool TD_RTC_KeepAlive = false;
#endif

/** RTC counter overflow flag */
volatile bool TD_RTC_Overflow = false;

/** Abort delay flag */
volatile bool TD_RTC_AbortDelay = false;

/** Flag to ask for an additional background loop round upon IRQ wake-up */
volatile bool BackgroundRoundWanted = false;

/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup RTC_LOCAL_VARIABLES Local Variables
 * @{ */

/** RTC peripheral initialization structure */
static RTC_Init_TypeDef RTCInit = {
	.debugRun = false,
	.comp0Top = false,
	.enable = false,
};

/** The system RTC interrupt handler */
static TD_RTC_handler_t TD_RTC_SystemHandler = 0;

/** The user RTC interrupt handler */
static TD_RTC_handler_t TD_RTC_UserHandler = 0;

/** The overflow RTC interrupt handler */
static TD_RTC_handler_t TD_RTC_OverflowHandler = 0;

/** RTC overflow counter */
static uint32_t TD_RTC_OverflowCounter = 0;

static int TD_RTC_OffsetTime = 0;

#ifdef TD1202_KEEPALIVE
/** The user keep-alive interrupt handler */
TD_RTC_handler_t TD_RTC_KeepAliveHandler = 0;

#ifndef LIB_TDCORE_TINY
/** Keep-alive counter in 512 s resolution */
static uint32_t TD_RTC_KeepAliveCounter = 0;

/** Keep-alive limit in 512 s resolution */
static uint32_t TD_RTC_KeepAliveLimit = 0;
#endif

#endif

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup RTC_LOCAL_FUNCTIONS Local Functions
 * @{ */

#ifdef __ICCARM__
// #pragma optimize = speed
#else
#pragma GCC optimize ("O3")
#endif

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup RTC_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   PendSV Interrupt Handler. Clears interrupt flag.
 *   The interrupt table is located in the assembly startup file startup_efm32.s.
 *
 * @details
 *   The purpose of the PendSV interrupt is a "software service interrupt".
 *   We use it here to call SystemHandler, outside RTC handler interrupt
 *   to be able to use RTC in interrupt processing
 ******************************************************************************/
void PendSV_Handler(void)
{
	// Clear flag
	SCB->ICSR |= SCB_ICSR_PENDSVCLR_Msk;

	// Call Handler
	TD_RTC_SystemHandler();
}

/***************************************************************************//**
 * @brief
 *   SysTick Interrupt Handler. Clears interrupt flag.
 *   The interrupt table is located in the assembly startup file startup_efm32.s.
 *
 * @details
 *   The purpose of the SysTick interrupt is a "software service interrupt".
 *   We use it here to call UserHandler, outside RTC handler interrupt
 *   to be able to use RTC in interrupt processing
 ******************************************************************************/
void SysTick_Handler(void)
{
	// Clear flag
	SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk;

	// Call Handler
	TD_RTC_UserHandler();
}

/***************************************************************************//**
 * @brief
 *   RTC Interrupt Handler. Clears interrupt flag.
 *   The interrupt table is located in the assembly startup file startup_efm32.s.
 *
 * @details
 *   The purpose of the RTC interrupt is to wake the CPU from deep sleep mode EM2
 *   at given intervals. This is a way to save energy, while ensuring that the
 *   CPU often enough checks if there are any other instructions ready to executed.
 ******************************************************************************/
#ifndef LIB_TDCORE_TINY
void RTC_IRQHandler(void)
{
	if (RTC->IF & RTC_IF_COMP0) {

		// Clear interrupt source, first, to catch another interrupt ASAP if needed
		RTC_IntClear(RTC_IFC_COMP0);

		// System RTC interrupt
		if ((RTC->IEN & RTC_IF_COMP0) && (TD_RTC_SystemHandler != 0)) {

			// Arm a "PendSV" interrupt
			SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
		}
	}
	if (RTC->IF & RTC_IF_COMP1) {

		// Clear interrupt source, first, to catch another interrupt ASAP if needed
		RTC_IntClear(RTC_IFC_COMP1);

		// User RTC interrupt
		if ((RTC->IEN & RTC_IF_COMP1) && (TD_RTC_UserHandler != 0)) {

			// Arm a "SysTick" interrupt
			SCB->ICSR |= SCB_ICSR_PENDSTSET_Msk;
		}
	}
	if (RTC->IF & RTC_IF_OF) {
		TD_RTC_OverflowCounter++;

		// Overflow interrupt
		RTC_IntClear(RTC_IF_OF);

		// Call Overflow Handler and set Overflow flag
		TD_RTC_Overflow = true;
		if (TD_RTC_OverflowHandler != 0) {
			TD_RTC_OverflowHandler();
		}

#ifdef TD1202_KEEPALIVE

		// Set Keep-alive flag if keep-alive period is fine
		if ((RTC->IEN & RTC_IF_OF) && (TD_RTC_KeepAliveLimit != 0)) {
			if (++TD_RTC_KeepAliveCounter >= (TD_RTC_KeepAliveLimit >> 9)) {
				TD_RTC_KeepAliveCounter -= TD_RTC_KeepAliveLimit >> 9;
				TD_RTC_KeepAlive = true;
			}
		}
	}
#endif

}
#endif

/***************************************************************************//**
 * @brief
 *   Initialize the RTC timer.
 *
 * @param[in] function
 *   Pointer to function to handle RTC interrupts.
 ******************************************************************************/
void TD_RTC_Init(TD_RTC_handler_t function)
{
	// Input RTC init struct in initialize function
	CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_1);

	// Enable the RTC clock
	CMU_ClockEnable(cmuClock_RTC, true);

	// Set system handler
	TD_RTC_SetSystemHandler(function);

	// Use internal crystal
	RTC_Reset();
	RTC_Init(&RTCInit);

	// No start if > T8mn just Tick timer 8mn
	RTC_CompareSet(0, 0xFFFFFF);

	// No start if > T8mn just Tick timer 8mn
	RTC_CompareSet(1, 0xFFFFFF);

	// Enable RTC interrupt vector in NVIC
	NVIC_EnableIRQ(RTC_IRQn);

	// Enable the RTC timer
	RTC_Enable(true);

	// Enable RTC overflow interrupt for keep-alive

#ifdef TD1202_KEEPALIVE
	TD_RTC_KeepAlive = false;

#ifndef LIB_TDCORE_TINY
	TD_RTC_KeepAliveLimit = 0;
	TD_RTC_KeepAliveCounter = 0;
#endif

	RTC_IntClear(RTC_IF_OF);
	RTC_IntEnable(RTC_IF_OF);
#endif

}

/***************************************************************************//**
 * @brief
 *   Wait for a delay period.
 *
 * @param[in] duration
 *   The delay duration in 32768 Hz units. Max delay : 0xFFFFF0 ticks
 *   Priority of RTC interrupt should be 0
 *
 * @return
 *   Returns true if the false completed, true if it was aborted.
 ******************************************************************************/
#ifndef LIB_TDCORE_TINY
bool TD_RTC_Delay(uint32_t duration)
{
	uint32_t cmp_alarm, cmp_delay;
	uint32_t rtc_now;

#ifdef DEBUG_RTC_DELAY
	uint32_t rtc_base;
	uint32_t rtc_now1, rtc_now2, rtc_now3, rtc_now4;
#endif

	uint32_t delta_delay, delta_alarm;
	uint32_t last_delta_delay, last_delta_alarm;
	uint32_t wait_delay, end_delay;
	uint8_t it_status, extra_check;
	uint32_t msk;

#ifdef DEBUG_RTC_DELAY_LOG
	tfp_printf("d:%d\r\n", duration);
#endif

	/* Empty RTC delay must be throw away */
	if (!duration) {
		return true;
	}
	TD_RTC_AbortDelay = false;
	end_delay = false;

	// Calculate delay end value
	rtc_now = RTC_CounterGet();

#ifdef DEBUG_RTC_DELAY
	rtc_base = rtc_now;
#endif

	// We must adjust duration by 1 to obtain accurate delay, as there is sync in place
	last_delta_delay = duration - 1;
	cmp_delay = (rtc_now + last_delta_delay) & 0xFFFFFF;

	// Particular use case:
	// if another interrupt (system alarm, scheduler) happened before delay, we do a new cycle
	// where COMP0 is grabbed another time if changed

	while (!end_delay && !TD_RTC_AbortDelay) {

#ifdef DEBUG_RTC_DELAY_INFO
		//tfp_printf("lp:%06X,%06X\r\n",RTC->CNT,RTC->COMP0);
		//tfp_printf("l%d\r\n",RTC->COMP0);
		//tfp_printf(".");
#endif

		// At this point, if system handler is triggered, it has been called.
		// Prevent Exception and Interrupt from calling handler
		// -- After this point, code is time critical ! --
		// FIXME: get and set is required so that an IRQ can't change primask value
		// in between. It can be assumed that an IRQ would set primask's value back though.
		msk = __get_PRIMASK();
		__set_PRIMASK(1);

		// Now we are almost time critical. No IRQ can interrupt us, but timer continue to run
		// so it can't be stopped. Order of instructions is important!
		// First do all initializations not depending on current timer
		wait_delay = false;
		extra_check = false;
		it_status = RTC->IEN & RTC_IF_COMP0;
		cmp_alarm = RTC->COMP0;

		// Wait Sync busy, in order to not add this delay during primary update
		RTC_WAIT_BUSY(RTC_SYNCBUSY_COMP0);

		// Now we are ready to process, time critical
		// Get current timer value. Beginning with this we must not spend more than 1 timer tick (30µs or 427 '14Mhz clock ticks')
		rtc_now = RTC_CounterGet();
		delta_delay = (cmp_delay - rtc_now) & 0xFFFFFF;
		delta_alarm = (cmp_alarm - rtc_now) & 0xFFFFFF;

		/* If COMP0 has triggered, we have delta_alarm at 0xFFFFFF and alarm just triggered during WAIT_BUSY
		 * Alarm is just NOW*/
		if (RTC->IF & RTC_IF_COMP0){
			delta_alarm = 0;
		}

		// If delay is now too small or delay is gone (delta has increased)
		// let it all go alone
		if (delta_delay > last_delta_delay) {
			delta_delay = 0;
		}

		// We can't accurately program system timer for delay < 5
		if (delta_delay <= 4) {

#ifdef DEBUG_RTC_DELAY_INFO
			uint32_t cd=RTC->CNT;
#endif

			__set_PRIMASK(msk);

			// We will not have any sync update, so recover tick removed for sync ...
			delta_delay++;

#ifdef DEBUG_RTC_DELAY_INFO
			tfp_printf("AW:%d,%d,%d,%d\r\n",delta_delay,cd,RTC->CNT,RTC->COMP0);
#endif

			// Actively wait remaining steps
			while (1) {
				if (!delta_delay) {
					break;
				}
				if (rtc_now != RTC->CNT) {
					rtc_now = RTC->CNT;
					delta_delay--;
				}
			}

#ifdef DEBUG_RTC_DELAY
			if (((RTC->CNT - rtc_base) & 0xFFFFFF) < duration) {
				tfp_printf("RTC_Delay errorA : get:%d wanted:%d\r\n", ((RTC->CNT - rtc_base) & 0xFFFFFF), duration);
				TD_Trap(TRAP_RTC_DELAY, duration);
			}
#endif

			return !TD_RTC_AbortDelay;
		}

#ifdef DEBUG_RTC_DELAY_INFO
		rtc_now3 = RTC_CounterGet();
#endif

		// Delay will end first, go. Else wait for system timer
		// Note : delta_alarm can't be < 4, so alarm will not happen during update
		// If we have delta_delay == delta alarm, alarm will trigger first
		// at next step we will throw away delay
		// If we have no AlarmHandler or IRQs are disabled, don't wait for it...
		if (!TD_RTC_SystemHandler || !it_status || (delta_delay < delta_alarm)) {

#ifdef DEBUG_RTC_DELAY_INFO
			rtc_now4 = RTC_CounterGet();
#endif

			RTC_CompareSet(TD_RTC_SYSTEM, cmp_delay);

			// Activate IT (not natively wait sync)
			RTC_IntEnable(RTC_IF_COMP0);

			// Now, sync
			RTC_WAIT_BUSY(RTC_SYNCBUSY_COMP0);

			// Now we can safely clear RTC->IF flag.
			RTC_IntClear(RTC_IFC_COMP0);
			wait_delay = true;
		}

		// We are already in an interrupt. Bad.
		if (__get_IPSR()) {

			// Allow events on pending IRQ. This doesn't work and should be fixed if possible
			// Playing with IRQ priority could be a solution, but simple try failed ...

#if 0
			// Allow all IRQs (even disabled) to trigger an event
			SCB->SCR |= SCB_SCR_SEVONPEND_Msk;

			// Set event flag
			__SEV();

			// Wait for event to clear flag
			__WFE();

			// Wait for real event
			__WFE();

			// Forbid all IRQs (even disabled) to trigger an event
			SCB->SCR &= (~SCB_SCR_SEVONPEND_Msk);
#endif

#ifdef DEBUG_RTC_DELAY_IRQ
			tfp_printf("IRQ Active wait ...\r\n");
#endif

			// Fallback method to active wait ...
			while (!(RTC->IF & RTC_IF_COMP0));
		} else {

#ifdef DEBUG_RTC_DELAY_INFO
			rtc_now1 = rtc_now;
			rtc_now2 = RTC_CounterGet();
#endif

#ifdef DEBUG_RTC_TRAP
			if (((RTC->CNT - rtc_now) & 0xFFFFFF) > 4) {
				tfp_printf("****RTC_DELAY late init : %d !!\r\n", (RTC->CNT - rtc_now) & 0xFFFFFF);

#ifdef DEBUG_RTC_DELAY_INFO
				tfp_printf("%d,%d,%d,%d\r\n", rtc_now, rtc_now3, rtc_now4, rtc_now2);
#endif

			}
#endif

			if (!wait_delay && it_status && delta_alarm < 4) {
				//tfp_printf("DELTA_ALARM!\r\n");
				while (!(RTC->IF & RTC_IF_COMP0));
			} else {

				// Enter EM2. At this time RTC->CNT could be between rtc_now and rtc_now + 4
				EMU_EnterEM2(false);

#ifdef DEBUG_RTC_DELAY_INFO
				if ((RTC->CNT > cmp_alarm) && (!(RTC->IF & RTC_IF_COMP0))) {
					tfp_printf("AL %d %d %d %d \r\n", rtc_now2, RTC->CNT, cmp_alarm, delta_alarm);
					tfp_printf("AL IF:%02X IEN:%02X COMP0:0x%08X COMP1:0x%08X\r\n", RTC->IF, RTC->IEN, RTC->COMP0, RTC->COMP1);
				}
				if (((cmp_alarm-RTC->CNT) & 0xFFFFFF) > delta_alarm) {
					tfp_printf("ALARM missed ! %d %d %d\r\n", RTC->CNT, cmp_alarm, delta_alarm);
				}
				if (((cmp_delay-RTC->CNT) & 0xFFFFFF) > delta_delay){
					tfp_printf("DELAY missed ! %d %d %d\r\n", RTC->CNT, cmp_delay, delta_delay);
				}
#endif

			}
		}
		last_delta_delay = delta_delay;
		last_delta_alarm = delta_alarm;

		// Handle actual IRQ, we have a RTC IRQ, one we are waiting for
		if (RTC->IF & RTC_IF_COMP0) {

			// Clear IRQ source, first, to be able to catch another IRQ (set by SystemHandler for example)
			RTC_IntClear(RTC_IFC_COMP0);

			// We are waiting our delay, finished.
			if (wait_delay) {
				end_delay = true;
				extra_check = true;
			} else {

#ifdef DEBUG_RTC_DELAY
				tfp_printf("R+\r\n");
#endif

				// We have a system timer, callback
				if (TD_RTC_SystemHandler) {

					// Restore system compare
					if (!it_status) {

						// Disable IRQ
						RTC_IntDisable(RTC_IF_COMP0);
					} else {

						// Here we just restore system compare for not disturbing System Handler
						// We do not have to synchronize it, because:
						// - it is already past
						// - it will not trigger another time
						// - it is just in case System Handler would read back it
						RTC_CompareSet(TD_RTC_SYSTEM, cmp_alarm);
						RTC_WAIT_BUSY(RTC_SYNCBUSY_COMP0);

						// Next, call handler

#ifdef DEBUG_RTC_EXT_CALL
				tfp_printf("CSHN s:%d\r\n", it_status);
#endif

				TD_RTC_SystemHandler();

#ifdef DEBUG_RTC_EXT_CALL
				tfp_printf("CSHNE%d,%d\r\n", RTC->COMP0, RTC->IEN & RTC_IF_COMP0);
#endif

					}
				}
			}
		} else {

			// We have another IRQ. Be careful, RTC IRQ can appear at any time.
			extra_check = true;
		}

		// If we are waiting for an alarm, no problem, alarm will occur and standard handler will be called,
		// no special processing required.
		// If we are waiting for a delay and wait is finished, check if we have not lost an alarm
		// If we are waiting for a delay and other IRQs happened, we should cancel the IRQ, or system handler will be called
		if (wait_delay && extra_check) {

			// Restore system compare
			if (!it_status) {
				RTC_IntDisable(RTC_IF_COMP0);
			} else {

				// For the time, just restore system handler value
				// We will synchronize it later if needed
				RTC_CompareSet(TD_RTC_SYSTEM, cmp_alarm);
				RTC_WAIT_BUSY(RTC_SYNCBUSY_COMP0);
			}

			// Check real alarm status
			rtc_now = RTC_CounterGet();
			delta_alarm = (cmp_alarm - rtc_now) & 0xFFFFFF;

#ifdef DEBUG_RTC_DELAY
			if (delta_alarm < 4) {
				tfp_printf("Al:%d\r\n", delta_alarm);
			}
#endif

			// If alarm is late, or alarm is now (now at clock domain diff)
			if (TD_RTC_SystemHandler && it_status && ((delta_alarm > last_delta_alarm) || (delta_alarm < 4))) {

				#ifdef DEBUG_RTC_DELAY
				tfp_printf("R*%d,%d,%d,%d,%d,%d\r\n",
					delta_delay,
					last_delta_alarm,
					rtc_now1,
					rtc_now3,
					rtc_now4,
					rtc_now2);
#endif

				// Clear flag that can have been set
				RTC_IntClear(RTC_IFC_COMP0);
				RTC_WAIT_BUSY(RTC_SYNCBUSY_COMP0);

				// Call system handler

#ifdef DEBUG_RTC_EXT_CALL
				tfp_printf("CSH s:%d dal:%d ldal:%d\r\n", it_status, delta_alarm, last_delta_alarm);
#endif

				TD_RTC_SystemHandler();

				// Alarm can't cleanly start, and flag can be lost. So set a new alarm as soon as possible
				// with clean process and synchronization
				//TD_RTC_AlarmAfter(0);

				// Call handler
				//TD_RTC_SystemHandler();

				// Clear interrupt source (to not re enter alarm)
				//RTC_IntClear(RTC_IFC_COMP0);
			} else {

				// Here we must clear alarm and synchronize with new value
				RTC_IntClear(RTC_IFC_COMP0);
				RTC_WAIT_BUSY(RTC_SYNCBUSY_COMP0);
			}
		}

		// Let IRQ go, if other IRQs are pending, they will execute here
		__set_PRIMASK(msk);

		// --- end time critical ---
	}

#ifdef DEBUG_RTC_DELAY
	if (((RTC->CNT - rtc_base) & 0xFFFFFF) < duration) {
		tfp_printf("RTC_Delay errorB : get:%d wanted:%d RTC->CNT:%d\r\n",
			((RTC->CNT - rtc_base) & 0xFFFFFF),
			duration,
			RTC->CNT);
		tfp_printf("TD_RTC_AbortDelay:%d end_delay:%d \r\n", TD_RTC_AbortDelay, end_delay);
		tfp_printf("cmp_delay:%d base:%d wait_delay:%d\r\n", cmp_delay, rtc_base, wait_delay);
		tfp_printf("cmp_alarm:%d\r\n", cmp_alarm);
		TD_Trap(TRAP_RTC_DELAY, duration);
	}
#endif

#ifdef DEBUG_RTC_DELAY_LOG
	tfp_printf("do\r\n");
#endif

	return !TD_RTC_AbortDelay;
}

#endif

/***************************************************************************//**
 * @brief
 *   Program a relative system alarm.
 *
 * @param[in] delay
 *   Relative delay in ticks before the alarm will occur. It must be >= 4.
 *
 *  @note
 *   Alarm will only trigger if a handler is set by TD_RTC_SetSystemHandler().
 ******************************************************************************/
void TD_RTC_AlarmAfter(int32_t delay)
{
	uint32_t msk;
	// delay is a signed value to correctly handle 'negatives' delays
	// An access to low frequency domain need at least 4 low freq clock to ensure IT
	// Synchronize read/set -> 1 clock
	// Update write -> 1 clock
	// Note at same clock edge COMP flag will be clear to avoid previous COMP
	// Time before interrupt -> 1 clock
	// Interrupt time -> 1 clock
	if (delay < 4) {
		delay = 4;
	}

	// Forbid interrupt
	msk = __get_PRIMASK();
	__set_PRIMASK(1);

	// Wait sync to avoid sync during update
	RTC_WAIT_BUSY(RTC_SYNCBUSY_COMP0);

#ifdef DEBUG_RTC_TRAP
	uint32_t rtc_now = RTC->CNT;
#endif

	// Now we can update COMP register without messing
	RTC_CompareSet(TD_RTC_SYSTEM, (RTC_CounterGet() + delay) & 0xFFFFFF);

	// We *MUST* wait new COMP is taken in account
	RTC_WAIT_BUSY(RTC_SYNCBUSY_COMP0);

	// Now we can safely clear RTC->IF flag.
	RTC_IntClear(RTC_IFC_COMP0);

	// This flag seems to be in sync with IRQ triggers. So we do not need to busy-wait
	// for it (it does not trigger an IRQ even if we enable IRQs at next instruction)

#ifdef DEBUG_RTC_TRAP
	if (((RTC->CNT - rtc_now) & 0xFFFFFF) > 4) {
		tfp_printf("****RTC_AlarmAfter late init : %d !!\r\n", (RTC->CNT - rtc_now) & 0xFFFFFF);
	}
#endif

	__set_PRIMASK(msk);
}

/***************************************************************************//**
 * @brief
 *   Program a relative user alarm.
 *
 * @param[in] delay
 *   Relative delay in ticks before the alarm will occur. It must be >= 4.
 *
 *  @note
 *   Alarm will only trigger if a handler is set by TD_RTC_SetSystemHandler().
 *   This function must not be called when TD_Scheduler is in use.
 ******************************************************************************/
void TD_RTC_UserAlarmAfter(int32_t delay)
{
	uint32_t msk;

	// delay is a signed value to correctly handle 'negatives' delays
	// An access to low frequency domain need at least 4 low frequency clock to ensure IRQ
	// Synchronize read/set -> 1 clock
	// Update write -> 1 clock
	// Note at same clock edge COMP flag will be clear to avoid previous COMP
	// Delay before interrupt -> 1 clock
	// IRQ delay -> 1 clock
	if (delay < 4) {
		delay = 4;
	}

	// Disable IRQs
	msk = __get_PRIMASK();
	__set_PRIMASK(1);

	// Wait sync to avoid sync during update
	RTC_WAIT_BUSY(RTC_SYNCBUSY_COMP1);

	// Now we can update COMP register without messing

#ifdef DEBUG_RTC_TRAP
	uint32_t rtc_now = RTC->CNT;
#endif

	RTC_CompareSet(TD_RTC_USER, (RTC_CounterGet() + delay) & 0xFFFFFF);

	// We *MUST* wait new COMP is taken in account
	RTC_WAIT_BUSY(RTC_SYNCBUSY_COMP1);

	// Now we can safely clear RTC->IF flag.
	RTC_IntClear(RTC_IFC_COMP1);

	// This flag seems to be in sync with IRQ triggers. So we do not need to busy-wait
	// for it (it does not trigger IRQs even if we enable IRQs at next instruction)

#ifdef DEBUG_RTC_TRAP
	if (((RTC->CNT - rtc_now) & 0xFFFFFF) > 4) {
		tfp_printf("****RTC_AlarmAfter late init : %d !!\r\n", (RTC->CNT - rtc_now) & 0xFFFFFF);
	}
#endif

	__set_PRIMASK(msk);
}

/***************************************************************************//**
 * @brief
 *   Enter into sleep mode (EM2).
 ******************************************************************************/
void TD_RTC_Sleep(void)
{
	RTC_IntClear(RTC_IFC_COMP0);
	RTC_IntEnable(RTC_IF_COMP0);
	EMU_EnterEM2(false);
}

/***************************************************************************//**
 * @brief
 *   Set RTC system interrupt handler.
 *
 * @param[in] function
 *   Pointer to function to handle RTC system interrupts.
 ******************************************************************************/
TD_RTC_handler_t TD_RTC_SetSystemHandler(TD_RTC_handler_t function)
{
	TD_RTC_handler_t old = TD_RTC_SystemHandler;
	TD_RTC_SystemHandler = function;
	return old;
}

/***************************************************************************//**
 * @brief
 *   Set RTC user interrupt handler.
 *
 * @param[in] function
 *   Pointer to function to handle RTC user interrupts.
 *
 *  @note
 *   This function must not be called when TD_Scheduler is in use.
 ******************************************************************************/
TD_RTC_handler_t TD_RTC_SetUserHandler(TD_RTC_handler_t function)
{
	TD_RTC_handler_t old = TD_RTC_UserHandler;
	TD_RTC_UserHandler = function;
	return old;
}

/***************************************************************************//**
 * @brief
 *   Set RTC overflow interrupt handler.
 *
 * @param[in] function
 *   Pointer to function to handle RTC overflow interrupts.
 ******************************************************************************/
TD_RTC_handler_t TD_RTC_SetOverflowHandler(TD_RTC_handler_t function)
{
	TD_RTC_handler_t old = TD_RTC_OverflowHandler;
	TD_RTC_OverflowHandler = function;
	return old;
}

#ifdef TD1202_KEEPALIVE
/***************************************************************************//**
 * @brief
 *   Set keep-alive interrupt handler.
 *
 * @param[in] function
 *   Pointer to function to handle keep-alive interrupts.
 *
 * @param[in] period
 *   keep-alive period in seconds.
 ******************************************************************************/
void TD_RTC_SetKeepAliveHandler(TD_RTC_handler_t function, uint32_t period)
{

#ifndef LIB_TDCORE_TINY
	TD_RTC_KeepAliveLimit = period;
#endif

	TD_RTC_KeepAliveHandler = function;
}
#endif

/***************************************************************************//**
 * @brief
 *   Perform a delay calibration.
 *
 * @param[in] udelay
 *   The calibration delay in µs.
 ******************************************************************************/
void TD_RTC_CalibratedDelay(uint32_t udelay)
{
	static uint32_t clock = 0;
	long long delay;

	if (clock == 0) {
		clock = CMU_ClockFreqGet(cmuClock_HFPER);
		clock /= CMU_ClockDivGet(cmuClock_HFPER);
	}
	delay = clock;
	delay *= udelay;
	delay /= 1000000;
	CMU_Calibrate(delay, cmuOsc_HFRCO);
}

/***************************************************************************//**
 * @brief
 *   Returns the difference between current time and a reference time.
 *
 * @param[in] reference
 *   The reference time to compare to.
 *
 * @return
 *   Returns the absolute difference with the reference time.
 ******************************************************************************/
uint32_t TD_RTC_TimeDiff(uint32_t reference)
{
	return (RTC_CounterGet() - reference) & 0xFFFFFF;
}

/***************************************************************************//**
 * @brief Set a time offset to clock timer.
 *
 * @param[in] delta
 *   The offset to apply to the clock timer.
 ******************************************************************************/
void TD_RTC_SetOffsetTime(int delta)
{
	TD_RTC_OffsetTime = delta;
}

/***************************************************************************//**
 * @brief Returns the number of overflows that occurred since start time.
 *
 * @return
 *  Returns the number of overflows that occurred since start time.
 ******************************************************************************/
uint32_t TD_RTC_GetOverflowCounter(void)
{
	return TD_RTC_OverflowCounter;
}

/***************************************************************************//**
 * @brief Returns overflow status
 *
 * @return
 *  Return true if the chip woke-up because of an overflow IRQ
 ******************************************************************************/
bool TD_RTC_IsOverflowed(void)
{
	return TD_RTC_Overflow;
}

/***************************************************************************//**
 * @brief clear RTC overflow flag
 ******************************************************************************/
void TD_RTC_ClearOverflow(void)
{
	TD_RTC_Overflow = false;
}

/***************************************************************************//**
 * @brief Process safely IRQ based RTC functions
 *
 ******************************************************************************/
void TD_RTC_Process(void)
{
	if (TD_RTC_Overflow) {
		TD_RTC_Overflow = false;
	}

	// Handle keep-alive periodic events safely
	if (TD_RTC_KeepAlive) {
		TD_RTC_KeepAlive = false;
		if (TD_RTC_KeepAliveHandler) {
			TD_RTC_KeepAliveHandler();
		}
	}
}

/***************************************************************************//**
 * @brief Returns the current system time
 *
 * @param[in] timer
 *   Pointer to copy the current system time to, if not NULL.
 *
 * @return
 *   Returns the current system time in seconds.
 *
 * @note
 *   it should return -1 if system time is not available, but it currently doesn't.
 *
 *****************************************************************************/
time_t __time32(time_t *timer)
{
	time_t t;

	// Add time based on number of counter overflows
	t = TD_RTC_OverflowCounter * TD_RTC_OVERFLOW_PERIOD;

	// Correct if overflow interval is not an integer
	if (TD_RTC_OVERFLOWED != 0) {
		t += TD_RTC_OverflowCounter * TD_RTC_OVERFLOWED / TD_RTC_TICKS_PER_SECOND;
	}

	// Add the number of seconds for RTC
	t += RTC->CNT / TD_RTC_TICKS_PER_SECOND;
	t += TD_RTC_OffsetTime;

	// Copy system time to timer if not NULL
	if (timer) {
		*timer = t;
	}
	return t;
}

/** @} */

/** @} (end addtogroup RTC) */
