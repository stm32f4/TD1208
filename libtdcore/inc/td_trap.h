/***************************************************************************//**
 * @file
 * @brief Utility functions for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
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

#ifndef __TD_TRAP_H
#define __TD_TRAP_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup TRAP
	 * @brief Trap functions for the TDxxxx RF modules
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 *************************   DEFINES   *****************************************
	 ******************************************************************************/

	/** @addtogroup TRAP_DEFINES Defines
	 * @{ */

/** Magic value to sign a RAM trap trace */
#define TRAP_MAGIC_TRACE		0xA90B0B00

/** Maximum number of RAM TRAP traces */
#define TRAP_MAX_TRACE			19

/** Maximum number of user variables in RAM TRAP traces */
#define TRAP_MAX_USER 			9

/** Flag for global oversize detection in RAM TRAP traces */
#define TRACE_MAGIC_OVERSIZE	(1 << 0)

/** Flag for stack overrun detection in RAM TRAP traces */
#define TRACE_MAGIC_OVERSTACK	(1 << 1)

/** Flag for stack trace in RAM TRAP traces */
#define TRACE_MAGIC_STACK		(1 << 6)

/** Flag for trap detection */
#define TRACE_MAGIC_TRAP		(1 << 7)

	/** @} */

/*******************************************************************************
	 *************************   ENUM   *******************************************
	 ******************************************************************************/

	/** @addtogroup TRAP_ENUMERATIONS Enumerations
	 * @{ */

	/** TRAP information */
	typedef enum {
		TRAP_NONE,					/**< Value not used, invalid trap */
		TRAP_LINE,					/**< Trap at specific line */
		TRAP_SPI_MAX_LOCK,			/**< Trap add a callback next to an SPI locked access */
		TRAP_SPI_INVALID_UNLOCK,	/**< Trap unlock a non-locked SPI bus */
		TRAP_SPI_NOT_AVAILABLE,		/**< Trap SPI can't be unavailable here */
		TRAP_SPI_INVALID_ID,		/**< Trap SPI ID out of range */
		TRAP_SPI_INVALID_BUS,		/**< Trap SPI BUS out of range */
		TRAP_SPI_NI,				/**< Trap SPI not implemented */
		TRAP_REMOVED_CODE,			/**< Trap Code removed */
		TRAP_ILLEGAL_FREQ,			/**< Trap Frequency forbidden */
		TRAP_SCHEDULER_QUEUE_OVF,	/**< Trap Scheduler Queue overflow */
		TRAP_FLASH_VAR_FULL,		/**< Trap Flash Variables Full */
		TRAP_FLASH_POINTER_OVF,		/**< Trap Flash Pointers Overflow */
		TRAP_RADIO_CHIP,			/**< Trap Radio Chip */
		TRAP_RTC_DELAY,				/**< Trap RTC Delay */
		TRAP_NOT_ALLOWED_IN_IRQ,	/**< Trap Can't do this in IRQ */
		TRAP_CUSTOM_1,				/**< Trap User 1 */
		TRAP_CUSTOM_2,				/**< Trap User 2 */
		TRAP_CUSTOM_3,				/**< Trap User 3 */
		TRAP_CUSTOM_4,				/**< Trap User 4 */
		TRAP_CUSTOM_5,				/**< Trap User 5 */
		TRAP_CUSTOM_6,				/**< Trap User 6 */
		TRAP_CUSTOM_7,				/**< Trap User 7 */
		TRAP_CUSTOM_8,				/**< Trap User 8 */
		TRAP_LAN_CALLBACK_LATE,		/**< Trap LAN callback timeout */
		TRAP_GPS_HARD_ERR,			/**< Trap GPS Hard error */
		TRAP_NMI_FAULT,				/**< Trap Non Maskable Interrupt Fault */
		TRAP_HARD_FAULT,			/**< Hard fault */
		TRAP_MEM_MANAGE_FAULT,		/**< Memory manage fault */
		TRAP_BUS_FAULT,				/**< Bus fault */
		TRAP_USAGE_FAULT,			/**< Usage fault */
		TRAP_DUMP_NOT_AVAILABLE,	/**< This type of dump is not available  */
		TRAP_BAD_SIGFOX_ID,			/**< Bad SIGFOX ID */
		TRAP_BAD_SIGFOX_KEY,		/**< Bad SIGFOX Key */
		TRAP_TIMER_INVALID			/**< Invalid timer id  */
	}
	TD_TRAP_t;

	/** @} */

	/*******************************************************************************
	 *************************   TYPEDEFS   ****************************************
	 ******************************************************************************/

	/** @addtogroup TRAP_TYPEDEFS Typedefs
	 * @{ */

	/** Action to perform upon trap handler exit */
	typedef enum {
		TRAP_CONTINUE,				/**< Continue normal execution */
		TRAP_SLEEP,					/**< Sleep product (standard, EM2) */
		TRAP_DEEP_SLEEP,			/**< Sleep product (EM4 mode, can only be reset)*/
		TRAP_RESTART,				/**< Reboot product */
		TRAP_HANG					/**< Hang execution and stay here */
	} TD_TRAP_action_t;

	/** Trap trace */
#pragma pack(1)
	typedef struct {
		uint32_t magic;				///< Magic trace
		TD_TRAP_t trap;          	///< Trap
		uint32_t param;				///< Param
		uint16_t trace_cnt;			///< Trace count number
		uint16_t trace[TRAP_MAX_TRACE];	///< Trace data
		uint8_t user[TRAP_MAX_USER];	///< Trace count number
	} TD_TRAP_Frame_t;
#pragma pack()

	/** Callback when TRAP occurs.
	 * trap is TD_Trap_t enumeration that define source of TRAP
	 * param is context-dependent
	 * This function must return the action to take */
	typedef TD_TRAP_action_t (*TD_TRAP_callback_t)(TD_TRAP_t trap, uint32_t param);

/** Default "Removed code" handler */
#define TD_TRAP_HERE TD_TrapHere

	/** SYSTEM dumps */
	typedef enum {
		DUMP_GPIO,					/**< TD_GPIO_Dump() */
		DUMP_RF,					/**< TD_RF_Dump() */
		DUMP_IRQ,					/**< TD_IRQ_Dump() */
		DUMP_SCHEDULER,				/**< TD_SCHEDULER_Dump() */
		DUMP_UBX7,					/**< TD_UBX7_Dump() */
		DUMP_SPILOCK,				/**< TD_SPILock_Dump() */
		DUMP_LIS3DH,				/**< LIS3DH */
		DUMP_SENSOR					/**< Sensor */
	}
	TD_Dump_t;

	/** System module dump handler function type */
	typedef void (*TD_Dump_Func_t)(void);

	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup TRAP_USER_FUNCTIONS User Functions
	 * @{ */

	void TD_TRAP_Set(TD_TRAP_callback_t trap);
	void TD_Trap(TD_TRAP_t trap, uint32_t param);
	bool TD_TRAP_DirectToFlash(TD_TRAP_t *trap, uint32_t *param);
	TD_TRAP_action_t TD_TRAP_Reset_Callback(TD_TRAP_t trap, uint32_t param);
	TD_TRAP_action_t TD_TRAP_Mini_Callback(TD_TRAP_t trap, uint32_t param);
	TD_TRAP_action_t TD_TRAP_Flash_Callback(TD_TRAP_t trap, uint32_t param);
	TD_TRAP_action_t TD_TRAP_Printf_Callback(TD_TRAP_t trap, uint32_t param);
	void TD_TrapHere(void);
	void TD_TRAP_TraceDump(bool force);
	void TD_TRAP_TraceStore(TD_TRAP_t trap, uint32_t param);
	void TD_TRAP_TraceDumpSigfox(uint8_t retry);
	bool TD_TRAP_TraceUserSet(uint8_t entry, uint32_t val);
	void TD_SystemDump(TD_Dump_t dump);

	/** @} */

	/** @} (end addtogroup TRAP) */

#ifdef __cplusplus
}
#endif

#endif // __TD_TRAP_H
