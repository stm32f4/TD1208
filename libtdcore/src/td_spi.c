/***************************************************************************//**
 * @file
 * @brief Serial Peripheral Interface (SPI) peripheral API for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.0.0
 * @section License
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

#include <stdint.h>
#include <stdbool.h>

#include <em_cmu.h>
#include <em_usart.h>
#include <em_gpio.h>
#include <em_assert.h>

#include "td_core.h"
#include "td_gpio.h"
#include "td_spi.h"
#include <td_printf.h>
#include "td_trap.h"

/***************************************************************************//**
 * @addtogroup SPI SPI
 * @brief Serial Peripheral Interface (SPI) peripheral API for the TDxxxx RF modules
 * @{
 ******************************************************************************/

/* @details
 *
 * The TD_SPI module uses a locking mechanism in order to share an SPI bus access.
 * For this to work, there should be only one peripheral which uses the SPI bus
 * in IRQ at a given time. All peripherals that are not using IRQs can lock the SPI
 * bus outside IRQ handling routines. If the one which needs IRQ can't
 * lock SPI within the IRQ, the corresponding callback will be called as soon as
 * the SPI is released.
 *
 * Therefore, only IRQ callbacks should be saved into the queue. They will be
 * called as soon as the SPI is released as SPI should always be available outside
 * the IRQ handling routines. As the callback will be call outside IRQ handling,
 * it must be protected against it.
 */

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup SPI_DEFINES Defines
 * @{ */

/** High clock frequency to use when communicating with the SPI bus (1 MHz). */
#define LOW_SPEED_CLOCK             1000000

/** Low clock frequency to use when communicating with the SPI bus (7 MHz). */
#define HIGH_SPEED_CLOCK            7000000

/** Maximum number of SPI busses */
#define MAX_SPI_BUS		2

//#define SPI_DEBUG
//#define SPI_DEBUG_INFO
//#define SPI_DEBUG_STAMP

#ifdef SPI_DEBUG

/** printf macro for SPI debug */
#define DEBUG_PRINTF(...)	tfp_printf(__VA_ARGS__)
#else

/** printf macro for SPI debug */
#define DEBUG_PRINTF(...)
#endif

#ifdef SPI_DEBUG_INFO

/** printf macro for SPI verbose debug */
#define DEBUG_PRINTF_INFO(...)	tfp_printf(__VA_ARGS__)
#else

/** printf macro for SPI verbose debug */
#define DEBUG_PRINTF_INFO(...)
#endif

/** USART used for RF access in SPI */
#define USART TD_SPI_Conf[id].usart

/** Maximum number of SPI lock call-back functions. Note : queue mode, need always one empty slot, so real queue length callback is one less*/
#define MAX_LOCKED_CALLBACK 3

/** Callback SPI */
typedef struct {
	uint8_t 				top;           				///< Top of queue
	uint8_t 				bottom;        				///< Bottom of queue
	uint8_t 				lock;						///< Lock counter
	uint8_t 				lock_id;					///< Id that locked bus
	bool 					inited;						///< Bus has been initialized
	uint32_t 				freq;						///< Current frequency
	USART_ClockMode_TypeDef	mode;						///< Current mode
	TD_SPI_LockedCallback 	cb[MAX_LOCKED_CALLBACK]; 	///< Callback queue
	uint8_t					flush_in_progress;			///< Currently flushing callback queue

#ifdef SPI_DEBUG_STAMP
	uint32_t				lock_stamp[MAX_LOCKED_CALLBACK]; 	///< Timestamp of lock
#endif

} TD_SPI_t;

/** Array of all SPI buses */
static TD_SPI_t SPI[MAX_SPI_BUS] = {{0}, {0}};

/** Array of all peripherals IDs using buses */
extern TD_SPI_Conf_t *TD_SPI_Conf;

static uint32_t 			ref_freq;					///< System reference frequency

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup SPI_GLOBAL_FUNCTIONS Global Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Get USART struct from bus number
 ******************************************************************************/
USART_TypeDef *TD_SPI_GetUsart(uint8_t bus)
{
	switch (bus) {
	case 0:
		return USART0;
		break;
	case 1:
		return USART1;
		break;
	default:
		TD_Trap(TRAP_SPI_INVALID_BUS, bus);
	}
	return NULL;
}

/***************************************************************************//**
 * @brief
 *   Initialize the SPI module.
 ******************************************************************************/
void TD_SPI_InitBus(uint8_t bus)
{
	uint32_t loc;
	GPIO_Port_TypeDef sdi_port, sdo_port, sclk_port;
	uint8_t sdo_bit, sdi_bit, sclk_bit;
	USART_TypeDef *usart;

	DEBUG_PRINTF("SPI init bus:%d\r\n", bus);
	if (SPI[bus].inited) {
		DEBUG_PRINTF("SPI already inited\r\n");
		return;
	}
	usart = TD_SPI_GetUsart(bus);
	switch (bus) {
	case 0:
		CMU_ClockEnable(cmuClock_USART0, true);
		loc = USART_ROUTE_LOCATION_LOC0;
		switch (loc) {
		case USART_ROUTE_LOCATION_LOC0:
			sdi_port = SDI_RF_PORT;
			sdi_bit = SDI_RF_BIT;
			sdo_port = SDO_RF_PORT;
			sdo_bit = SDO_RF_BIT;
			sclk_port = SCLK_RF_PORT;
			sclk_bit = SCLK_RF_BIT;
			break;
		default:
			TD_Trap(TRAP_SPI_NI, loc);
			break;
		}
		break;
	case 1:
		CMU_ClockEnable(cmuClock_USART1, true);
		loc = USART_ROUTE_LOCATION_LOC0;
		switch (loc) {
		default:
			TD_Trap(TRAP_SPI_NI, loc);
			break;
		}
		break;
	}

	// Enabling clock to USART
	CMU_ClockEnable(cmuClock_HFPER, true);
	usart->IRCTRL |= USART_CTRL_TXBIL;

	// Enabling pins and setting location, SPI CS not enable
	usart->ROUTE = USART_ROUTE_TXPEN |
				   USART_ROUTE_RXPEN |
				   USART_ROUTE_CLKPEN |
				   loc;

	// Set required GPIOs to SPI mode
	GPIO_PinModeSet(sdi_port, sdi_bit, gpioModePushPull, 0);
	GPIO_PinModeSet(sdo_port, sdo_bit, gpioModeInput, 0);
	GPIO_PinModeSet(sclk_port, sclk_bit, gpioModePushPull, 0);

	// Further initialize the USART in master SPI mode (MSB first)
	usart->CTRL = (USART_CTRL_SYNC | USART_CTRL_MSBF) ;
	usart->CLKDIV = 0;
	usart->CMD = USART_CMD_MASTEREN | (USART_CMD_RXEN | USART_CMD_TXEN);
	SPI[bus].inited = true;
	SPI[bus].freq = 0;
	SPI[bus].mode = (USART_ClockMode_TypeDef)0xFF;
}

/***************************************************************************//**

 * @brief
 *   Update the SPI peripheral with given baudrate and mode
 ******************************************************************************/
void TD_SPI_UpdateConf(uint8_t id)
{
	uint8_t bus = TD_SPI_Conf[id].bus;

	if (!SPI[bus].inited) {
		TD_SPI_InitBus(bus);
	}
	if (SPI[bus].freq != TD_SPI_Conf[id].freq) {
		SPI[bus].freq = TD_SPI_Conf[id].freq;
		DEBUG_PRINTF("SPI:%dF=%d\r\n", bus, SPI[bus].freq);
		USART_BaudrateSyncSet(USART, ref_freq, SPI[bus].freq);
	}
	if (SPI[bus].mode != TD_SPI_Conf[id].mode) {
		SPI[bus].mode = TD_SPI_Conf[id].mode;
		USART->CMD = USART_CMD_RXDIS | USART_CMD_TXDIS | USART_CMD_MASTERDIS |
					 USART_CMD_RXBLOCKDIS | USART_CMD_TXTRIDIS | USART_CMD_CLEARTX | USART_CMD_CLEARRX;
		USART->CTRL = (USART->CTRL & (~0x300)) | ((uint32_t) SPI[bus].mode);
		USART->CMD = USART_CMD_MASTEREN | (USART_CMD_RXEN | USART_CMD_TXEN);
		DEBUG_PRINTF("SPI:%dM=%d\r\n", bus, SPI[bus].mode);
	}
}

/***************************************************************************//**
 * @brief
 *   Dump current lock queue
 ******************************************************************************/
void TD_SPI_LockDump(void)
{
	uint8_t i,bus;

	for (bus = 0; bus < MAX_SPI_BUS; bus++) {
		tfp_printf("Locked byid:%d cnt:%d\r\n", SPI[bus].lock_id, SPI[bus].lock);
		tfp_printf("SPI Lock Dump, bus:%d bottom:%d top:%d\r\n", bus, SPI[bus].bottom, SPI[bus].top);
		for (i = 0; i < MAX_LOCKED_CALLBACK; i++) {

#ifdef SPI_DEBUG_STAMP
			tfp_printf("Bus CB %d:0x%08X Lock stamp:%d\r\n", i, SPI[bus].cb[i], SPI[bus].lock_stamp[i]);
#else
			tfp_printf("Bus CB %d:0x%08X \r\n", i, SPI[bus].cb[i]);
#endif

		}
	}
}

/***************************************************************************//**
 * @brief
 *   Try to lock the given SPI bus.
 *
 * @param[in] id
 *   The unique ID used for SPI bus locking.
 *
 * @param[in] callback
 *   Pointer to the call-back function that will be called when the bus is unlocked.
 *
 * @return
 * 	 true if bus was successfully locked, false otherwise
 ******************************************************************************/
bool TD_SPI_Lock(uint8_t id, TD_SPI_LockedCallback callback)
{
	uint8_t msk, ret, top, bus;

	DEBUG_PRINTF("Lck%d\r\n", id);
	EFM_ASSERT(id <= CONFIG_MAX_SPI_ID);
	bus = TD_SPI_Conf[id].bus;
	EFM_ASSERT(bus < MAX_SPI_BUS);
	EFM_ASSERT(SPI[bus].lock != 0xFF);
	msk = __get_PRIMASK();
	if (!msk) {
		__set_PRIMASK(1);
	}

	// We have already locked this bus for this usage, just increment lock count
	if (SPI[bus].lock && (SPI[bus].lock_id == id || SPI[bus].lock_id == TD_SPI_Conf[id].friend_id)) {
		SPI[bus].lock++;
		ret = true;

		// Bus is not actually locked, lock it
	} else if (!SPI[bus].lock) {
		SPI[bus].lock_id = id;
		SPI[bus].lock = 1;
		TD_SPI_UpdateConf(id);
		ret = true;
	} else {

		// Bus is locked, but for another usage
		if (callback) {

			// We should append callback to queue
			top = SPI[bus].top + 1;
			if (top >= MAX_LOCKED_CALLBACK) {
				top = 0;
			}

			// Enough room
			if (top != SPI[bus].bottom) {
				SPI[bus].cb[SPI[bus].top] = callback;

#ifdef SPI_DEBUG_STAMP
				SPI[bus].lock_stamp[SPI[bus].top] = RTC->CNT;
#endif

				SPI[bus].top = top;
			} else {
				TD_Trap(TRAP_SPI_MAX_LOCK, (bus << 8) | id);
			}
			DEBUG_PRINTF("Lock add 0x%08X, lbyid:%d cnt:%d\r\n", callback, SPI[bus].lock_id, SPI[bus].lock);
		}
		ret = false;
	}
	DEBUG_PRINTF_INFO("Cnt%d\r\n", SPI[bus].lock);
	if (!msk) {
		__set_PRIMASK(0);
	}
	return ret;
}

/***************************************************************************//**
 * @brief
 *   Unlock the given SPI bus.
 *
 * @param[in] id
 *   The unique ID used for SPI bus locking.
 ******************************************************************************/
void TD_SPI_UnLock(uint8_t id)
{
	TD_SPI_LockedCallback temp;
	uint32_t msk;
	uint8_t bus;

	EFM_ASSERT(id <= CONFIG_MAX_SPI_ID);
	bus = TD_SPI_Conf[id].bus;
	EFM_ASSERT(bus < MAX_SPI_BUS);
	EFM_ASSERT(SPI[bus].lock && SPI[bus].lock_id == id);
	DEBUG_PRINTF("UnL%d\r\n", id);
	DEBUG_PRINTF_INFO("Cnt%d\r\n", SPI[bus].lock);
	msk = __get_PRIMASK();
	__set_PRIMASK(1);
	if (SPI[bus].lock && (SPI[bus].lock_id == id || SPI[bus].lock_id == TD_SPI_Conf[id].friend_id)) {

		// Bus not locked, and no flushing of queue in callstack
		if ((--SPI[bus].lock) == 0 && !SPI[bus].flush_in_progress) {

			// Now, we are flushing queue
			SPI[bus].flush_in_progress = true;

			// Process all callback to flush in queue
			while (SPI[bus].top != SPI[bus].bottom) {

				// There is callback to call, get it
				temp = SPI[bus].cb[SPI[bus].bottom];

				// Remove it
				SPI[bus].bottom++;
				if (SPI[bus].bottom >= MAX_LOCKED_CALLBACK) {
					SPI[bus].bottom = 0;
				}
				__set_PRIMASK(msk);

				// Call it, IRQ available if needed
				DEBUG_PRINTF("Lock feed 0x%08X\r\n", temp);
				(*temp)();
				msk = __get_PRIMASK();
				__set_PRIMASK(1);
			}
			SPI[bus].flush_in_progress = false;
		}
	} else {
		TD_Trap(TRAP_SPI_INVALID_UNLOCK, (bus << 8) | id);
	}
	__set_PRIMASK(msk);
}

/***************************************************************************//**
 * @brief
 *   Register a SPI register with given parameters
 *
 * @param[in] id
 *   ID usage to initialize
 *
 * @param[in] friend_id
 *   Other ID on which we can take gracefully take ownership. Set to 0xFF is no friend id
 *
 * @param[in] bus
 * 	 Bus to speak on
 *
 * @param[in] freq
 * 	 Speed of bus for this use
 *
 * @param[in] mode
 * 	 SPI mode for this use
 ******************************************************************************/
void TD_SPI_Register(uint8_t id, uint8_t friend_id, uint8_t bus, uint32_t freq, USART_ClockMode_TypeDef mode)
{
	EFM_ASSERT(id <= CONFIG_MAX_SPI_ID);
	EFM_ASSERT(bus < MAX_SPI_BUS);
	if (id > CONFIG_MAX_SPI_ID) {
		TD_Trap(TRAP_SPI_INVALID_ID, id);
	}
	if (bus >= MAX_SPI_BUS) {
		TD_Trap(TRAP_SPI_INVALID_BUS, id);
	}
	TD_SPI_Conf[id].friend_id = friend_id;
	TD_SPI_Conf[id].bus = bus;
	TD_SPI_Conf[id].freq = freq;
	ref_freq = CMU_ClockFreqGet(cmuClock_HFPER);
	TD_SPI_Conf[id].mode = mode;
	TD_SPI_Conf[id].usart = TD_SPI_GetUsart(bus);
}

/***************************************************************************//**
 * @brief
 *   Write a buffer to the SPI device.
 *
 * @note
 *   This function is waiting actively for the bytes to be transfered.
 *
 * @param[in] id
 *   The unique ID used for SPI bus locking.
 *
 * @param[in] count
 *   The count of bytes to write.
 *
 * @param[in] buffer
 *   Pointer to the buffer to write to.
 ******************************************************************************/
void TD_SPI_WriteBuffer(uint8_t id, uint8_t count, uint8_t *buffer)
{
	//DEBUG_PRINTF("SPI Write buffer : USART0 ? : %d\r\n",USART==USART0);
	USART->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
	while (count--) {
		while (!(USART->STATUS & USART_STATUS_TXBL)) {
			;
		}
		USART->TXDATA = *buffer++;
	}
	while (!(USART->STATUS & USART_STATUS_TXC)) {
		;
	}
}

/***************************************************************************//**
 * @brief
 *   Get next PN9 random sequence number
 *
 * @param[in] pn9
 *   PN9 in number
 *
 * @param[out] br
 *   PN9 out seq number, bit reversed
 *
 * @return
 *   PN9 next value
 ******************************************************************************/
static uint16_t TD_SPI_PN9_Next(uint16_t pn9, uint8_t *br)
{
	uint8_t bit;
	uint8_t msb;

	*br = 0;
	for (bit = 8; bit; bit--) {
		msb = pn9 & 0x21;
		pn9 >>= 1;
		(*br) <<= 1;
		if (msb == 0x20 || msb == 0x01) {
			pn9 |= 0x100;
			(*br) |= 1;
		}
	}
	(*br) >>= 1;
	if (pn9 & 0x1) {
		(*br) |= 0x80;
	}
	return pn9;
}

/***************************************************************************//**
 * @brief
 *   Write a buffer to the SPI device with PN9 whitening
 *
 * @note
 *   This function is waiting actively for the bytes to be transfered.
 *
 * @param[in] id
 *   The unique ID used for SPI bus locking.
 *
 * @param[in] count
 *   The count of bytes to write.
 *
 * @param[in] buffer
 *   Pointer to the buffer to write to.

 * @param[in] reset
 *   Reset seed generator
 ******************************************************************************/
void TD_SPI_WriteBuffer_PN9(uint8_t id, uint8_t count, uint8_t *buffer, bool reset)
{
	static uint16_t PN9;
	uint8_t br;

	if (reset) {
		PN9 = 0x1FF;
	}
	USART->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
	while (count--) {
		PN9 = TD_SPI_PN9_Next(PN9, &br);
		while (!(USART->STATUS & USART_STATUS_TXBL)) {
			;
		}
		USART->TXDATA = (*buffer++) ^ (br);
	}
	while (!(USART->STATUS & USART_STATUS_TXC)) {
		;
	}
}

/***************************************************************************//**
 * @brief
 *   Read a buffer from the SPI device.
 *
 * @note
 *   This function is waiting actively for the bytes to be transfered.
 *
 * @param[in] id
 *   The unique ID used for SPI bus locking.
 *
 * @param[in] count
 *   The count of bytes to read.
 *
 * @param[out] buffer
 *   Pointer to the buffer to read the data into.
 ******************************************************************************/
void TD_SPI_ReadBuffer(uint8_t id, uint8_t count, uint8_t *buffer)
{
	USART->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;

	// Wait buffer empty
	while (!(USART->STATUS & USART_STATUS_TXBL)) {
		;
	}
	while (count--) {
		USART->TXDATA = 0xFF;
		// Wait data available (TXDATA sent)
		while (!(USART->STATUS & USART_STATUS_RXDATAV)) {
			;
		}
		if (buffer) {
			*buffer++ = USART->RXDATA;
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Write a single byte to the SPI device.
 *
 * @note
 *   This function is waiting actively for the bytes to be transfered.
 *
 * @param[in] id
 *   The unique ID used for SPI bus locking.
 *
 * @param[in] c
 *   The byte to write.
 ******************************************************************************/
void TD_SPI_WriteByte(uint8_t id, uint8_t c)
{
	USART->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
	USART->TXDATA = c;
	while (!(USART->STATUS & USART_STATUS_TXC)) {
		;
	}
}

/***************************************************************************//**
 * @brief
 *   Write then Read 4 bytes to and from a SPI device
 *
 * @note
 *   This function is waiting actively for the bytes to be transfered.
 *
 * @param[in] id
 *   The unique ID used for SPI bus locking.
 *
 * @param[in] value
 *   The value to write
 *
 * @return
 *   The value read
 ******************************************************************************/
uint32_t TD_SPI_WriteReadDouble(uint8_t id, uint32_t value)
{
	uint32_t read;

	USART->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
	USART->TXDOUBLE = value;
	while (!(USART->STATUS & USART_STATUS_TXC)) {
		;
	}
	read = USART->RXDOUBLE;
	return read;
}

/***************************************************************************//**
 * @brief
 *   Write 4 bytes to a SPI device.
 *
 * @note
 *   This function is waiting actively for the bytes to be transfered.
 *
 * @param[in] id
 *   The unique ID used for SPI bus locking.
 *
 * @param[in] value
 *  The value to write
 ******************************************************************************/
void TD_SPI_WriteDouble(uint8_t id, uint32_t value)
{
	USART->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
	USART->TXDOUBLE = value;
	while (!(USART->STATUS & USART_STATUS_TXC)) {
		;
	}
	return;
}

/***************************************************************************//**
 * @brief
 *   Back-to-Back SPI access.
 *
 * @param[in] id
 *   The unique ID used for SPI bus locking.
 *
 * @param[out] write
 *  Pointer to the buffer to write to the device. If buffer is NULL, will send 0
 *
 * @param[out] read
 *   Pointer to the buffer to read from the device, buffer updated only if data different from 0xFF
 *
 * @param[in] count
 *   The count of bytes to write/read.
 *
 * @return
 *   Count of byte read
 ******************************************************************************/
uint16_t TD_SPI_BackToBack(uint8_t id, uint8_t *write, uint8_t *read, uint16_t count)
{
	volatile uint8_t dummy;
	uint8_t *rd_base = read;

	while (count--) {
		while (!(USART->STATUS & USART_STATUS_TXBL)) {
			;
		}
		USART->TXDATA = write ? (*write++) : 0;
		while (!(USART->STATUS & USART_STATUS_RXDATAV)) {
			;
		}
		dummy = USART->RXDATA;
		if (read && (dummy != 0xFF)) {
			*read++ = dummy;
		}
	}
	return read - rd_base;
}

/***************************************************************************//**
 * @brief
 *   Start Back-to-Back SPI access.
 *
 * @param[in] id
 *   The unique ID used for SPI bus locking.
 ******************************************************************************/
void TD_SPI_StartBackToBack(uint8_t id)
{
	USART->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
}

/***************************************************************************//**
 * @brief
 *   Stop Back-to-Back SPI access.
  *
 * @param[in] id
 *   The unique ID used for SPI bus locking.
 ******************************************************************************/
void TD_SPI_EndBackToBack(uint8_t id)
{
}

/** @} */
/** @} (end addtogroup SPI) */
