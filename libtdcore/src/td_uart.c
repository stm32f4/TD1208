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

#include <stdint.h>
#include <stdbool.h>

#include <em_cmu.h>
#include <em_gpio.h>
#include <em_leuart.h>

#include "td_core.h"
#include "td_uart.h"
#include "td_rtc.h"

/***************************************************************************//**
 * @addtogroup UART
 * @brief Asynchronous Receiver/Transmitter (UART) peripheral API for the TDxxxx RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup UART_DEFINES Defines
 * @{ */

/** Receive buffer size */
#define TD_UART_RXBUFSIZE               61

//#define DEBUG_UART
#ifdef DEBUG_UART

/** printf macro for UART debug */
#define DEBUG_PRINTF(...) tfp_printf(__VA_ARGS__)
#else

/** printf macro for UART debug */
#define DEBUG_PRINTF(...)
#endif

/** @} */

/*******************************************************************************
 **************************   PUBLIC VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup UART_GLOBAL_VARIABLES Global Variables
 * @{ */

/** UART receive callback function pointer */
volatile TD_UART_CALLBACK TD_UART_RxCallback = 0;

/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup UART_LOCAL_VARIABLES Local Variables
 * @{ */

/** Flag for sharing the UART port with other GPIO functions */
static bool PortShared = false;

/** UART enable flag */
static LEUART_Enable_TypeDef PortEnable = leuartDisable;

/** UART receive FIFO read index */
static int RxReadIndex = 0;

/** UART receive FIFO write index */
static int RxWriteIndex = 0;

/** UART FIFO buffer */
static char RxBuffer[TD_UART_RXBUFSIZE];

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup UART_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Initialize the UART peripheral.
 *
 * @param[in] speed
 *   The baudrate in bps.
 *
 * @param[in] rxEnable
 *   Enable receive flag.
 *
 * @param[in] shared
 *   Flag for sharing the UART port with alternative GPIO functions.
 *
 * @return
 *   Returns an opaque UART descriptor (fixed to LEUART0).
 ******************************************************************************/
void *TD_UART_Init(uint32_t speed, bool rxEnable, bool shared)
{
	// Define the LEUART0 initialization data
	LEUART_Init_TypeDef init = LEUART_INIT_DEFAULT;

	if (speed > 9600) {
		CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_CORELEDIV2);
		CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_4);

		// 14MHz/2 pre-scaled by 4
		init.refFreq = 1750000;
	} else {
		CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
		CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_1);
		init.refFreq = 0;
	}

	// Enable the LEUART0 clock
	CMU_ClockEnable(cmuClock_LEUART0, true);
	init.enable     = leuartDisable;
	init.baudrate   = speed;

	// Reseting and initializing LEUART0
	LEUART_Reset(LEUART0);
	LEUART_Init(LEUART0, &init);

	// TX PORT
	GPIO_PinModeSet(TX_PORT, TX_BIT, gpioModePushPull, 1);

	// Always enable TX
	PortEnable = leuartEnableTx;
	if (rxEnable) {

		// RX PORT
		GPIO_PinModeSet(RX_PORT, RX_BIT, gpioModeInputPull, 1);

		// Clear previous RX interrupts
		LEUART_IntClear(LEUART0, LEUART_IF_RXDATAV);
		NVIC_ClearPendingIRQ(LEUART0_IRQn);

		// Enable RX interrupts
		LEUART_IntEnable(LEUART0, LEUART_IF_RXDATAV);
		NVIC_EnableIRQ(LEUART0_IRQn);

		// Enable RX
		PortEnable |= leuartEnableRx;
	}
	if (!shared) {

		// Port will not be shared
		// Temporarily set this flag to set initial route
		PortShared = true;
		TD_UART_Start(LEUART0);
	}

	// Save port shared flag
	PortShared = shared;

	// Eventually enable UART
	LEUART_Enable(LEUART0, PortEnable);
	return(LEUART0);
}

/***************************************************************************//**
 * @brief
 *   Start using the UART peripheral.
 *
 * @param[in] p
 *   Pointer to the UART opaque descriptor.
 ******************************************************************************/
void TD_UART_Start(void *p)
{
	if (PortShared) {

		// Enable LEUART0 TX and RX route
		if (PortEnable & leuartEnableRx) {
			LEUART0->ROUTE = LEUART_ROUTE_TXPEN | LEUART_ROUTE_RXPEN | LEUART_ROUTE_LOCATION_LOC0;
			LEUART0->CMD = LEUART_CMD_TXDIS | LEUART_CMD_RXDIS | LEUART_CMD_CLEARTX | LEUART_CMD_CLEARRX;
			LEUART0->CMD = LEUART_CMD_TXEN | LEUART_CMD_RXEN;
		} else {
			LEUART0->ROUTE = LEUART_ROUTE_TXPEN | LEUART_ROUTE_LOCATION_LOC0;
			LEUART0->CMD = LEUART_CMD_TXDIS | LEUART_CMD_CLEARTX;
			LEUART0->CMD = LEUART_CMD_TXEN;
		}
		// TD_RTC_CalibratedDelay(1000);
	}
}

/***************************************************************************//**
 * @brief
 *   Stop using the UART peripheral.
 *
 * @param[in] p
 *   Pointer to the UART opaque descriptor.
 ******************************************************************************/
void TD_UART_Stop(void *p)
{
	// Wait end of transmission
	while (!(LEUART0->STATUS & LEUART_STATUS_TXC));
	if (PortShared) {

		// Disable LEUART0 TX and RX route
		LEUART0->CMD = LEUART_CMD_TXDIS | LEUART_CMD_RXDIS | LEUART_CMD_CLEARTX | LEUART_CMD_CLEARRX;
		LEUART0->ROUTE = 0;
	}
}

/***************************************************************************//**
 * @brief
 *   Send a character to the UART.
 *
 * @param[in] p
 *   Pointer to the UART opaque descriptor.
 *
 * @param[in] c
 *   The character to send.
 ******************************************************************************/
void TD_UART_Putc(void *p, char c)
{
	LEUART_Tx(LEUART0, c);
}

/***************************************************************************//**
 * @brief
 *   Receive a character from the UART.
 *
 * @return
 *   The received character if one is available, -1 otherwise.
 ******************************************************************************/
int TD_UART_GetChar(void)
{
	int c;

	if (RxReadIndex == RxWriteIndex) {
		return -1;
	}
	c = RxBuffer[RxReadIndex++];
	if (RxReadIndex == TD_UART_RXBUFSIZE) {

		// Wrapped RX read Index
		RxReadIndex = 0;
	}
	if (RxReadIndex == RxWriteIndex) {
		RxReadIndex = RxWriteIndex = 0;
	}
	return c;
}

/***************************************************************************//**
 * @brief
 *   Returns the number of available characters from the UART.
 *
 * @return
 *   The number of available characters.
 ******************************************************************************/
int TD_UART_AvailableChars(void)
{
	int count = RxWriteIndex - RxReadIndex;

	return count < 0 ? TD_UART_RXBUFSIZE + count : count;
}

/***************************************************************************//**
 * @brief
 *   Flush the UART RX buffer.
 ******************************************************************************/
void TD_UART_Flush(void)
{
	RxReadIndex = RxWriteIndex = 0;
}

/***************************************************************************//**
 * @brief
 *   Send a string to the UART.
 *
 * @param[in] string
 *   Pointer to null-terminated string to send to the UART.
 ******************************************************************************/
void TD_UART_SendString(char *string)
{
	char c;

	while ((c = *string++) != '\0') {
		LEUART_Tx(LEUART0, c);
	}
}

/***************************************************************************//**
 * @brief
 *   Send a byte buffer to the UART.
 *
 * @param[in] buffer
 *   Pointer to buffer to send to the UART.
 *
 * @param[in] length
 *   The length of the buffer to send to the UART.
 ******************************************************************************/
void TD_UART_Send(char *buffer, char length)
{
	int i;

	// Check that the transmit buffer is empty
	for (i = 0; i < length; i++) {
		LEUART_Tx(LEUART0, buffer[i]);
	}
}

/***************************************************************************//**
 * @brief
 *   UART interrupt handler.
 ******************************************************************************/
void LEUART0_IRQHandler(void)
{
	// Get the received byte
	DEBUG_PRINTF("IF:%02X ST:0x%02X EX:0x%02X\r\n", LEUART0->IF, LEUART0->STATUS, (LEUART0->RXDATAXP) >> 8);
	char data = LEUART0->RXDATA;

	if (TD_UART_RxCallback != 0) {

		// RX callback supplied
		(*TD_UART_RxCallback)(data);
	}

	if (RxWriteIndex < TD_UART_RXBUFSIZE) {

		// Enough room in receive buffer
		// Save the received byte
		RxBuffer[RxWriteIndex++] = data;
		TD_WakeMainLoop();
	} else {
		if (RxWriteIndex == RxReadIndex) {

			// Buffer overflow
			TD_WakeMainLoop();
			return;
		}
		RxWriteIndex = 0;
		TD_WakeMainLoop();
	}
}

/** @} */

/** @} (end addtogroup UART) */
