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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <string.h>

#include <em_emu.h>

#include "td_core.h"
#include "td_printf.h"
#include "td_scheduler.h"
#include "td_rtc.h"
#include "td_utils.h"
#include "td_gpio.h"

/***************************************************************************//**
 * @addtogroup UTILS
 * @brief Utility functions for the TDxxxx RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup UTILS_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Convert an unsigned long long (64 bits) to ASCII.
 *
 * @param[in] num
 *   Number to convert.
 *
 * @param[in] base
 *   Arithmetic base to convert to.
 *
 * @param[in] uc
 *   Convert digits to uppercase if base > 10 and uc is different from 0.
 *
 * @param[out] bf
 *   Pointer to output character buffer.
 ******************************************************************************/
void uli2a(unsigned long int num, unsigned int base, int uc, char *bf)
{
	int n = 0;
	unsigned int d = 1;

	while (num / d >= base) {
		d *= base;
	}
	while (d != 0) {
		int dgt = num / d;
		num %= d;
		d /= base;
		if (n || dgt > 0 || d == 0) {
			*bf++ = dgt + (dgt < 10 ? '0' : (uc ? 'A' : 'a') - 10);
			++n;
		}
	}
	*bf = 0;
}

/***************************************************************************//**
 * @brief
 *   Convert a long long (64 bits) to ASCII.
 *
 * @param[in] num
 *   Number to convert.
 *
 * @param[out] bf
 *   Pointer to output character buffer.
 ******************************************************************************/
void li2a(long int num, char *bf)
{
	if (num < 0) {
		num = -num;
		*bf++ = '-';
	}
	uli2a(num, 10, 0, bf);
}

/***************************************************************************//**
 * @brief
 *   Convert an unsigned integer (32 bits) to ASCII.
 *
 * @param[in] num
 *   Number to convert.
 *
 * @param[in] base
 *   Arithmetic base to convert to.
 *
 * @param[in] uc
 *   Convert digits to uppercase if base > 10 and uc is different from 0.
 *
 * @param[out] bf
 *   Pointer to output character buffer.
 ******************************************************************************/
void ui2a(unsigned int num, unsigned int base, int uc, char *bf)
{
	int n = 0;
	unsigned int d = 1;

	while (num / d >= base) {
		d *= base;
	}
	while (d != 0) {
		int dgt = num / d;
		num %= d;
		d /= base;
		if (n || dgt > 0 || d == 0) {
			*bf++ = dgt + (dgt < 10 ? '0' : (uc ? 'A' : 'a') - 10);
			++n;
		}
	}
	*bf = 0;
}

/***************************************************************************//**
 * @brief
 *   Convert an integer (32 bits) to ASCII.
 *
 * @param[in] num
 *   Number to convert.
 *
 * @param[out] bf
 *   Pointer to output character buffer.
 ******************************************************************************/
void i2a(int num, char *bf)
{
	if (num < 0) {
		num = -num;
		*bf++ = '-';
	}
	ui2a(num, 10, 0, bf);
}

/***************************************************************************//**
 * @brief
 *   Convert an ASCII character to decimal.
 *
 * @param[in] ch
 *   Character to convert.
 *
 * @return
 *   Arithmetic base to convert to.
 ******************************************************************************/
int a2d(char ch)
{
	if (ch >= '0' && ch <= '9') {
		return(ch - '0');
	} else if (ch >= 'a' && ch <= 'f') {
		return(ch - 'a' + 10);
	} else if (ch >= 'A' && ch <= 'F') {
		return(ch - 'A' + 10);
	} else {
		return -1;
	}
}

/***************************************************************************//**
 * @brief
 *   Convert an ASCII-coded number to binary.
 *
 * @param[in] ch
 *   Overhead character.
 *
 * @param[out] src
 *   Handle to the string to convert from.
 *
 * @param[in] base
 *   Arithmetic base to convert from.
 *
 * @param[out] nump
 *   Pointer to the output integer number buffer.
 ******************************************************************************/
char a2i(char ch, char **src, int base, int *nump)
{
	char *p = *src;
	int num = 0;
	int digit;

	while ((digit = a2d(ch)) >= 0) {
		if (digit > base) {
			break;
		}
		num = num * base + digit;
		ch = *p++;
	}
	*src = p;
	*nump = num;
	return ch;
}

/***************************************************************************//**
* @brief
 *   Convert a string to long long integer and ignore up to 1 char.
 *
 * @param[in] instr
 *  Pointer to the input string.
 *
 * @param[in] ignore
 *  Char to ignore within the String. 0 if not used.
 *
 * @return
 *  The converted long long integer value.
 ******************************************************************************/
long long atolli(char *instr, char ignore)
{
	long long retval = 0, sign = 1;

	// Skip leading space and tab characters
	for (; *instr == ' ' || *instr == '\t'; instr++) {
		;
	}

	// Handle optional sign
	if (*instr == '-') {
		sign = -1;
		instr++;
	}
	if (*instr == '+') {
		instr++;
	}
	for (; *instr && ((*instr >= '0' && *instr <= '9') || (*instr == ignore)); instr++) {

		if (*instr != ignore) {
			retval = (10 * retval) + (*instr - '0');
		}
	}
	return sign * retval;
}

/***************************************************************************//**
 * @brief
 *   Convert a string to long long integer.
 *
 * @param[in] instr
 *  Pointer to the input string.
 *
 * @return
 *  The converted long long integer value.
 ******************************************************************************/
long long atoll(char *instr)
{
	return atolli(instr, 0);
}

/***************************************************************************//**
 * @brief
 *   Convert a string to integer.
 *
 * @param[in] instr
 *  Pointer to the input string.
 *
 * @return
 *  The converted integer value.
 ******************************************************************************/
int atoi(char *instr)
{
	int retval = 0, sign = 1;

	// Skip leading space and tab characters
	for (; *instr == ' ' || *instr == '\t'; instr++) {
		;
	}

	// Handle optional sign
	if (*instr == '-') {
		sign = -1;
		instr++;
	}
	if (*instr == '+') {
		instr++;
	}
	for (; *instr && *instr >= '0' && *instr <= '9'; instr++) {
		retval = (10 * retval) + (*instr - '0');
	}
	return sign * retval;
}

#ifndef __ICCARM__
/***************************************************************************//**
 * @brief
 *   Copy a byte into a memory region.
 *
 * @param[out] s
 *  Pointer to the output memory region.
 *
 * @param[in] c
 *  The byte value to copy.
 *
 * @param[in] n
 *  The memory region size in bytes.
 *
 * @return
 *  Returns a pointer to the output buffer.
 ******************************************************************************/
void *memset(void *s, int c, size_t n)
{
	char *s8 = (char *) s;
	int i;

	for (i = 0; i < n; i++) {
		s8[i] = c;
	}
	return s;
}

/***************************************************************************//**
 * @brief
 *   Copy a region of memory into a another.
 *
 * @param[out] d
 *  Pointer to the destination memory region.
 *
 * @param[in] s
 *  Pointer to the source memory region.
 *
 * @param[in] n
 *  The source memory region size in bytes.
 *
 * @return
 *  Returns a pointer to the output buffer.
 ******************************************************************************/
void *memcpy(void *d, const void *s, size_t n)
{
	const char *s8 = (const char *) s;
	char *d8 = (char *) d;
	int i;

	for (i = 0; i < n; i++) {
		d8[i] = s8[i];
	}
	return d;
}

/***************************************************************************//**
 * @brief
 *   Compare two memory regions.
 *
 * @param[in] s1
 *  Pointer to the first memory region.
 *
 * @param[in] s2
 *  Pointer to the second memory region.
 *
 * @param[in] n
 *  The source memory region size in bytes.
 *
 * @return
 *  Returns 0 if memory regions are equal, 1 if first memory region is greater
 *  and -1 if first memory region is lighter than the second one.
 ******************************************************************************/
int memcmp(const void *s1, const void *s2, size_t n)
{
	const uint8_t *sc1 = (const uint8_t *) s1;
	const uint8_t *sc2 = (const uint8_t *) s2;
	int i;

	for (i = 0; i < n; i++) {
		if (sc1[i] > sc2[i]) {
			return 1;
		}
		if (sc1[i] < sc2[i]) {
			return -1;
		}
	}
	return 0;
}

#endif

/***************************************************************************//**
 * @brief
 *   Stack usage
 *
 * @return
 *  Returns stack usage in bytes.
 ******************************************************************************/
int TD_STACK_Usage(void)
{
	extern const char __cs3_stack;
	uint8_t dummy;

	return (uint32_t) &__cs3_stack - (uint32_t) &dummy;
}

/***************************************************************************//**
 * @brief
 *   IRQ Dump function
 ******************************************************************************/
void TD_IRQ_Dump(void)
{
	uint32_t irq;
	uint8_t msk;
	extern TD_GPIO_hook_t TD_GPIO_Hooks[TD_GPIO_MAX_HOOKS];

	TD_RTC_handler_t hdle;
	char *irqs = "";
	tfp_printf("=== IRQ Dump ===\r\n");
	irq = __get_IPSR();
	tfp_printf("IPSR:0x%08X\r\n", irq);
	if (!irq) {
		tfp_printf("In background function\r\n");
	} else {
		if (irq < 16) {
			switch (irq) {
			case 14:
				irqs = "PendSV/SystemRTC";
				break;
			case 15:
				irqs = "SYSTick/UserRTC(Scheduler)";
				break;
			}
			tfp_printf("SYSTEM ISR number:%d:%s\r\n", irq, irqs);
		} else {
			tfp_printf("ISR number:%d\r\n", irq - 16);
		}
	}
	tfp_printf("GPIO.IF:0x%08X\r\n", GPIO->IF);

	// Disable IRQs
	msk = __get_PRIMASK();
	__set_PRIMASK(1);

	hdle = TD_RTC_SetSystemHandler(NULL);
	TD_RTC_SetSystemHandler(hdle);
	tfp_printf("SYSHandler:0x%08X\r\n", hdle);
	hdle = TD_RTC_SetUserHandler(NULL);
	TD_RTC_SetUserHandler(hdle);
	tfp_printf("USERHandler:0x%08X\r\n", hdle);
	tfp_printf("GPIO_Hook[USER]:0x%08X[0x%08X]\r\n", TD_GPIO_Hooks[TD_GPIO_USER].callback, TD_GPIO_Hooks[TD_GPIO_USER].mask);
	tfp_printf("GPIO_Hook[SYS]:0x%08X[0x%08X]\r\n", TD_GPIO_Hooks[TD_GPIO_SYSTEM].callback, TD_GPIO_Hooks[TD_GPIO_SYSTEM].mask);
	__set_PRIMASK(msk);
}

/** @} */

/** @} (end addtogroup UTILS) */
