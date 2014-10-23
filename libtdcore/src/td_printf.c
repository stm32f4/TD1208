/***************************************************************************//**
 * @file
 * @brief printf utility for the TDxxxx RF modules.
 * @author Kustaa Nyholm
 * @version 2.0.2
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2004 Kustaa Nyholm</b>
 * <b>(C) Copyright 2012-2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
 *******************************************************************************
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 ******************************************************************************/

#include <em_usart.h>

#include "td_core.h"
#include "td_utils.h"
#include "td_printf.h"

/***************************************************************************//**
 * @addtogroup PRINTF
 * @brief printf utility for the TDxxxx RF modules
 * @{
 ******************************************************************************/

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

/** @addtogroup PRINTF_DEFINES Defines
 * @{ */

/** Definition for long support in printf */
#define PRINTF_LONG_SUPPORT

/** @} */

/*******************************************************************************
 *************************   TYPEDEFS   ****************************************
 ******************************************************************************/

/** @addtogroup PRINTF_TYPEDEFS Typedefs
 * @{ */

/** Pointer to function for flushing characters out */
typedef void (*putcf)(void *, char);

/** Pointer to function for starting to use the output stream */
typedef void (*start)(void *);

/** Pointer to function for stopping to use the output stream */
typedef void (*stop)(void *);

/** @} */

/*******************************************************************************
 *************************   PRIVATE VARIABLES   *******************************
 ******************************************************************************/

/** @addtogroup PRINTF_LOCAL_VARIABLES Local Variables
 * @{ */

/** Pointer to opaque context for outputting characters */
static void *stdout_putp;

/** Pointer to function for flushing characters out */
static putcf stdout_putf;

/** Pointer to function for starting to use the output stream */
static start stdout_start;

/** Pointer to function for stopping to use the output stream */
static stop stdout_stop;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup PRINTF_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Output an argument with left padding.
 *
 * @param[in] putp
 *   Pointer to character output function.
 *
 * @param[in] putf
 *   Pointer to flush output function.
 *
 * @param[in] n
 *   Number of left-padding characters.
 *
 * @param[in] z
 *   Padding with zero characters if not null, with blank caracters otherwise.
 *
 * @param[in] bf
 *   Pointer to the input buffer.
 ******************************************************************************/
static void putchw(void *putp, putcf putf, int n, char z, char *bf)
{
	char fc = z ? '0' : ' ';
	char ch;
	char *p = bf;

	while (*p++ && n > 0) {
		n--;
	}
	while (n-- > 0) {
		putf(putp, fc);
	}
	while ((ch = *bf++)) {
		putf(putp, ch);
	}
}

/***************************************************************************//**
 * @brief
 *   Function to output a character to a string.
 *
 * @param[out] p
 *   Pointer to the output buffer.
 *
 * @param[in] putf
 *   Pointer to flush output function.
 *
 * @param[in] c
 *   Input character.
******************************************************************************/
static void putcp(void *p, char c)
{
	*(*((char **)p))++ = c;
}

/***************************************************************************//**
 * @brief
 *   Function to output a string.
 *
 * @param[in] putp
 *   Pointer to character output function.
 *
 * @param[in] putf
 *   Pointer to flush output function.
 *
 * @param[in] bf
 *   Pointer to the input buffer.
******************************************************************************/
static void putcs(void *putp, putcf putf, char *bf)
{
	char ch;
	while ((ch = *bf++)) {
		putf(putp, ch);
	}
}

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup PRINTF_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Output a string according to a format.
 *
 * @param[in] putp
 *   Pointer to character output function.
 *
 * @param[in] putf
 *   Pointer to flush output function.
 *
 * @param[in] fmt
 *   Pointer to format string in printf format.
 *
 * @param[in] va
 *   Variable list of arguments according to the format string.
 ******************************************************************************/
void tfp_format(void *putp, putcf putf, char *fmt, va_list va)
{
	char bf[12];

	char ch;

	while ((ch = *(fmt++))) {
		if (ch != '%') {
			putf(putp, ch);
		} else {
			char lz = 0;
#ifdef PRINTF_LONG_SUPPORT
			char lng = 0;
#endif
			int w = 0;
			ch = *(fmt++);
			if (ch == '0') {
				ch = *(fmt++);
				lz = 1;
			}
			if (ch >= '0' && ch <= '9') {
				ch = a2i(ch, &fmt, 10, &w);
			}
#ifdef PRINTF_LONG_SUPPORT
			if (ch == 'l') {
				ch = *(fmt++);
				lng = 1;
			}
#endif
			switch (ch) {
			case 0:
				goto abort;
			case 'u': {
#ifdef PRINTF_LONG_SUPPORT
				if (lng) {
					uli2a(va_arg(va, unsigned long int), 10, 0, bf);
				} else
#endif
					ui2a(va_arg(va, unsigned int), 10, 0, bf);
				putchw(putp, putf, w, lz, bf);
				break;
			}
			case 'd': {
#ifdef PRINTF_LONG_SUPPORT
				if (lng) {
					li2a(va_arg(va, unsigned long int), bf);
				} else
#endif
					i2a(va_arg(va, int), bf);
				putchw(putp, putf, w, lz, bf);
				break;
			}
			case 'x':
			case 'X':
#ifdef PRINTF_LONG_SUPPORT
				if (lng) {
					uli2a(va_arg(va, unsigned long int), 16, (ch == 'X'), bf);
				} else
#endif
					ui2a(va_arg(va, unsigned int), 16, (ch == 'X'), bf);
				putchw(putp, putf, w, lz, bf);
				break;
			case 'c':
				putf(putp, (char)(va_arg(va, int)));
				break;
			case 's':
				putchw(putp, putf, w, 0, va_arg(va, char *));
				break;
			case '%':
				putf(putp, ch);
				break;
			default:
				break;
			}
		}
	}
abort:
	;
}

/***************************************************************************//**
 * @brief
 *   Printf initialization function.
 *
 * @param[in] putp
 *   Pointer to character output function.
 *
 * @param[in] putf
 *   Pointer to flush output function.
 *
 * @param[in] start
 *   Pointer to function for starting to use the output stream.
 *
 * @param[in] stop
 *   Pointer to function for stopping to use the output stream.
 ******************************************************************************/
void init_printf(void *putp, void (*putf)(void *, char), void (*start)(void *), void (*stop)(void *))
{
	stdout_putp = putp;
	stdout_putf = putf;
	stdout_start = start;
	stdout_stop = stop;
}

/***************************************************************************//**
 * @brief
 *   Output a string according to a format.
 *
 * @param[in] fmt
 *   Pointer to format string in printf format.
 *
 * @param[in] ...
 *   Number and types of arguments according to the format string.
 ******************************************************************************/
void tfp_printf(char *fmt, ...)
{
	va_list va;

	va_start(va, fmt);
	if (stdout_start) {
		stdout_start(stdout_putp);
	}
	if (stdout_putf) {
		tfp_format(stdout_putp, stdout_putf, fmt, va);
	}
	if (stdout_stop) {
		stdout_stop(stdout_putp);
	}
	va_end(va);
}

/***************************************************************************//**
 * @brief
 *   Output a string wit hvaraible argument list according to a format.
 *
 * @param[in] fmt
 *   Pointer to format string in printf format.
 *
 * @param[in] va
 *   Variable list of arguments according to the format string.
 ******************************************************************************/
void tfp_vprintf(char *fmt, va_list va)
{
	if (stdout_start) {
		stdout_start(stdout_putp);
	}
	if (stdout_putf) {
		tfp_format(stdout_putp, stdout_putf, fmt, va);
	}
	if (stdout_stop) {
		stdout_stop(stdout_putp);
	}
}

/***************************************************************************//**
 * @brief
 *   Output a string into another string according to a format.
 *
 * @param[out] s
 *   Pointer to the output string buffer.
 *
 * @param[in] fmt
 *   Pointer to format string in printf format.
 *
 * @param[in] ...
 *   Variable list of arguments according to the format string.
 ******************************************************************************/
void tfp_sprintf(char *s, char *fmt, ...)
{
	va_list va;

	va_start(va, fmt);
	tfp_format(&s, putcp, fmt, va);
	putcp(&s, 0);
	va_end(va);
}

/***************************************************************************//**
 * @brief
 *   Dump a buffer in hexadecimal.
 *
 * @param[in] prompt
 *   Pointer to a prompt string to ouptut in front of the hexadecimal dump.
 *
 * @param[in] buffer
 *   Pointer to the input buffer.
 *
 * @param[in] length
 *   Length of the input buffer in characters.
 ******************************************************************************/
void tfp_dump(char *prompt, unsigned char *buffer, unsigned char length)
{
	static char dump[(3 * 16) + 3];
	unsigned char i, j, k;
	char hex[4];

	if (stdout_start) {
		stdout_start(stdout_putp);
	}
	if (prompt) {
		putcs(stdout_putp, stdout_putf, prompt);
	}
	for (i = 0, k = 0; i < length; i++) {
		tfp_sprintf(hex, "%02x ", buffer[i]);
		for (j = 0; hex[j]; j++) {
			dump[k++] = hex[j];
		}
		if (i && ((i % 16) == 15)) {
			dump[k++] = '\r';
			dump[k++] = '\n';
			dump[k] = '\0';
			putcs(stdout_putp, stdout_putf, dump);
			k = 0;
		}
	}
	if (k > 0) {
		dump[k++] = '\r';
		dump[k++] = '\n';
		dump[k] = '\0';
		putcs(stdout_putp, stdout_putf, dump);
	}
	if (stdout_stop) {
		stdout_stop(stdout_putp);
	}
}

/** @} */

/** @} (end addtogroup PRINTF) */
