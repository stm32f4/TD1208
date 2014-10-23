/**
 * @file
 * @brief printf utility  for the TDxxxx RF modules.
 * @author Kustaa Nyholm
 * @version 2.0.2
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2004  Kustaa Nyholm</b>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * @section Usage
 * This library is realy just two files: 'printf.h' and 'printf.c'.
 * They provide a simple and small (+200 loc) printf functionality to
 * be used in embedded systems.
 *
 * I've found them so useful in debugging that I do not bother with a
 * debugger at all.
 *
 * They are distributed in source form, so to use them, just compile them
 * into your project.
 *
 * Two printf variants are provided: printf and sprintf.
 *
 * The formats supported by this implementation are: 'd' 'u' 'c' 's' 'x' 'X'.
 *
 * Zero padding and field width are also supported.
 *
 * If the library is compiled with 'PRINTF_SUPPORT_LONG' defined then the
 * long specifier is also
 * supported. Note that this will pull in some long math routines (pun intended!)
 * and thus make your executable noticeably longer.
 *
 * The memory foot print of course depends on the target CPU, compiler and
 * compiler options, but a rough estimate (based on a H8S target) is about
 * 1.4 kB for code and some twenty 'int's and 'char's, say 60 bytes of stack space.
 * Not too bad. Your mileage may vary. By hacking the source code you can
 * get rid of some hundred bytes, I'm sure, but personally I feel the balance of
 * functionality and flexibility versus  code size is close to optimal for
 * many embedded systems.
 *
 * To use the printf you need to supply your own character output function,
 * something like :
 *
 * @code
 * void putc ( void* p, char c)
 * {
 * while (!SERIAL_PORT_EMPTY) ;
 * SERIAL_PORT_TX_REGISTER = c;
 * }
 * @endcode
 *
 * Before you can call printf you need to initialize it to use your
 * character output function with something like:
 *
 * @code
 * init_printf(NULL,putc);
 * @endcode
 *
 * Notice the 'NULL' in 'init_printf' and the parameter 'void* p' in 'putc',
 * the NULL (or any pointer) you pass into the 'init_printf' will eventually be
 * passed to your 'putc' routine. This allows you to pass some storage space (or
 * anything realy) to the character output function, if necessary.
 * This is not often needed but it was implemented like that because it made
 * implementing the sprintf function so neat (look at the source code).
 *
 * The code is re-entrant, except for the 'init_printf' function, so it
 * is safe to call it from interupts too, although this may result in mixed output.
 * If you rely on re-entrancy, take care that your 'putc' function is re-entrant!
 *
 * The printf and sprintf functions are actually macros that translate to
 * 'tfp_printf' and 'tfp_sprintf'. This makes it possible
 * to use them along with 'stdio.h' printf's in a single source file.
 * You just need to undef the names before you include the 'stdio.h'.
 * Note that these are not function like macros, so if you have variables
 * or struct members with these names, things will explode in your face.
 * Without variadic macros this is the best we can do to wrap these
 * function. If it is a problem just give up the macros and use the
 * functions directly or rename them.
 *
 * For further details see source code.
 *
 * regs Kusti, 23.10.2004
 */

#ifndef __TD_PRINTF
#define __TD_PRINTF

#ifndef USE_PRINTF

/** sprintf() conditional macro */
#define SPRINTF(i,...)

/** printf() conditional macro */
#define PRINTF(i,...)

/** dump() conditional macro */
#define DUMP(i,...)

#else // USE_PRINTF

#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup PRINTF
	 * @brief printf utility  for the TDxxxx RF modules
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 *************************   DEFINES   *****************************************
	 ******************************************************************************/

	/** @addtogroup PRINTF_DEFINES Defines
	 * @{ */

	/** Alias for tfp_printf() */
#define printf			tfp_printf

	/** Alias for tfp_sprintf() */
#define sprintf			tfp_sprintf

	/** Conditional alias for tfp_printf() */
#define PRINTF(i,...)	tfp_printf(i, ##__VA_ARGS__)

	/** Conditional alias for tfp_sprintf() */
#define SPRINTF(i,...)	tfp_sprintf(i, ##__VA_ARGS__)

	/** Conditional alias for tfp_dump() */
#define DUMP(p, b, l)	tfp_dump(p, b, l)

	/** @} */

	/*******************************************************************************
	  *************************   PROTOTYPES   **************************************
	  ******************************************************************************/

	/** @addtogroup PRINTF_USER_FUNCTIONS User Functions
	 * @{ */

	void init_printf(void *putp, void (*putf)(void *, char), void (*start)(void *), void (*stop)(void *));
	void tfp_printf(char *fmt, ...);
	void tfp_vprintf(char *fmt, va_list va);
	void tfp_sprintf(char *s, char *fmt, ...);
	void tfp_dump(char *text, unsigned char *s, unsigned char sz);
	void tfp_format(void *putp, void (*putf)(void *, char), char *fmt, va_list va);
	void tfp_printf_dump(char *text, unsigned char sz);

	/** @} */

	/** @} (end addtogroup PRINTF) */

#ifdef __cplusplus
}
#endif

#endif // USE_PRINTF

#endif // __TD_PRINTF
