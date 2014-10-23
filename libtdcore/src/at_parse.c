/***************************************************************************//**
 * @file
 * @brief AT parser API for the TDxxxx RF modules.
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
#include <stdarg.h>

#include "at_parse.h"
#include "td_printf.h"
#include "td_flash.h"
#include "td_rtc.h"

/***************************************************************************//**
 * @addtogroup AT_PARSER AT Command Parser
 * @brief AT parser API for the TDxxxx RF modules
 * @{
 * @details
 *   # Introduction
 *
 *   The AT command parser provides a [Hayes command set](http://en.wikipedia.org/wiki/Hayes_command_set)
 *   compatible command parser. Originally established by Hayes for its
 *   SmartModem 300 modem in 1981, this command set has  been further extended by
 *   other manufacturers and standardized by the ITU-T as "Recommendation V.250"
 *   and the 3GPP Consortium as "ETSI GSM 07.07 (3GPP TS 27.007)".
 *
 *   # Implementation
 *
 *   This implementation provides an extensible AT command parser that performs
 *   a lexical analysis of a character stream, followed by a grammatical analysis
 *   into command and parameter tokens, that are used to call pieces of code in
 *   charge of handling the corresponding command and returning both a character
 *   stream and a result code.
 *
 *   This AT command parser is independent of the input and output devices used
 *   to read and write the character streams, as it receives the input characters
 *   using its AT_Parse() function, and calls the tfp_printf() for outputting
 *   characters.
 *
 *   # Calling Sequence
 *
 *   Thus, the normal sequence of calls to use the AT command parser is first to
 *   initialize the tfp_printf() module to output characters to the desired
 *   device, then call the AT_Init() function, and then call repeatedly the
 *   AT_Parse() function with the characters to interpret.
 *
 *   # Interpreter Extensions
 *
 *   This AT command interpreter supports extensions to the basic command set by
 *   the mean of the *AT Extensions*, effectively allowing a user program to add
 *   custom commands. See the AT_extension_t documentation for further details.
 *
 ******************************************************************************/

/*******************************************************************************
 *************************   DEFINES   *****************************************
 ******************************************************************************/

/** @addtogroup AT_DEFINES Defines
 * @{ */

/** Maximum number of AT extensions */
#define AT_EXTENSION_NUMBER 9

#ifndef AT_FORCE_BANNER
/** Flag to force startup banner */
#define AT_FORCE_BANNER	0
#endif

/** @} */

/*******************************************************************************
 *************************  CONSTANTS  *****************************************
 ******************************************************************************/

/** @addtogroup AT_CONSTANTS Constants
 * @{ */

/** Basic AT command set */
static AT_command_t const AT_commands[] = {
	{"AT&F", AT_FACTORY},
	{"AT&V", AT_DISPLAY_CONFIG},
	{"AT&W", AT_WRITE_CONFIG},
	{"AT?", AT_HELP},
	{"ATE", AT_SET_ECHO},
	{"ATI0", AT_MANUFACTURER},
	{"ATI11", AT_HARDWARE_REV},
	{"ATI12", AT_PRODUCT_REV},
	{"ATI13", AT_SOFTWARE_REV},
#if defined(__ICCARM__) || defined(__GNUC__)
	{"ATI14", AT_FREE_STACK},
#endif
	{"ATI5", AT_RELEASE_DATE},
	{"ATI7", AT_SERIAL_NUMBER},
	{"ATI10", AT_UNIQUE_ID},
	{"ATI", AT_MANUFACTURER},
	{"ATQ", AT_SET_QUIET},
	{"ATS200=?", AT_QUERY_BANNER},
	{"ATS200=", AT_SET_BANNER},
	{"ATS200?", AT_GET_BANNER},
	{"ATV", AT_SET_VERBOSITY},
	{"ATX", AT_SET_EXTENDED_RESULTS},
	{"ATZ", AT_RESET},
	{"AT", AT},
	{0, 0}
};

/** Basic AT help string */
static char const AT_help[] = {
	"----------------\r\n"
	"E    => Echo\r\n"
	"I    => Information\r\n"
	"Q    => Quiet\r\n"
	"V    => Verbose\r\n"
	"X    => Extended result code\r\n"
	"Z    => Reset\r\n"
	"?    => Help\r\n"
	"&F   => Factory\r\n"
	"&V   => Status\r\n"
	"&W   => Save\r\n"
	"S200 => Reboot banner display\r\n"
};

/** @} */

/*******************************************************************************
 *************************   PUBLIC VARIABLES   ********************************
 ******************************************************************************/

/** @addtogroup AT_GLOBAL_VARIABLES Global Variables
 * @{ */

/** AT parser extensions */
AT_extension_t *AT_extension[AT_EXTENSION_NUMBER] = {0};

/** AT command buffer */
char AT_buffer[AT_BUFFER_SIZE] = "";

/** Previous AT command */
char AT_previousCommand[AT_BUFFER_SIZE] = "";

/** AT command argument array */
char *AT_argv[AT_MAX_ARGS];

/** Number of AT command arguments */
int AT_argc = 0;

/** Pointer to last character in AT command buffer */
char *AT_last = &AT_buffer[0];

/** AT persist buffer */
uint8_t AT_persist_buffer[AT_PERSIST_SIZE];

/** AT parser state */
AT_states AT_state = AT_A;

/** AT parser verbosity flag */
bool AT_verbose = true;

/** AT parser extended command result flag (print baudrate in connect string) */
bool AT_extended = true;

/** AT parser command echo flag */
bool AT_echo = true;

/** AT parser quiet result flag */
bool AT_quietResult = false;

/** AT parser reboot banner display flag */
bool AT_banner = AT_FORCE_BANNER ? true : false;

/** @} */

/*******************************************************************************
 **************************   PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup AT_LOCAL_FUNCTIONS Local Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Save Persistent Buffer
 ******************************************************************************/
static void AT_SavePersistBuffer(void)
{
	int extension;
	uint8_t persist_size = 1, extension_write, *persist_pointer;

	persist_pointer = AT_persist_buffer;
	*persist_pointer = (AT_echo == true) ? 0x01 : 0x00;
	*persist_pointer |= (AT_verbose == true) ? 0x02 : 0x00;
	*persist_pointer |= (AT_quietResult == true) ? 0x04 : 0x00;
	*persist_pointer |= (AT_extended == true) ? 0x08 : 0x00;
	*persist_pointer |= (AT_banner == true) ? 0x10 : 0x00;
	persist_pointer++;
	for (extension = 0; extension < AT_EXTENSION_NUMBER; extension++) {
		if (AT_extension[extension] == 0) {
			break;
		}
		if (AT_extension[extension]->persist != 0) {
			persist_size += AT_extension[extension]->persist(false, 0, 0);
		}
	}
	for (extension = 0; extension < AT_EXTENSION_NUMBER; extension++) {
		if (AT_extension[extension] == 0) {
			break;
		}
		if (AT_extension[extension]->persist != 0) {
			extension_write = AT_extension[extension]->persist(true, persist_pointer, persist_size);
			persist_pointer += extension_write;
			persist_size -= extension_write;
		}
	}
}

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup AT_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Tokenize a character for Hayes AT commands.
 *
 * @details
 *   This function implements the AT lexical analyzer by transforming the
 *   received characters into recognized tokens.
 *
 * @param[in] c
 *   The character to parse.
 *
 * @param[out] extension
 *   Pointer to the AT extension number that will be filled in with the
 *   extension that implement the recognized token.
 *
 * @return
 *   Returns:
 *   - the command token if found
 *   - AT_PARSE to continue parsing
 *   - AT_UNKNOWN if no corresponding command has been found
 ******************************************************************************/
uint8_t AT_Tokenize(char c, int *extension)
{
	char *s, *d;
	const AT_command_t *command;

	if (c == '/' && AT_last == &AT_buffer[1] && AT_buffer[0] == 'A') {

		// Replay last command
		for (s = AT_previousCommand, d = AT_buffer; *s;) {
			*d++ = *s++;
		}
		*d = '\0';
		c = '\r';
		AT_last = AT_buffer;
		AT_state = AT_A;
	} else {
		if (c >= 'a' && c <= 'z') {

			// Convert lower-case to upper-case
			c -= 'a' - 'A';
		}
		if (AT_last >= &AT_buffer[AT_BUFFER_SIZE]) {

			// AT command buffer overflow
			goto error;
		} else {
			switch (AT_state) {
			case AT_A:
				if (c != 'A') {
					return AT_PARSE;
				}
				AT_state = AT_AT;
				*AT_last++ = 'A';
				*AT_last = '\0';
				return AT_PARSE;

			case AT_AT:
				if (c != 'T') {
					AT_state = AT_A;
					AT_last = &AT_buffer[0];
					return AT_PARSE;
				}
				AT_state = AT_COMMAND;
				*AT_last++ = 'T';
				*AT_last = '\0';
				return AT_PARSE;
				break;

			case AT_COMMAND:
				if (c == '\r' || c == '\n') {

					// Command terminator, save command (if any) for replay
					for (s = AT_buffer, d = AT_previousCommand; s <= AT_last;) {
						*d++ = *s++;
					}
					*d = '\0';
					c = '\r';
					AT_last = AT_buffer;
					AT_state = AT_A;
					break;
				} else if (c == '\b' || c == 0x7f) {
					if (AT_last > &AT_buffer[2]) {

						// Backspace one character
						*(--AT_last) = '\0';
					}
					return AT_PARSE;
				} else if (c == '\0' || c == 0x18) {

					// Cancel command
					if (AT_echo) {
						for (; AT_last != AT_buffer; AT_last--) {
							tfp_printf("\b \b");
						}
					}
					AT_buffer[0] = '\0';
					c = '\0';
					AT_state = AT_A;
					return AT_PARSE;
				} else {
					*AT_last++ = c;
					*AT_last = '\0';
					return AT_PARSE;
				}
				break;

			default:
				AT_state = AT_A;
				return AT_PARSE;
			}
		}
	}

	// Command lookup
	for (*extension = 0; *extension <= AT_EXTENSION_NUMBER; (*extension)++) {
		if (*extension == AT_EXTENSION_NUMBER || AT_extension[*extension] == 0) {
			command = AT_commands;
		} else if (AT_extension[*extension]->commands != 0) {
			command = AT_extension[*extension]->commands;
		} else {
			continue;
		}
		for (; command->token; command++) {
			for (s = command->ascii, d = AT_buffer; *s; s++, d++) {
				if (*s != *d) {
					break;
				}
			}
			if (*s == '\0') {
				if (command->token == AT && AT_buffer[2] != '\0') {

					// Matched "AT", but have some extra characters behind => unknown command
					break;
				}

				// Found a matching command, parse the arguments (if any)
				AT_argc = 0;
				while (*d != '\0') {
					if (AT_argc < AT_MAX_ARGS) {
						AT_argv[AT_argc++] = d;
					} else {

						// Argument count overflow
						goto error;
					}
					while (*d != '\0' && *d != ',') {
						d++;
					}
					if (*d == ',') {
						*d++ = '\0';
					}
				}
				return command->token;
			}
		}
	}

	// No matching command found
error:
	AT_last = AT_buffer;
	return AT_UNKNOWN;
}

/***************************************************************************//**
 * @brief
 *   Output an AT string according to a format.
 *
 * @note
 *   This function pre-pend the required line termination depending on the AT
 *   parser verbosity flag, and should be called for each new string output
 *   sequence.
 *   Each subsequent string output should then use tfp_printf().
 *
 * @param[in] fmt
 *   Pointer to format string in printf format.
 *
 * @param[in] ...
 *   Number and types of arguments according to the format string.
 ******************************************************************************/
void AT_printf(char *fmt, ...)
{
	va_list va;

	va_start(va, fmt);
	if (AT_verbose == true) {
		tfp_printf("\r\n");
	}
	tfp_vprintf(fmt, va);
	va_end(va);
}

/***************************************************************************//**
 * @brief
 *   Output the #AT_results of an AT command.
 *
 * @param[in] result
 *   The AT_results_t result to output.
 ******************************************************************************/
void AT_PrintResult(int8_t result)
{
	if (result != AT_NOTHING) {
		AT_last = AT_buffer;
		AT_buffer[0] = '\0';
	}
	if (AT_quietResult == true) {
		return;
	}
	switch (result) {
	case AT_NOTHING:
		break;

	case AT_OK:
		if (AT_verbose) {
			AT_printf("OK\r\n");
		} else {
			AT_printf("0\r");
		}
		break;

	case AT_ERROR:
		if (AT_verbose) {
			AT_printf("ERROR\r\n");
		} else {
			AT_printf("4\r");
		}
		break;

	default:
		break;
	}
}

/***************************************************************************//**
 * @brief
 *   Add an AT extension to the parser.
 *
 * @param[in] extension
 *   Pointer to the AT_extension_t structure to add.
 ******************************************************************************/
bool AT_AddExtension(AT_extension_t *extension)
{
	int i;

	for (i = 0; i < AT_EXTENSION_NUMBER; i++) {
		if (AT_extension[i] == 0) {
			AT_extension[i] = extension;
			if (i < AT_EXTENSION_NUMBER - 1) {
				AT_extension[i + 1] = 0;
			}
			return true;
		}
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *   Initialize the AT command parser.
 ******************************************************************************/
void AT_Init(void)
{
	int i;
	uint8_t extension_read, *persist_pointer;
	uint16_t persist_size = 1;

	for (i = 0; i < AT_BUFFER_SIZE; i++) {
		AT_buffer[i] = '\0';
	}
	for (i = 0; i < AT_BUFFER_SIZE; i++) {
		AT_previousCommand[i] = '\0';
	}
	for (i = 0; i < AT_MAX_ARGS; i++) {
		AT_argv[i] = 0;
	}
	AT_argc = 0;
	AT_last = &AT_buffer[0];
	AT_state = AT_A;
	AT_verbose = true;
	AT_extended = true;
	AT_echo = true;
	AT_quietResult = false;
	AT_banner = AT_FORCE_BANNER ? true : false;

	// Find out how many persistent bytes are required by extensions
	for (i = 0; i < AT_EXTENSION_NUMBER; i++) {
		if (AT_extension[i] == 0) {
			break;
		}
		if (AT_extension[i]->init != 0) {
			AT_extension[i]->init();
		}
		if (AT_extension[i]->persist != 0) {
			persist_size += AT_extension[i]->persist(false, 0, 0);
		}
	}
	if (persist_size > AT_PERSIST_SIZE) {
		persist_size = AT_PERSIST_SIZE;
	}
	/* Read persist data. If not available, create them. After that AT_persist_buffer is correctly filled */
	if (!TD_FLASH_DeclareVariable((uint8_t *) AT_persist_buffer, AT_PERSIST_SIZE, 0)) {
		AT_SavePersistBuffer();
	}

	// Read persistent data
	persist_pointer = AT_persist_buffer;
	AT_echo = (*persist_pointer & 0x01) ? true : false;
	AT_verbose = (*persist_pointer & 0x02) ? true : false;
	AT_quietResult = (*persist_pointer & 0x04) ? true : false;
	AT_extended = (*persist_pointer & 0x08) ? true : false;
	AT_banner  = (*persist_pointer & 0x10) ? true : false;
	persist_pointer++;

	// Pass persistent data to each extension
	for (i = 0; i < AT_EXTENSION_NUMBER; i++) {
		if (AT_extension[i] == 0) {
			break;
		}
		if (AT_extension[i]->persist != 0) {
			extension_read = AT_extension[i]->persist(false, persist_pointer, persist_size);
			persist_pointer += extension_read;
			persist_size -= extension_read;
		}
	}

	if (AT_verbose == true && AT_quietResult == false && AT_banner == true) {
		AT_printf("^SYSSTART\r\n");
	}
}

/***************************************************************************//**
 * @brief
 *   AT command parser.
 *
 * @details
 *   This function is the main AT parser function to call when a new input
 *   character is received from the main idle loop.
 *
 *   It will perform lexical analysis, argument collection and call the piece of
 *   code in charge of handling the corresponding AT command.
 *
 * @param[in] c
 *   The new character to parse.
 ******************************************************************************/
void AT_Parse(char c)
{
	uint32_t x, y;
	int extension, i;
	char td_serial[13];

#if defined(__ICCARM__)
	extern unsigned char CSTACK$$Base;
	extern unsigned char CSTACK$$Limit;
	unsigned char *memptr;
#elif defined(__GNUC__)
	extern unsigned char __end;
	extern unsigned char __cs3_region_end_ram;
	unsigned char *memptr;
#endif

	uint8_t token = AT_PARSE;
	int8_t result = AT_OK, extension_result = AT_PARSE;
	uint8_t echo, verbosity, quiet_result, extended_result, banner;
	TD_DEVICE device = {0, 0, 0, 0, 0};
	TD_DEVICE_EXT device_ext = {
		1,
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	};

	for (extension = 0; extension < AT_EXTENSION_NUMBER; extension++) {
		if (AT_extension[extension] == 0) {
			break;
		}
		if (AT_extension[extension]->tokenize != 0) {
			token = AT_extension[extension]->tokenize(c);
			if (token == AT_ESCAPE) {
				return;
			}
			if (token > AT_BASE_LAST) {
				break;
			}
		}
	}
	if (AT_echo) {
		switch (AT_state) {
		case AT_A:
			if (c == 'A' || c == 'a') {
				tfp_printf("%c", c);
			}
			break;

		case AT_AT:
			if (c == 'T' || c == 't' || c == '/') {
				tfp_printf("%c", c);
			} else {
				tfp_printf("\b \b");
			}
			break;

		default:
			if (c == '\b' || c == 0x7f) {
				if (AT_last > &AT_buffer[2]) {
					tfp_printf("\b \b");
				}
			} else if (c == '\r' || c == '\n') {
				tfp_printf("\r");
			} else {
				tfp_printf("%c", c);
			}
			break;
		}
	}
	if (token == AT_PARSE) {
		token = AT_Tokenize(c, &extension);
	}
	if (token > AT_BASE_LAST && extension < AT_EXTENSION_NUMBER && AT_extension[extension]->parse != 0) {
		extension_result = AT_extension[extension]->parse(token);
		if (extension_result != AT_NOTHING) {
			AT_PrintResult(extension_result);
		}
		return;
	}
	switch (token) {
	case AT_PARSE:
		result = AT_NOTHING;
		break;

	case AT_UNKNOWN:
		result = AT_ERROR;
		break;

	case AT:
		break;

	case AT_DISPLAY_CONFIG:
		for (extension = 0; extension < AT_EXTENSION_NUMBER; extension++) {
			if (AT_extension[extension] == 0) {
				break;
			}
			if (AT_extension[extension]->status != 0) {
				AT_extension[extension]->status(false);
			}
		}
		AT_printf("%s\r\n", CONFIG_MANUFACTURER);
		tfp_printf("Hardware Version: %s\r\n", CONFIG_HARDWARE_VERSION);
		tfp_printf("Software Version: %s\r\n", CONFIG_SOFTWARE_VERSION);
		TD_FLASH_DeviceRead(&device);
		tfp_printf("S/N: %08X\r\n", device.Serial);
		if (TD_FLASH_DeviceReadExtended(&device, &device_ext) == true) {
			for (i = 0; i < 12; i++) {
				td_serial[i] = device_ext.TDSerial[i];
			}
			td_serial[12] = '\0';
			if (td_serial[0] != '?') {
				tfp_printf("TDID: %12s\r\n", td_serial);
			}
		}
		tfp_printf("ACTIVE PROFILE\r\n");
		tfp_printf("E%d V%d Q%d",
				   AT_echo,
				   AT_verbose,
				   AT_quietResult);
		tfp_printf(" X%d S200:%d",
				   AT_extended,
				   AT_banner
				  );
		for (extension = 0; extension < AT_EXTENSION_NUMBER; extension++) {
			if (AT_extension[extension] == 0) {
				break;
			}
			if (AT_extension[extension]->status != 0) {
				AT_extension[extension]->status(true);
			}
		}
		tfp_printf("\r\n");
		break;

	case AT_FACTORY:
		AT_verbose = true;
		AT_extended = true;
		AT_echo = true;
		AT_quietResult = false;
		AT_banner = AT_FORCE_BANNER ? true : false;
		for (extension = 0; extension < AT_EXTENSION_NUMBER; extension++) {
			if (AT_extension[extension] == 0) {
				break;
			}
			if (AT_extension[extension]->init != 0) {
				AT_extension[extension]->init();
			}
		}
		break;

	case AT_GET_BANNER:
		if (AT_argc == 0) {
			AT_printf("%d\r\n", AT_banner);
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_HARDWARE_REV:
		if (AT_argc == 0) {
			AT_printf("%s\r\n", CONFIG_HARDWARE_VERSION);
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_HELP:
		if (AT_argc == 0) {
			AT_printf("%s", AT_help);
			for (extension = 0; extension < AT_EXTENSION_NUMBER; extension++) {
				if (AT_extension[extension] == 0) {
					break;
				}
				if (AT_extension[extension]->help != 0) {
					AT_extension[extension]->help();
				}
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_MANUFACTURER:
		if (AT_argc == 0) {
			AT_printf("%s\r\n", CONFIG_MANUFACTURER);
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_PRODUCT_REV:
		if (AT_argc == 0) {
			if (TD_FLASH_DeviceRead(&device)) {
				AT_printf("%02X\r\n", device.ProdResult);
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_QUERY_BANNER:
		if (AT_argc != 0) {
			result = AT_ERROR;
		} else {
			AT_printf("0..1\r\n");
		}
		break;

	case AT_RELEASE_DATE:
		if (AT_argc == 0) {
			AT_printf("%s\r\n", CONFIG_RELEASE_DATE);
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_RESET:
		if (AT_argc == 0) {
			AT_PrintResult(result);
			TD_RTC_Delay(T100MS);
			NVIC_SystemReset();
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SERIAL_NUMBER:
		if (AT_argc == 0) {
			TD_FLASH_DeviceRead(&device);
			AT_printf("%04X\r\n", device.Serial);
			if (TD_FLASH_DeviceReadExtended(&device, &device_ext) == true) {
				for (i = 0; i < 12; i++) {
					td_serial[i] = device_ext.TDSerial[i];
				}
				td_serial[12] = '\0';
				tfp_printf("TDID: %12s\r\n", td_serial);
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SET_BANNER:
		if (AT_argc == 1) {
			banner = AT_atoll(AT_argv[0]);
			if (banner > 1) {
				result = AT_ERROR;
			} else {
				AT_banner = banner;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SET_ECHO:
		if (AT_argc == 0) {
			AT_echo = false;
		} else if (AT_argc == 1) {
			echo = AT_atoll(AT_argv[0]);
			if (echo < 2) {
				AT_echo = echo ? true : false;
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SET_EXTENDED_RESULTS:
		if (AT_argc == 0) {
			AT_extended = true;
		} else if (AT_argc == 1) {
			extended_result = AT_atoll(AT_argv[0]);
			if (extended_result < 2) {
				AT_extended =  extended_result ? true : false;
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SET_QUIET:
		if (AT_argc == 0) {
			AT_quietResult = true;
		} else if (AT_argc == 1) {
			quiet_result = AT_atoll(AT_argv[0]);
			if (quiet_result < 2) {
				AT_quietResult = quiet_result ? true : false;
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SET_VERBOSITY:
		if (AT_argc == 0) {
			AT_verbose =  true;
		} else if (AT_argc == 1) {
			verbosity = AT_atoll(AT_argv[0]);
			if (verbosity < 2) {
				AT_verbose = verbosity ? true : false;
			} else {
				result = AT_ERROR;
			}
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_SOFTWARE_REV:
		if (AT_argc == 0) {
			AT_printf("%s\r\n", CONFIG_SOFTWARE_VERSION);
		} else {
			result = AT_ERROR;
		}
		break;

#if defined(__ICCARM__)
	case AT_FREE_STACK:
		memptr = &CSTACK$$Base;
		while (memptr < &CSTACK$$Limit) {
			if (*memptr++ != 0xCD) {
				break;
			}
		}
		AT_printf("Free Stack: %d bytes", memptr - &CSTACK$$Base);
		break;
#elif defined(__GNUC__)
	case AT_FREE_STACK:
		memptr = &__end;
		while (memptr < &__cs3_region_end_ram) {
			if (*memptr++ != 0xCD) {
				break;
			}
		}
		AT_printf("Free Stack: %d bytes", memptr - &__end);
		break;
#endif

	case AT_UNIQUE_ID:
		if (AT_argc == 0) {
			x = DEVINFO->UNIQUEH;
			y = DEVINFO->UNIQUEL;
			AT_printf("%08X%08X\r\n", x, y);
		} else {
			result = AT_ERROR;
		}
		break;

	case AT_WRITE_CONFIG:
		if (AT_argc == 0) {
			AT_SavePersistBuffer();
			TD_FLASH_WriteVariables();
		} else {
			result = AT_ERROR;
		}
		break;

	default:
		result = AT_ERROR;
		break;
	}
	AT_PrintResult(result);
}

/***************************************************************************//**
 * @brief
 *   Optimized ASCII to long long integer conversion function.
 *
 * @note
 *   This function also accepts hex and optionally octal numbers.
 *
 * @param[in] s
 *   Pointer to the string to convert to number.
 *
 * @return
 *   Returns the converted value.
 ******************************************************************************/
long long AT_atoll(const char *s)
{
	long long value = 0;
	bool sign = false;
	char c = *s++;

	while (c == ' ' || c == '\t') {
		c = *s++;
	}
	if (c == '-') {
		sign = true;
		c = *s++;
	} else if (c == '+') {
		c = *s++;
	} else if (c == '0') {
		c = *s++;
		if (c == 'x' || c == 'X') {
			c = *s++;
			while (1) {
				if (c >= '0' && c <= '9') {
					value = (value << 4) + (c - '0');
				} else if (c >= 'A' && c <= 'F') {
					value = (value << 4) + (c - 'A' + 10);
				} else if (c >= 'a' && c <= 'f') {
					value = (value << 4) + (c - 'a'  + 10);
				} else {
					return value;
				}
				c = *s++;
			}
		}

#ifdef AT_OCTAL
		else {
			while (1) {
				if (c >= '0' && c <= '7') {
					value = (value << 3) + (c - '0');
				} else {
					return value;
				}
				c = *s++;
			}
		}
#endif

	}
	while (1) {
		if (c >= '0' && c <= '9') {
			value = (value * 10) + (c - '0');
		} else {
			break;
		}
		c = *s++;
	}
	return sign == true ? -value : value;
}

/** @} */

/** @} (end addtogroup AT_PARSER) */
