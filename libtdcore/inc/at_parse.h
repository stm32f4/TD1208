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

#ifndef __AT_PARSE_H
#define __AT_PARSE_H

#include <stdint.h>
#include <stdbool.h>

#include "td_module.h"

#ifdef __cplusplus
extern "C" {
#endif

	/***************************************************************************//**
	 * @addtogroup AT_PARSER AT Command Parser
	 * @brief AT parser API for the TDxxxx RF modules
	 * @{
	 ******************************************************************************/

	/*******************************************************************************
	 *************************   DEFINES   *****************************************
	 ******************************************************************************/

	/** @addtogroup AT_DEFINES Defines
	 * @{ */

#ifndef USE_PRINTF

	/** Flag to force the use of printf */
#define USE_PRINTF
#endif

	/** AT command buffer size */
#define AT_BUFFER_SIZE	75

	/** Maximum number of AT command arguments */
#define AT_MAX_ARGS		9

#ifdef EFM32TG210F32
	/** AT persist buffer size */
#define AT_PERSIST_SIZE	32
#else
	/** AT persist buffer size */
#define AT_PERSIST_SIZE	192
#endif

	/** @} */

	/*******************************************************************************
	 ***********************   ENUMERATIONS   **************************************
	 ******************************************************************************/

	/** @addtogroup AT_ENUMERATIONS Enumerations
	 * @{ */

	/** AT parser states */
	typedef enum AT_states {
		AT_A = 0,				///< Got an 'A'
		AT_AT,					///< Got 'AT'
		AT_COMMAND				///< Got a command
	}
	AT_states;

	/** Basic AT command tokens */
	typedef enum AT_tokens {
		AT_PARSE = 0,			///< Continue parsing next character
		AT_UNKNOWN,				///< Unknown token
		AT_ESCAPE,				///< Ignore token

		// Below here, please try to keep the enum in alphabetical order!!!
		AT,						///< AT
		AT_DISPLAY_CONFIG,		///< AT&V
		AT_FACTORY,				///< AT&F
		AT_GET_BANNER,			///< AT$S200?
		AT_HARDWARE_REV,		///< ATI11
		AT_HELP,				///< AT?
		AT_MANUFACTURER,		///< ATI0
		AT_PRODUCT_REV,			///< ATI12
		AT_QUERY_BANNER,		///< AT$S200=?
		AT_RELEASE_DATE,		///< ATI5
		AT_RESET,				///< ATZ
		AT_SERIAL_NUMBER,		///< ATI7
		AT_SET_BANNER,			///< AT$S200=<value>
		AT_SET_ECHO,			///< ATE<value>
		AT_SET_EXTENDED_RESULTS, ///< ATX<value>
		AT_SET_QUIET,			///< ATQ<value>
		AT_SET_VERBOSITY,		///< ATV<value>
		AT_SOFTWARE_REV,		///< ATI13
		AT_UNIQUE_ID,			///< ATI10
		AT_WRITE_CONFIG,		///< AT&W
#if defined(__ICCARM__) || defined(__GNUC__)
		AT_FREE_STACK,			///< AT$FS
#endif
		AT_BASE_LAST			///< last basic token
	} AT_tokens;

	/** AT command result codes */
	typedef enum AT_results {
		AT_NOTHING = -1,		///< Do not output a result
		AT_OK = 0,				///< 'OK' result
		AT_ERROR = 4,			///< 'ERROR' result
	} AT_results;

	/** @} */

	/*******************************************************************************
	 *************************   TYPEDEFS   ****************************************
	 ******************************************************************************/

	/** @addtogroup AT_TYPEDEFS Typedefs
	 * @{ */

	/** AT command structure
	 *
	 * Basically, a string LUT with corresponding token.
	 *
	 * Both members are set to NULL to flag the last command.
	 */
	typedef struct {
		char *ascii;				///< The ASCII command representation
		uint8_t token;				///< The corresponding command token
	} AT_command_t;

	/**
	 * @brief AT parser extension structure
	 *
	 * @details
	 *   #Introduction
	 *
	 *   The AT parser extension provides a plugin mechanism that allows to extend
	 *   the set of available AT commands.
	 *
	 *   #Implementation
	 *
	 *   The At parser extension consists in a structure that contains:
	 *    - a table with the additional AT commands supported by the extension in
	 *    ASCII with the corresponding token
	 *    - some functions that will be called back for some specific purpose: during
	 *    initialization, while display help or status, when parsing a new character,
	 *    when a command has been recognized or when the system needs to perform
	 *    a persistent write of its configuration or when it has to read it back
	 *
	 *    #Calling Sequence
	 *
	 *    The only required operation dealing with an AT extension is to register it
	 *    using AT_AddExtension().
	 */
	typedef struct {
		AT_command_t const *commands;	///< Pointer to the list of extension commands
		void (*init)(void);				///< Pointer to the extension initialization function
		void (*help)(void);				///< Pointer to the extension help display function
		void (*status)(bool show);		///< Pointer to the extension status display function
		int8_t (*parse)(uint8_t);		///< Pointer to the extension parse function
		uint8_t (*tokenize)(char);		///< Pointer to the extension tokenization function
		uint8_t (*persist)(bool write, uint8_t *buffer, uint8_t count);	///< Pointer to the extension persistence function
	} AT_extension_t;

	/** @} */

	/*******************************************************************************
	 *************************  CONSTANTS  *****************************************
	 ******************************************************************************/

	/** @addtogroup AT_CONSTANTS Constants
	 * @{ */

	/** @} */

	/*******************************************************************************
	 *************************   PROTOTYPES   **************************************
	 ******************************************************************************/

	/** @addtogroup AT_USER_FUNCTIONS User Functions
	 * @{ */
	/** @addtogroup AT_PROTOTYPES Prototypes
	 * @{ */

	uint8_t AT_Tokenize(char c, int *extension);
	void AT_printf(char *fmt, ...);
	void AT_PrintResult(int8_t result);
	bool AT_AddExtension(AT_extension_t *extension);
	void AT_Init(void);
	void AT_Parse(char c);
	long long AT_atoll(const char *s);

	/** @} */
	/** @} */

	/*******************************************************************************
	 **************************   PUBLIC VARIABLES   *******************************
	 ******************************************************************************/

	/** @addtogroup AT_GLOBAL_VARIABLES Global Variables
	 * @{ */
	/** @addtogroup AT_EXTERN External Declarations
	 * @{ */

	extern char AT_buffer[AT_BUFFER_SIZE];
	extern char AT_previousCommand[AT_BUFFER_SIZE];
	extern char *AT_argv[AT_MAX_ARGS];
	extern int AT_argc;
	extern char *AT_last;
	extern bool AT_verbose;
	extern bool AT_extended;
	extern bool AT_echo;
	extern bool AT_quietResult;

	/** @} */
	/** @} */

	/** @} (end addtogroup AT_PARSER) */

#ifdef __cplusplus
}
#endif

#endif // __AT_PARSE_H
