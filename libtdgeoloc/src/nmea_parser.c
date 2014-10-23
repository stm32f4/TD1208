/***************************************************************************//**
 * @file
 * @brief GPS NMEA parser.
 * @author Telecom Design S.A.
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2013-2014 Telecom Design S.A., http://www.telecomdesign.fr</b>
 ******************************************************************************
 ******************************************************************************
 *
 * This source code is the property of Telecom Design S.A.
 *
 * This copyright notice may not be removed from the source code nor changed.
 *
 ******************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <td_rtc.h>
#include <td_core.h>
#include <td_printf.h>
#include <td_utils.h>

#include "ubx7.h"
#include "nmea_parser.h"

/***************************************************************************//**
 * @addtogroup TD_NMEA NMEA Parser
 * @brief NMEA parser.
 * @{
 ******************************************************************************/

/******************************************************************************
 **************************  DEFINES   ****************************************
 ******************************************************************************/

/** @addtogroup TD_NMEA_DEFINES Defines
 * @{ */

/** Number of entries in the NMEA command table */
#define LAST_COMMAND			(sizeof (NMEACommands) / sizeof (NMEA_command_t))

/** Maximum field length */
#define MAX_FIELD_COUNT			20

/** Maximum data length */
#define MAX_DATA				255

/** Maximum command length (NMEA address) */
#define MAX_COMMAND				8


/** NMEA parser states */
#define NMEA_STATE_SOM			0			///< Search for start of message
#define NMEA_STATE_CMD			1			///< Get command
#define NMEA_STATE_DATA			2			///< Get data
#define NMEA_STATE_CHECKSUM_1	3			///< Get first checksum character
#define NMEA_STATE_CHECKSUM_2	4			///< Get second checksum character

/** Turn on trace mode if tfp_printf not commented */
//#define DEBUG_PARSER
//#define DEBUG_PARSER_INFO

#ifdef DEBUG_PARSER_INFO
#define DEBUG_PRINTF(...) tfp_printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif

/** @} */

/*******************************************************************************
 ***********************   ENUMERATIONS   **************************************
 ******************************************************************************/

/** @addtogroup TD_NMEA_ENUMERATIONS Enumerations
 * @{ */

/** NMEA command tokens */
typedef enum nmea_command_token_t {
	NMEA_UNKNOWN = 0,
	NMEA_GPGGA = 1,
	NMEA_GPRMC = 2,
	NMEA_GPTXT = 3
} nmea_command_token;

/** @} */

/*******************************************************************************
 *************************   TYPEDEFS   ****************************************
 ******************************************************************************/

/** @addtogroup TD_NMEA_TYPEDEFS Typedefs
 * @{ */

/** NMEA command structure */
typedef struct {
	char *ascii;						///< ASCII command representation
	nmea_command_token token;			///< Corresponding command token
	uint8_t length;						///< ASCII command length in characters
	bool display;						///< Display flag
	bool (*process)(char *data);		///< Command process function
} NMEA_command_t;

/** @} */

/*******************************************************************************
 ************************   PROTOTYPES   ********************************
 ******************************************************************************/

static bool TD_NMEA_ProcessGPGGA(char *data);
static bool TD_NMEA_ProcessGPRMC(char *data);
static bool TD_NMEA_ProcessGPTXT(char *data);

/*******************************************************************************
 ************************   PRIVATE VARIABLES   ********************************
 ******************************************************************************/

/** @addtogroup TD_NMEA_LOCAL_VARIABLES Local Variables
 * @{ */

#pragma pack(1)

/** Computed NMEA sentence checksum */
static uint8_t ComputedChecksum = 0;

/** Received NMEA sentence checksum (if exists) */
static uint8_t ReceivedChecksum = 0;

/** Index used for both commands and data */
static int Index = 0;

/** NMEA command buffer */
static char Command[MAX_COMMAND] = "";

/** NMEA data buffer */
static char Data[MAX_DATA] = "";

/** Data Field Index list */
static uint8_t FieldIndexList[MAX_FIELD_COUNT];

/** Data Field Count */
static uint8_t FieldCount = 0;

#pragma pack()

/** NMEA parser state */
static uint8_t NMEAState = NMEA_STATE_SOM;

/** NMEA command table */
static NMEA_command_t NMEACommands[] = {
	{"GPGGA", NMEA_GPGGA, 5, false, TD_NMEA_ProcessGPGGA},
	{"GPRMC", NMEA_GPRMC, 5, false, TD_NMEA_ProcessGPRMC},
	{"GPTXT", NMEA_GPTXT, 5, false, TD_NMEA_ProcessGPTXT},
};

/** Global display flag */
static bool DisplayAll = false;

/** Last NMEA command */
static NMEA_command_t LastCommand = {0, NMEA_UNKNOWN, 0, false, 0};

/** Flag array used during NMEA command parsing */
static bool CommandsElector[LAST_COMMAND];

/** Last NMEA position */
static TD_GEOLOC_Fix_t Fix;

/** NMEA user callback function pointer */
static void (*UpdateCallback)(TD_GEOLOC_Fix_t *Fix) = 0;

/** @} */

/*******************************************************************************
 **************************  PRIVATE FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_NMEA_FUNCTIONS Private Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Reset NMEA command parsing structure.
 ******************************************************************************/
static void TD_NMEA_ResetElector(void)
{
	memset(CommandsElector, true, LAST_COMMAND);
}

/***************************************************************************//**
 * @brief
 *   Process a time information within any NMEA command
 *
 * @param[in] index
 *   Index of time into the global string array Data.
 *
 * @return
 *   Returns true if valid time is found, false otherwise.
 ******************************************************************************/
static bool TD_NMEA_ProcessTime(uint8_t index)
{
	char temp[3];
	char *field = &Data[FieldIndexList[index]];

	if (field[0] != 0) {

		// Hour
		temp[0] = field[0];
		temp[1] = field[1];
		temp[2] = '\0';
		Fix.datetime.hours = atoi(temp);

		// minute
		temp[0] = field[2];
		temp[1] = field[3];
		Fix.datetime.minutes = atoi(temp);

		// Second
		temp[0] = field[4];
		temp[1] = field[5];
		Fix.datetime.seconds = atoi(temp);

		//Milliseconds
		return true;
	} else {
		Fix.datetime.hours = 0xFF;
		Fix.datetime.minutes = 0xFF;
		Fix.datetime.seconds = 0xFF;
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *   Process a date within any NMEA command
 *
 * @param[in] index
 *   Index of date into the global string array Data.
 *
 * @return
 *   Returns true if valid date is found, false otherwise.
 ******************************************************************************/
static bool TD_NMEA_ProcessDate(uint8_t index)
{
	char *field = &Data[FieldIndexList[index]];

	if (field[0] != 0) {
		Fix.datetime.day = (field[0] - '0') * 10 + (field[1] - '0');
		Fix.datetime.month = (field[2] - '0') * 10 + (field[3] - '0');
		Fix.datetime.year = (field[4] - '0') * 10 + (field[5] - '0');
	} else {
		Fix.datetime.day = 0xFF;
		Fix.datetime.month = 0xFF;
		Fix.datetime.year = 0xFF;
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *   Process a position within any NMEA command
 *
 * @param[in] index
 *   Index of position into the global string array Data.
 *
 * @return
 *   Returns true if valid position is found, false otherwise.
 ******************************************************************************/
static bool TD_NMEA_ProcessPosition(uint8_t index)
{
	char *field_pos, * field_sign;
	bool update = false;

	field_pos = &Data[FieldIndexList[index]];
	field_sign = &Data[FieldIndexList[index + 1]];

	// Latitude
	if (field_pos[0] != 0 && field_sign[0] != 0) {
		Fix.position.latitude = atolli(field_pos, '.');

		// South latitudes are converted to negative values
		if (field_sign[0] == 'S') {
			Fix.position.latitude = -Fix.position.latitude;
		}
		update = true;
	} else {
		Fix.position.latitude = 0x7FFFFFFF;
	}
	field_pos = &Data[FieldIndexList[index + 2]];
	field_sign = &Data[FieldIndexList[index + 3]];

	// Longitude
	if (field_pos[0] != 0 && field_sign[0] != 0) {
		Fix.position.longitude = atolli(field_pos, '.');

		// West longitude are converted to negative values
		if (field_sign[0] == 'W') {
			Fix.position.longitude = -Fix.position.longitude;
		}
		update = true;
	} else {
		Fix.position.longitude = 0x7FFFFFFF;
	}
	return update;
}

/***************************************************************************//**
 * @brief
 *   Process a GPTXT NMEA command and reset Ublox if dumping errors
 *
 * @param[in] data
 *   Pointer to NMEA string.
 *
 * @return
 *   Returns false.
 ******************************************************************************/
static bool TD_NMEA_ProcessGPTXT(char *data)
{
	char *field;

	if (FieldCount >= 3) {
		field = &Data[FieldIndexList[3]];
		if (field[0] != 0) {
			if (field[0] == 'e' && field[1] == 'x' && field[2] == 'c' && field[3] == 'e') {
				DEBUG_PRINTF("TD_NMEA_ProcessGPTXT UBX7 RESET - ERROR \r\n");
				TD_UBX7_PowerOff();
				TD_RTC_Delay(T1S);
				TD_UBX7_PowerUp(false);
				//TD_UBX7_PollMonExcept();
			}
		}
	} else {
		DEBUG_PRINTF("TD_NMEA_ProcessGPTXT Field count %d - ERROR", FieldCount);
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *   Process a GPGGA NMEA command.
 *
 * @param[in] data
 *   Pointer to NMEA string.
 *
 * @return
 *   Returns the required update mask.
 ******************************************************************************/
static bool TD_NMEA_ProcessGPGGA(char *data)
{
	char *field;
	bool update = false;

	if (FieldCount >= 7) {

		// Time field 0
		if (TD_NMEA_ProcessTime(0)) {
			update = true;
		}

		// Position: field 1,2,3,4
		if (TD_NMEA_ProcessPosition(1)) {
			update = true;
		}

		// Satellites in use
		field = &Data[FieldIndexList[6]];
		if (field[0] != 0) {
			Fix.quality.sat = atoi(field);
			update = true;
		} else {
			Fix.quality.sat = 0xFF;
		}

		// HDOP
		field = &Data[FieldIndexList[7]];
		if (field[0] != 0) {
			Fix.quality.hdop = atolli(field, '.');
			update = true;
		} else {
			Fix.quality.hdop = 9999;
		}

		// Altitude
		field = &Data[FieldIndexList[8]];
		if (field[0] != 0) {
			Fix.position.altitude = atoi(field);
			update = true;
		} else {
			Fix.position.altitude = 0x7FFF;
		}
	} else {
		DEBUG_PRINTF("TD_NMEA_ProcessGPGGA Field count %d - ERROR", FieldCount);
	}
	return update;
}

/***************************************************************************//**
 * @brief
 *   Process a GPRMC NMEA command.
 *
 * @param[in] data
 *   Pointer to NMEA string.
 *
 * @return
 *   Returns the required update mask.
 ******************************************************************************/
static bool TD_NMEA_ProcessGPRMC(char *data)
{
	char *field;
	bool update = false;
	uint32_t speed_2kmh;

	if (FieldCount >= 8) {

		// Time: field 0
		if (TD_NMEA_ProcessTime(0)) {
			update = true;
		}

		// Position: field 2,3,4,5
		if (TD_NMEA_ProcessPosition(2)) {
			update = true;
		}

		// Speed over ground
		field = &Data[FieldIndexList[6]];
		if (field[0] != 0) {
			Fix.speed.speed_knot = atoi(field);

			// 1 knot = 1.852 km/h, but we pre-multiply by 2 to get
			// 1 more bit of accuracy for rounding purposes
			speed_2kmh = (Fix.speed.speed_knot * 2 * 1852) / 1000;

			// Rounding to nearest integer
			if (speed_2kmh & 1) {
				speed_2kmh++;
			}
			Fix.speed.speed_kmh = speed_2kmh >> 1;
			update = true;
		} else {
			Fix.speed.speed_knot = 0xFFFF;
			Fix.speed.speed_kmh = 0xFFFF;
		}

		// Date
		if (TD_NMEA_ProcessDate(8)) {
			update = true;
		}
	} else {
		DEBUG_PRINTF("TD_NMEA_ProcessGPRMC Field count %d - ERROR", FieldCount);
	}
	return update;
}

/***************************************************************************//**
 * @brief
 *   Display a valid NMEA trame.
 *
 * @param[in] command
 *   Pointer to the command ascii representation.
 *
 * @param[in] checksum
 *   Checksum.
 ******************************************************************************/
static void TD_NMEA_DisplayCommand(char *command, uint8_t checksum)
{
	int i;

	tfp_printf("$%s", command);
	for (i = 0; i <= FieldCount; i++) {
		tfp_printf(",");
		tfp_printf(&Data[FieldIndexList[i]]);
	}
	tfp_printf("*%02X\r\n", checksum);
}

/***************************************************************************//**
 * @brief
 *   Process a valid NMEA command.
 *
 * @param[in] command
 *   Pointer to the NMEA command structure.
 *
 * @param[in] data
 *   Pointer to the 0-splitted NMEA data string.
 *
 * @param[in] checksum
 *   Trame checksum.
 ******************************************************************************/
static void TD_NMEA_ProcessCommand(NMEA_command_t *command, char *data, uint8_t checksum)
{
	bool update = false;

	if (command->token != NMEA_UNKNOWN) {
		update = (*command->process)(data);

		if (command->display && !DisplayAll) {
			TD_NMEA_DisplayCommand(command->ascii, checksum);
		}
	}
	if (update && UpdateCallback != 0) {
		UpdateCallback(&Fix);
	}
}

/***************************************************************************//**
 * @brief
 *   Process an NMEA character. Will call appropriate command process if
 *   the character is the last of the command. Will also call
 *   the UpdateCallback and dislay NMEA if enabled.
 *
 * @param[in] data
 *   The NMEA character to process.
 ******************************************************************************/
static void TD_NMEA_Process(char data)
{
	int i;

	// Discard all 0xFF
	if (data == 0xFF) {
		return;
	}

#ifdef DEBUG_PARSER
	if ((data >= ' ' && data <= 'z') || (data == 0xD) || (data == 0xA)) {
		tfp_printf("%c", data);
	} else {
		if (data == 0xB5) {
			tfp_printf("\r\n");
		}
		tfp_printf("0x%02X(%c)", data, data ? data : ' ');
	}
#endif

	switch (NMEAState) {

	// Search for start of message '$'
	case NMEA_STATE_SOM:
		if (data == '$') {
			TD_NMEA_ResetElector();
			LastCommand.token = NMEA_UNKNOWN;

			// Reset checksum
			ComputedChecksum = 0;

			// Initialize command Counter
			Index = 0;
			NMEAState = NMEA_STATE_CMD;
		}
		break;

	// Retrieve command (NMEA Address)
	case NMEA_STATE_CMD:
		if (data != ',' && data != '*') {
			for (i = 0; i < LAST_COMMAND; i++) {
				if (CommandsElector[i]) {

					// If elector for current command is still valid
					if (Index < NMEACommands[i].length) {

						// If length is OK
						if (NMEACommands[i].ascii[Index] != data) {
							CommandsElector[i] = false;
						}
					} else {
						CommandsElector[i] = false;
					}
				}
			}
			Command[Index++] = data;
			ComputedChecksum ^= data;

			// Check for command overflow
			if (Index >= MAX_COMMAND) {
				NMEAState = NMEA_STATE_SOM;
			}
		} else {
			LastCommand.token = NMEA_UNKNOWN;

			// Should be only one left
			for (i = 0; i < LAST_COMMAND; i++) {
				if (CommandsElector[i]) {
					memcpy(&LastCommand, &NMEACommands[i], sizeof(NMEA_command_t));
				}
			}

			// Terminate command
			Command[Index] = '\0';
			ComputedChecksum ^= data;

			// Initialize data counter
			Index = 0;
			FieldCount = 0;
			FieldIndexList[0] = 0;

			// Go to get data state
			NMEAState = NMEA_STATE_DATA;
		}
		break;

	// Store data and check for end of sentence or checksum flag
	case NMEA_STATE_DATA:
		if (FieldCount+1 >= MAX_FIELD_COUNT) {
			NMEAState = NMEA_STATE_SOM;
			break;
		}
		if (data == '*') {

			// Checksum flag
			Data[Index] = '\0';
			NMEAState = NMEA_STATE_CHECKSUM_1;
		} else {

			// No checksum flag -> bad
			if (data == '\r') {
				NMEAState = NMEA_STATE_SOM;
			}

			// Split parameters with 0s
			if (data == ',') {
				FieldIndexList[++FieldCount] = Index + 1;
				Data[Index] = '\0';
			} else {
				Data[Index] = data;
			}

			// Store data and calculate checksum
			ComputedChecksum ^= data;

			// Check for buffer overflow
			if (++Index >= MAX_DATA) {
				NMEAState = NMEA_STATE_SOM;
			}
		}
		break;

	case NMEA_STATE_CHECKSUM_1:
		if ((data - '0') <= 9) {
			ReceivedChecksum = (data - '0') << 4;
		} else {
			ReceivedChecksum = (data - 'A' + 10) << 4;
		}
		NMEAState = NMEA_STATE_CHECKSUM_2;
		break;

	case NMEA_STATE_CHECKSUM_2:
		if ((data - '0') <= 9) {
			ReceivedChecksum |= (data - '0');
		} else {
			ReceivedChecksum |= (data - 'A' + 10);
		}
		NMEAState = NMEA_STATE_SOM;
		if (ComputedChecksum == ReceivedChecksum) {
			if (DisplayAll) {

#ifdef DEBUG_PARSER
				tfp_printf("\r\n");
#endif

				TD_NMEA_DisplayCommand(Command, ReceivedChecksum);

#ifdef DEBUG_PARSER
				tfp_printf("|");
#endif

			}

			// All parameters are global, no real need to use them as parameters...
			TD_NMEA_ProcessCommand(&LastCommand, Data, ReceivedChecksum);
		}
		break;

	default:
		NMEAState = NMEA_STATE_SOM;
		break;
	}
}

/** @} */

/*******************************************************************************
 **************************   PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_NMEA_GLOBAL_FUNCTIONS Global Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Initialize the NMEA parser.
 *
 * @param[in] callback
 *   Pointer to the update callback function. Each time the NMEA parser
 *   parses a valid command, the callback will be called and fix will
 *   point to parsed informations.
 ******************************************************************************/
void TD_NMEA_Init(void (*callback)(TD_GEOLOC_Fix_t *fix))
{
	TD_NMEA_Reset();
	LastCommand.token = NMEA_UNKNOWN;
	UpdateCallback = callback;
}

/***************************************************************************//**
 * @brief
 *   Reset the NMEA parser. Must be called each time the GPS is power on to
 *   make sure all buffers are cleared.
 ******************************************************************************/
void TD_NMEA_Reset(void)
{
	NMEAState = NMEA_STATE_SOM;
	TD_NMEA_ResetElector();
	FieldCount = 0;
	TD_GEOLOC_FixInit(&Fix);
}

/***************************************************************************//**
 * @brief
 *   Parse an NMEA buffer.
 *
 * @param[in] buffer
 *   Pointer to the NMEA buffer to parse.
 *
 * @param[in] length
 *   Length in characters of the NMEA buffer to parse.
 *
 * @return
 *   Returns true if the buffer was correctly parsed, false otherwise.
 ******************************************************************************/
bool TD_NMEA_ParseBuffer(char *buffer, int length)
{
	int i;

	for (i = 0; i < length; i++) {
		TD_NMEA_Process(buffer[i]);
	}
	return true;
}

/** @} */


/** @addtogroup TD_NMEA_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 *   Enable output of given NMEA commands. Call several times (one per NMEA
 *   command you want to enable/disable). Default is disabled. Use * to
 *   outputs raw GPS NMEA. Handled NMEA commands are GPGGA and GPRMC.
 *
 * @param[in] enabled
 *   Enable/disable flag.
 *
 * @param[in] command
 *   NMEA command as an ASCII string to process, "*" processes all commands.
 ******************************************************************************/
void TD_NMEA_EnableOutput(bool enabled, char *command)
{
	int i;

	if (strcmp(command, "*") == 0) {
		DisplayAll = enabled;
	} else {
		for (i = 0; i < LAST_COMMAND; i++) {
			if (strcmp(NMEACommands[i].ascii, command) == 0) {
				NMEACommands[i].display = enabled;
				break;
			}
		}
	}
}

/** @} */

/** @} (end addtogroup TD_NMEA) */
