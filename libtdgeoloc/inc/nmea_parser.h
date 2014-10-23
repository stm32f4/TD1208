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

#ifndef __NMEA_PARSER_H
#define __NMEA_PARSER_H

#include <stdint.h>
#include <stdbool.h>

#include "td_geoloc.h"

/**************************************************************************//**
 * @addtogroup TD_NMEA
 * @brief NMEA GPS parser.
 * @{
 ******************************************************************************/

/*******************************************************************************
 *************************   PROTOTYPES   **************************************
 ******************************************************************************/

/** @addtogroup TD_NMEA_GLOBAL_FUNCTIONS Global Functions
 * @{ */

void TD_NMEA_Init(void (*callback)(TD_GEOLOC_Fix_t *fix));
void TD_NMEA_Reset(void);
bool TD_NMEA_ParseBuffer(char *buffer, int length);
/** @} */

/** @addtogroup TD_NMEA_USER_FUNCTIONS User Functions
 * @{ */

void TD_NMEA_EnableOutput(bool enabled, char *command);

/** @} */


/** @} (end addtogroup TD_NMEA) */

#endif // __NMEA_PARSER_H
