/***************************************************************************//**
 * @file
 * @brief TD Sensor utils
 * @author Telecom Design S.A.
 * @version 1.0.0
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

#include <stdbool.h>
#include <stdint.h>

#include <td_core.h>
#include <td_utils.h>
#include <td_printf.h>

#include "td_sensor_utils.h"

/***************************************************************************//**
 * @addtogroup TD_SENSOR_UTILS Sensor Utils
 * @brief Utility functions
 * @{
 ******************************************************************************/

/*******************************************************************************
 **************************  PUBLIC FUNCTIONS   *******************************
 ******************************************************************************/

/** @addtogroup TD_SENSOR_UTILS_USER_FUNCTIONS User Functions
 * @{ */

/***************************************************************************//**
 * @brief
 * 	 Bit to bit array concatenation.
 *
 *   data = [xxxxxxxx][xxxxx...] ; len = 13
 *   data_append = [aaaaaaaa][aa......] ; len_append = 10
 *
 *   -> data [xxxxxxxx][xxxxxaaa][aaaaaaa.] ; len = 23
 *
 * @param[in] data
 *  Pointer to array to be completed
 *
 * @param[in] len
 *  Pointer to original len in bits of array to be completed. Updated with final len.
 *
 * @param[in] data_append
 *  Pointer to array to be concatenated
 *
 * @param[in] len_append
 *  Len in bits of data to be concatenated
 ******************************************************************************/
void TD_SENSOR_UTILS_BitConcat(uint8_t *data,  uint8_t *len, uint8_t *data_append, uint8_t len_append)
{
	//[xxxxxxxx][xxxxx...] append [(aaa)(aaaaa)][aaaaaaaa][(aaaaa)xxx]
	//*****************************left***right************remain
	uint8_t right_mask;
	uint8_t index ;
	uint8_t temp_len_append = len_append;

//	tfp_printf("len %d len_append %d\r\n",*len,len_append);

	// Left, right and begin index
	uint8_t left = BITS_TO_BYTES(*len) * 8 - *len;
	uint8_t right  = 8 - left;

	// First byte on which data will be appended
	uint8_t index_data = (*len) / 8;

	//tfp_printf("left %d right %d\r\n",left,right);
	//left_mask = GET_BITFIELD_MASK(left);
	right_mask = GET_BITFIELD_MASK(right) << left;
	//tfp_printf("leftmask %x rightmask %x\r\n",left_mask,right_mask);
	index = 0;

	// While this is not the last byte of data to append
	while (temp_len_append > 0) {
		if (left != 0 && right != 0) {
			//tfp_printf("%x %x %x\r\n", data[index_data + index], (data[index_data + index] & right_mask), data_append[index] >> right);

			// Clean up remaining bits and add left bits
			data[index_data + index] = (data[index_data + index] & right_mask) | data_append[index] >> right;
			if (temp_len_append >= left) {
				temp_len_append -= left;
			} else {
				temp_len_append = 0;
			}

			// Append right bits in brand new byte's MSB
			if (temp_len_append > 0) {
				//tfp_printf("r %x %x %x %x\r\n",temp_len_append, data_append[index], data_append[index] << left,(data_append[index] << left)&right_mask);
				data[index_data + index + 1] = (data_append[index] << left) & right_mask;
				if (temp_len_append >= right) {
					temp_len_append -= right;
				} else {
					temp_len_append = 0;
				}
			}
		} else {

			// If length is a multiple of bytes, do it the easy way
			data[index_data + index] = data_append[index];
			if (temp_len_append >= 8) {
				temp_len_append -= 8;
			} else {
				temp_len_append = 0;
			}
		}
		index++;
	}
	*len += len_append;
}

/** @} */

/** @} (end addtogroup TD_SENSOR_UTILS) */
