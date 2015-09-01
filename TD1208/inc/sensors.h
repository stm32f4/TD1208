/*
 * sensors.h
 *
 *  Created on: 31 mai 2015
 *      Author: thierry
 */

#ifndef __SENSORS_H_
#define __SENSORS_H_

#include <em_device.h>
#include <td_gpio.h>

void TD_USER_Init();
void TD_USER_Heartbeat();
void TD_USER_InitSensors();
void TD_USER_Measure(uint32_t arg, uint8_t repetition);
void TD_USER_MeasurePressure(int32_t *temperature, uint32_t *pressure);
void TD_USER_MeasureHumidity(int32_t *temperature, uint32_t *humidity);
void TD_USER_MeasureLight(uint16_t *light);
void TD_USER_Measure_VDD(uint32_t *vdd);
void TD_USER_Measure_CH6(uint32_t *vdd);
void TD_USER_MeasureDust(uint32_t *dust);

#endif /* __SENSORS_H_ */
