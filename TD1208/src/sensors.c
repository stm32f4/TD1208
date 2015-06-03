/*
 * sensors.c
 *
 *  Created on: 31 mai 2015
 *      Author: thierry
 */

/******************************************************************************
 *************************          headers        ****************************
 ******************************************************************************/

// Custom
#include <sensors.h>
#include <i2cdrv.h>
#include <adcdrv.h>
#include <bmp180_i2cdrv.h>
#include <si7020_i2cdrv.h>
#include <isl29035_i2cdrv.h>

// SDK
#include <td_rtc.h>
#include <td_scheduler.h>
#define USE_PRINTF // Don't miss this
#include <td_printf.h>

#define HEARTBEAT_INTERVAL  	10
#define SENSOR_INTERVAL  		5
#define SIGFOX_INTERVAL  		60

#define DUST_VCC_ON		GPIO_PinOutSet(gpioPortC, 0)
#define DUST_VCC_OFF	GPIO_PinOutClear(gpioPortC, 0)
#define DUST_LED_ON		GPIO_PinOutClear(gpioPortC, 0)
#define DUST_LED_OFF	GPIO_PinOutSet(gpioPortC, 0)

#define VOUT_NO_DUST	500

/******************************************************************************
 *************************  Functions declaration  ****************************
 ******************************************************************************/
void TD_USER_Heartbeat(uint32_t arg, uint8_t repetition);
void TD_USER_InitSensors();
void TD_USER_Measure(uint32_t arg, uint8_t repetition);
void TD_USER_MeasurePressure(int32_t *temperature, uint32_t *pressure);
void TD_USER_MeasureHumidity(int32_t *temperature, uint32_t *humidity);
void TD_USER_MeasureLight(uint16_t *light);
void TD_USER_Init();

/******************************************************************************
 *************************  Global variable declaration  **********************
 ******************************************************************************/

// Task ID for LED blink
uint8_t Scheduler_LED_Id;

// Task ID for BMP180 Acquisition
uint8_t Scheduler_Sensors_Id;

/**
 * @brief Custom init routine. Init I2C and register call back functions
 */
void TD_USER_Init() {

	//USR2 : enable 5V
	GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 0);
	//USR1 : enable led
	GPIO_PinModeSet(gpioPortC, 15, gpioModeDisabled, 0);

	// Initialize I2C
	I2CDRV_Init();
	TD_RTC_Delay(32678 / 10);

	// Initialize sensors
	TD_USER_InitSensors();

	// Register a task for the heartbeat
	tfp_printf("Register Heart beat task.\r\n");
	Scheduler_LED_Id = TD_SCHEDULER_Append(HEARTBEAT_INTERVAL, 0, 0, 0xFF,
			TD_USER_Heartbeat, 1);

	// Register a task for the sensors
	tfp_printf("Register Sensors task.\r\n");
	Scheduler_Sensors_Id = TD_SCHEDULER_Append(SENSOR_INTERVAL, 0, 0, 0xFF,
			TD_USER_Measure, 0);

}

/**
 * @brief Initialize sensors
 */
void TD_USER_InitSensors() {

	// Initialize bmp180
	BMP180_GetCalData();
	tfp_printf("BMP180 Initialized.\r\n");

	// Check Si7020
	uint8_t chipId;
	uint8_t fwRev;
	Si7020_GetFirmwareRevision(&fwRev);
	if (Si7020_Detect(&chipId)) {
		tfp_printf("SI7020 with chip Id %X and firmware %X detected.\r\n",
				chipId, fwRev);
	} else {
		tfp_printf("No SI7020 detected.\r\n");
	}

	// Check isl29035
	if (Isl29035_Detect(&chipId)) {
		tfp_printf("ISL29035 with chip Id %X detected.\r\n", chipId);
	} else {
		tfp_printf("No ISL29035 detected.\r\n");
	}

	// Reset brown out bit
	Isl29035_WriteRegister(ISL29035_ID_REG, 0b00000000);

	// Set ADC resolution to 12 bits (4096 samples)
	Isl29035_SetRes(ISL29035_CMD2_ADC_16);

	// Set Scale  to 4000 lux
	Isl29035_SetScale(ISL29035_CMD2_SCALE_1000);

	// One shot acquisition
	Isl29035_Start(ISL29035_CMD1_ASL_ONCE);

	/** Check registers
	 Isl29035_ReadRegister(ISL29035_COMMAND_1_REG, &regValue);
	 tfp_printf("Reg1 : %X\r\n", regValue);
	 Isl29035_ReadRegister(ISL29035_COMMAND_2_REG, &regValue);
	 tfp_printf("Reg2 : %X\r\n", regValue);
	 */
}

/**
 * @brief  Toggle LED
 **/
void TD_USER_Heartbeat(uint32_t arg, uint8_t repetition) {
	if (arg == 0) {
		// LED Off
		GPIO->P[PRODUCT_LED_PORT].DOUTCLR = 1 << PRODUCT_LED_BIT;
		TD_SCHEDULER_SetArg(Scheduler_LED_Id, 10);
		TD_SCHEDULER_SetInterval(Scheduler_LED_Id, HEARTBEAT_INTERVAL, 0, 0);
	} else {
		// LED On
		GPIO->P[PRODUCT_LED_PORT].DOUTSET = 1 << PRODUCT_LED_BIT;
		TD_SCHEDULER_SetArg(Scheduler_LED_Id, 0);
		TD_SCHEDULER_SetInterval(Scheduler_LED_Id, 0, 0x100, 0);
	}
}

/**
 * @Brief Acquires temperature, pressure, humidity, light
 *
 * @param[in] arg
 * 		current tag for the scheduler
 *
 * @param[in] repetition
 * 		number of remaining repetition
 **/
void TD_USER_Measure(uint32_t arg, uint8_t repetition) {
	typedef struct structMes {
		uint8_t temperature;
		uint8_t pressure;
		uint8_t humidity;
		uint8_t light;
		uint8_t dust;
		uint8_t battery;
	} measures;

	typedef union {
		measures mes;
		uint8_t bmes[6];
	} allmes;

	allmes measure;

	// Measure pressure and temp
	uint32_t pressure = 0;
	int32_t temperature = 0;
	TD_USER_MeasurePressure(&temperature, &pressure);

	// Measure humidity
	uint32_t humidity = 0;
	int32_t humTemperature = 0;
	TD_USER_MeasureHumidity(&humTemperature, &humidity);

	// Measure light
	uint16_t light = 0;
	TD_USER_MeasureLight(&light);

	// Measure voltage
	uint32_t vdd = 0;
	TD_USER_Measure_VDD(&vdd);

	//  Measure dust
	uint32_t dust = 0;
	TD_USER_MeasureDust(&dust);

	// Prepare the buffer
	measure.mes.temperature = (temperature + humTemperature / 100) / 20;
	measure.mes.pressure = (pressure / 100 - 900);
	measure.mes.humidity = humidity / 1000;
	measure.mes.light = light > 0xFF ? 0xFF : light;
	measure.mes.battery = vdd / 10;
	measure.mes.dust = dust;

	// Update counter
	arg++;

	// When counter reaches 60 send again next time
	if (arg == SIGFOX_INTERVAL) {
		// Send to sigfox
		GPIO->P[PRODUCT_LED_PORT].DOUTSET = 1 << PRODUCT_LED_BIT;
		//TD_SIGFOX_Send(measure.bmes, 8, 2);
		GPIO->P[PRODUCT_LED_PORT].DOUTCLR = 1 << PRODUCT_LED_BIT;
		tfp_printf("Data sent to sigfox: T=%d P=%u H=%u L=%u D=%u V=%u\r\n",
				measure.mes.temperature, measure.mes.pressure,
				measure.mes.humidity, measure.mes.light, measure.mes.dust,
				measure.mes.battery);
		// Reset counter
		arg = 0;
	}

	// Store counter
	TD_SCHEDULER_SetArg(Scheduler_Sensors_Id, arg);
}

/**
 *@brief read BMP180
 *
 *@param[out] temperature
 *@param[out] pressure
 */
void TD_USER_MeasurePressure(int32_t *temperature, uint32_t *pressure) {
	BMP180_CalculateTempPressure(BMP180_GetTempData(), BMP180_GetPressureData(),
			temperature, (int32_t*) pressure);
	tfp_printf("Bmp180 T: %d.%u  P: %u   ", *temperature / 10,
			*temperature - *temperature / 10 * 10, *pressure);
}

/**
 *@brief read SI7029
 *
 *@param[out] temperature
 *@param[out] humidity
 *
 */
void TD_USER_MeasureHumidity(int32_t *temperature, uint32_t *humidity) {
	Si7020_MeasureRHAndTemp(humidity, temperature);

	tfp_printf("Si7020 T: %d.%u", *temperature / 1000,
			(*temperature - *temperature / 1000 * 1000) / 100);
	tfp_printf("  H: %u.%u", *humidity / 1000,
			(*humidity - *humidity / 1000 * 1000) / 100);

}

/**
 *@brief Read light intensity
 *
 *@param[out] light
 *
 */
void TD_USER_MeasureLight(uint16_t *light) {

	// One shot conversion
	Isl29035_Start(ISL29035_CMD1_ASL_ONCE);

	// Wait 110 ms (105 required for ADC Res 16  bits)
	TD_RTC_Delay(32768 / 90);

	// Read value
	if (!Isl29035_ReadValue(light)) {
		tfp_printf("Error while reading light value\r\n");
	}
	Isl29035_Start(ISL29035_CMD1_ASL_ONCE);

	tfp_printf("   Isl29035 L: %u", *light);
}

void TD_USER_Measure_VDD(uint32_t *vdd) {

	TD_USER_InitAdc(adcSingleInpVDDDiv3);
	TD_USER_ReadAdc(vdd);
	*vdd = *vdd * 750 / 4096L;
	tfp_printf("   VDD: %u", *vdd);
}

void TD_USER_Measure_CH6(uint32_t *value) {

	TD_USER_InitAdc(adcSingleInpCh6);
	TD_USER_ReadAdc(value);
	*value = *value * 500 / 4096L;
}

/**
 *@brief Measure dust in mg/m3
 *
 *@param[out] dust
 *		dust  value
 */
void TD_USER_MeasureDust(uint32_t *dust) {

	CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_TIMER0;
	TIMER0->CTRL = TIMER_CTRL_PRESC_DIV256;

	//Sets 5 v
	DUST_VCC_ON;

	// Wait for amplifier to stabilize (1s)
	// Wait 100 ms
	TIMER0->CNT = 0;
	TIMER0->CMD = TIMER_CMD_START;
	while (TIMER0->CNT < 5300)
		;
	TIMER0->CMD = TIMER_CMD_STOP;

	// Perform 10 measures
	uint8_t cnt = 10;
	uint32_t value = 0;
	uint32_t avgValue = 0;
	while (cnt > 0) {
		// Set LED on
		DUST_LED_ON;

		// WAit 0,28 ms (27,5 precisely)
		TIMER0->CNT = 0;
		TIMER0->CMD = TIMER_CMD_START;
		while (TIMER0->CNT < 15)
			;
		TIMER0->CMD = TIMER_CMD_STOP;

		// Read value
		TD_USER_Measure_CH6(&value);

		// Average
		avgValue += value;

		// Switch lED off
		DUST_LED_OFF;

		// Wait 10 ms
		TIMER0->CNT = 0;
		TIMER0->CMD = TIMER_CMD_START;
		while (TIMER0->CNT < 530)
			;
		TIMER0->CMD = TIMER_CMD_STOP;

		cnt--;
	}

	// Remove offset
	if (avgValue > VOUT_NO_DUST * 10) {
		avgValue -= VOUT_NO_DUST * 10;
	}

	// Scale value (10*mg)
	*dust = avgValue / 5;

	// Wait 100 ms
	TIMER0->CNT = 0;
	TIMER0->CMD = TIMER_CMD_START;
	while (TIMER0->CNT < 5300)
		;
	TIMER0->CMD = TIMER_CMD_STOP;

	//Unset 5 v
	DUST_VCC_OFF;

	tfp_printf("   GP2Y1010 D: %d.%u\r\n", *dust / 10, *dust - *dust / 10 * 10);
}

