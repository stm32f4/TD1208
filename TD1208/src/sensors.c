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

#define SENSOR_INTERVAL  		30u
#define SIGFOX_SAMPLES  		30u
#define	DUST_SAMPLES_SKIP		5U
#define VOUT_NO_DUST			50

#define DUST_VCC_ON		GPIO_PinOutSet(gpioPortC, 0)
#define DUST_VCC_OFF	GPIO_PinOutClear(gpioPortC, 0)
#define DUST_LED_ON		GPIO_PinOutSet(gpioPortC, 15)
#define DUST_LED_OFF	GPIO_PinOutClear(gpioPortC, 15)
#define BUTTON_PORT		gpioPortC
#define BUTTON_BIT		14
#define BUTTON_MASK		1<<14

#define OPERATION_DUST_ENABLED		0b00000001u
#define OPERATION_LIGHT_ENABLED		0b00000010u
#define OPERATION_HUMIDITY_ENABLE 	0b00000100u
#define OPERATION_PRESS_ENABLED 	0b00001000u
#define OPERATION_SEND_IMMEDIATE	0b00010000u
#define OPERATION_SIGFOX_ENABLED	0b00100000u
#define OPERATION_HEARTBEAT_ENABLED	0b01000000u
#define OPERATION_PROCESS_COMMAND	0b10000000u

#define BUTTON_COUNT_MASK	0b01110000u
#define BUTTON_DELAY_MASK	0b00001111u
#define BUTTON_STATUS_MASK	0b10000000u

#define BUTTON_PRESSED	(buttonState & BUTTON_STATUS_MASK)
#define BUTTON_RELEASED	((buttonState & BUTTON_STATUS_MASK)^(BUTTON_STATUS_MASK))
#define BUTTON_STATE	(buttonState & BUTTON_STATUS_MASK)
//#define DEBUG_BUTTON

/******************************************************************************
 *************************  Functions declaration  ****************************
 ******************************************************************************/
void TD_USER_Heartbeat();
void TD_USER_InitSensors();
void TD_USER_Measure(uint32_t count, uint8_t repetition);
void TD_USER_MeasurePressure(int32_t *temperature, uint32_t *pressure);
void TD_USER_MeasureHumidity(int32_t *temperature, uint32_t *humidity);
void TD_USER_MeasureLight(uint16_t *light);
void TD_USER_MeasureDust(uint32_t *dust);
void TD_USER_Init();
void TD_USER_Init_BMP180();
void TD_USER_Init_SI7020();
void TD_USER_Init_ISL29035();
void TD_USER_Button(uint32_t mask);
void TD_USER_ButtonProcess(uint8_t count);

/******************************************************************************
 *************************  Global variable declaration  **********************
 ******************************************************************************/

// Task ID for LED blink
uint8_t Scheduler_LED_Id;

// Task ID for sensor Acquisition
uint8_t Scheduler_Sensors_Id;

// Task ID for button led flash
uint8_t Scheduler_Button_Id;

/** Operation status */
uint8_t operationMode = 0;

/** Button repetition and delay
 * bits 3-0 : Duration of the last press in seconds
 * bits 6-4 : Number of short pulses
 * bit 7	: Button state (1: pressed, 0: released
 */
uint8_t buttonState;

// Number of samples for dust measure
uint8_t dustSamples = 50;

void TD_USER_Button(uint32_t mask) {
	static uint32_t lastOff = 0;
	static uint32_t lastOn = 0;
#ifdef DEBUG_BUTTON
	static uint32_t delay = 0;
#endif
	uint32_t temp = 0;

	// Exit if button state is unchanged (process only state change)
	if (GPIO_PinInGet(BUTTON_PORT, BUTTON_BIT)) {
		if (BUTTON_STATE) {
			return;
		}
	} else if (!BUTTON_STATE) {
		return;
	}

	// Debounce -  Check delay since last change
	if (BUTTON_STATE) {
		// If switch is pressed starting time is when button went down
		temp = lastOn;
	} else {
		// If switch is released starting time is when button went up
		temp = lastOff;
	}
	if (RTC->CNT < temp) {
		temp = RTC->CNT + 0XFFFFFFFF - temp;
	} else {
		temp = RTC->CNT - temp;
	}
#ifdef DEBUG_BUTTON
	delay = temp;
#endif
	if ((temp < 32768 / 100) && (lastOff != lastOn)) {
		return;
	}

	//Update state - Read button
	if (GPIO_PinInGet(BUTTON_PORT, BUTTON_BIT)) {
		buttonState |= BUTTON_STATUS_MASK;
	} else {
		buttonState &= ~BUTTON_STATUS_MASK;
	}

	// Get current counter
	uint8_t count = ((buttonState & BUTTON_COUNT_MASK) >> 4); // get current count

	// Button released
	// 1- Check duration (if short -> increment counter if not process sequence)
	//
	if (BUTTON_RELEASED) {
		// Clear count
		buttonState &= (~BUTTON_COUNT_MASK); // Clear old count

		// If short press, increase counter
		if (temp < 32768 * 2) {
			count++; // add one to count
			count = (count << 4) & BUTTON_COUNT_MASK; // Shift left and apply mask
			buttonState |= count; // Apply new count
		} else {
			// Perform action for the short press sequence
			buttonState &= (~BUTTON_COUNT_MASK); // reset count
			TD_USER_ButtonProcess(count);
		}

		lastOff = RTC->CNT;

		// Button pressed
		// 1- Check if the pressure is in sequence (if yes do nothing else reset sequence)
	} else {
		// Delay between last press
		if (RTC->CNT < lastOff) {
			temp = RTC->CNT + 0XFFFFFFFF - lastOff;
		} else {
			temp = RTC->CNT - lastOff;
		}

		// If last key press is not within 2 s then reset counter
		if (temp > 32768 * 2) {
			buttonState &= ~BUTTON_COUNT_MASK;
		}

		lastOn = RTC->CNT;
	}

	// Get current counter
	count = ((buttonState & BUTTON_COUNT_MASK) >> 4); // get current count
#ifdef DEBUG_BUTTON
			tfp_printf("%sButton : %s (count=%u) (delay=%u)\r\n",
					buttonState & BUTTON_STATUS_MASK ? "\r\n" : "",
					buttonState & BUTTON_STATUS_MASK ? "pressed" : "released", count,
					(delay * 1000) / 32768);
#endif
}

/**
 * @Brief Apply action for the number of strokes
 * @Param[in] count
 * 		Number of key presses
 */
void TD_USER_ButtonProcess(uint8_t count) {

	switch (count) {
	case 0:
		//Reset
		tfp_printf("Key press : Reset system...\r\n");
		NVIC_SystemReset();
		break;
	case 1:
		//Command 1 : Toggle monitoring
		operationMode ^= OPERATION_PRESS_ENABLED;
		operationMode ^= OPERATION_HUMIDITY_ENABLE;
		operationMode ^= OPERATION_LIGHT_ENABLED;
		operationMode ^= OPERATION_DUST_ENABLED;
		tfp_printf("Sensing %s...\r\n",
				operationMode & OPERATION_PRESS_ENABLED ?
						"enabled" : "disabled");
		break;
	case 2:
		//Set samples to 100
		dustSamples = 100;
		tfp_printf("Dust samples set to %u...\r\n", dustSamples);
		break;
	case 3:
		//Set samples to 50
		dustSamples = 50;
		tfp_printf("Dust samples set to %u...\r\n", dustSamples);
		break;
	case 4:
		//Set samples to 10
		dustSamples = 10;
		tfp_printf("Dust samples set to %u...\r\n", dustSamples);
		break;
	case 5:
		//Command 5 : Toggle Heartbeat LED pulse
		operationMode ^= OPERATION_HEARTBEAT_ENABLED;
		tfp_printf("Heartbeat %s...\r\n",
				operationMode & OPERATION_HEARTBEAT_ENABLED ? "on" : "off");
		break;
	case 6:
		//Command 6 : Toggle Sigfox emission
		operationMode ^= OPERATION_SIGFOX_ENABLED;
		tfp_printf("Sigfox %s\r\n",
				operationMode & OPERATION_SIGFOX_ENABLED ? "On" : "Off");
		break;
	case 7:
		//Force sending data
		operationMode |= OPERATION_SEND_IMMEDIATE;
		tfp_printf("Force sigfox message next time...\r\n",dustSamples);
		break;
	}

	operationMode |= OPERATION_PROCESS_COMMAND;
	CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_TIMER0;
	TIMER0->CTRL = TIMER_CTRL_PRESC_DIV256;

	while (count > 0) {
		GPIO_PinOutSet(PRODUCT_LED_PORT, PRODUCT_LED_BIT);
		TIMER0->CNT = 0;
		TIMER0->CMD = TIMER_CMD_START;
		while (TIMER0->CNT < 5300 * 1)
			;
		TIMER0->CMD = TIMER_CMD_STOP;

		GPIO_PinOutClear(PRODUCT_LED_PORT, PRODUCT_LED_BIT);
		TIMER0->CNT = 0;
		TIMER0->CMD = TIMER_CMD_START;
		while (TIMER0->CNT < 5300 * 1)
			;
		TIMER0->CMD = TIMER_CMD_STOP;

		count--;
	}

	CMU->HFPERCLKEN0 ^= CMU_HFPERCLKEN0_TIMER0;

	operationMode ^= OPERATION_PROCESS_COMMAND;
}

/**
 * @brief Custom init routine. Init I2C and register call back functions
 */
void TD_USER_Init() {

//USR2 : enable 5V
	GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 0);
//DUST_VCC_ON;

//USR1 : enable led
	GPIO_PinModeSet(gpioPortC, 15, gpioModePushPull, 0);

//Enable interrupt for switch connected to USR4
	GPIO_PinModeSet(BUTTON_PORT, BUTTON_BIT, gpioModeInput, 0);
// Set up a user hook on button pin interrupt
	int type;
	IRQn_Type irq_parity;
	type = (BUTTON_MASK & TD_GPIO_ODD_MASK) ?
	TD_GPIO_USER_ODD :
												TD_GPIO_USER_EVEN;
	TD_GPIO_SetCallback(type, TD_USER_Button, BUTTON_MASK);
// Enable rising edge interrupts on button pin
	GPIO_IntConfig(BUTTON_PORT, BUTTON_BIT, true, true, true);

// Clear and enable the corresponding interrupt in the CPU's Nested Vector
// Interrupt Controller
	irq_parity =
			(BUTTON_MASK & TD_GPIO_ODD_MASK) ? GPIO_ODD_IRQn : GPIO_EVEN_IRQn;
	NVIC_ClearPendingIRQ(irq_parity);
	NVIC_EnableIRQ(irq_parity);

// Initialize I2C
	I2CDRV_Init();
	TD_RTC_Delay(32678 / 10);

// Initialize sensors
	TD_USER_InitSensors();

// Enable heartbeat flash
	operationMode |= OPERATION_HEARTBEAT_ENABLED;

// Enable Sigfox transmission
	operationMode |= OPERATION_SIGFOX_ENABLED;

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
	TD_USER_Init_BMP180();

// Check Si7020
	TD_USER_Init_SI7020();

// Check isl29035
	TD_USER_Init_ISL29035();

// Enable sensors
	operationMode = operationMode | OPERATION_DUST_ENABLED
			| OPERATION_HUMIDITY_ENABLE | OPERATION_LIGHT_ENABLED
			| OPERATION_PRESS_ENABLED;
}

/**
 * @brief  Toggle LED
 **/
void TD_USER_Heartbeat() {

	// Return if heartbeat flash is disable or processing command confirmation
	if (!(operationMode
			& (OPERATION_HEARTBEAT_ENABLED | OPERATION_PROCESS_COMMAND))) {
		GPIO->P[PRODUCT_LED_PORT].DOUTCLR = 1 << PRODUCT_LED_BIT;
		return;
	}

	// Pulse LED - Volatile counter to avoid optimization !
	volatile uint32_t time = 4000;
	GPIO->P[PRODUCT_LED_PORT].DOUTSET = 1 << PRODUCT_LED_BIT;
	while (time > 0)
		time--;
	GPIO->P[PRODUCT_LED_PORT].DOUTCLR = 1 << PRODUCT_LED_BIT;
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
void TD_USER_Measure(uint32_t count, uint8_t repetition) {
	typedef struct structMes {
		int8_t temperatureMin;
		int8_t temperatureAvg;
		int8_t temperatureMax;
		uint8_t pressure;
		uint8_t humidity;
		uint8_t lightMin;
		uint8_t lightAvg;
		uint8_t lightMax;
		uint8_t dustMin;
		uint8_t dustAvg;
		uint8_t dustMax;
		uint8_t battery;
		int16_t temperatureSum;
		uint16_t lightSum;
		uint16_t dustSum;
	} measures;

	typedef union {
		measures mes;
		uint8_t bmes[12];
		uint16_t sum[3];
	} allmes;

	static allmes measure;

// Reset values
	if (count == 0) {
		measure.mes = (measures ) { 0x7F, 0, 0, 0, 0, 0xFF, 0, 0, 0xFF, 0, 0, 0,
						0, 0, 0 };
	}

// Measure pressure and temp
	uint32_t pressure = 0;
	int32_t temperature = 0;
	TD_USER_MeasurePressure(&temperature, &pressure);

// Measure humidity
	uint32_t humidity = 0;
	int32_t humTemperature = 0;
	TD_USER_MeasureHumidity(&humTemperature, &humidity);

	// Store temperature value in 1/10 of °
	measure.mes.temperatureSum += temperature + humTemperature / 100;
	if (measure.mes.temperatureMin
			> (temperature + humTemperature / 100 + 10) / 20) {
		measure.mes.temperatureMin = (temperature + humTemperature / 100) / 20;
	}
	if (measure.mes.temperatureMax
			< (temperature + humTemperature / 100 + 10) / 20) {
		measure.mes.temperatureMax = (temperature + humTemperature / 100) / 20;
	}

// Measure light
	uint16_t light = 0;
	TD_USER_MeasureLight(&light);
	measure.mes.lightSum += light;

	// Truncate to 8 bits values before computing min and max
	if (light > 0xFF) {
		light = 0xFF;
	}
	if (light > measure.mes.lightMax) {
		measure.mes.lightMax = light;
	}
	if (light < measure.mes.lightMin) {
		measure.mes.lightMin = light;
	}

//  Measure dust
	uint32_t dust = 0;
	TD_USER_MeasureDust(&dust);
	measure.mes.dustSum += dust;
	if (dust > measure.mes.dustMax) {
		measure.mes.dustMax = dust;
	}
	if (dust < measure.mes.dustMin) {
		measure.mes.dustMin = dust;
	}

// Measure voltage
	uint32_t vdd = 0;
	TD_USER_Measure_VDD(&vdd);

// Update counter
	count++;

// Sigfox status
	if (operationMode & OPERATION_SIGFOX_ENABLED) {
		tfp_printf("   (%u/%u).\r\n", count, SIGFOX_SAMPLES);
	} else {
		tfp_printf("   Sigfox off.\r\n");
	}

// When counter reaches the send counter or if immediate sending is requested
	if ((count == SIGFOX_SAMPLES)
			|| (operationMode & OPERATION_SEND_IMMEDIATE)) {
		// Prepare the buffer. Add 0.5 * divider for all values to get rounded values
		measure.mes.temperatureAvg = (measure.mes.temperatureSum
				+ count * 5) / (count * 20);
		measure.mes.pressure = (pressure + 50) / 100 - 900;
		measure.mes.humidity = (humidity + 500) / 1000;
		measure.mes.lightMin =
				measure.mes.lightMin > 0xFF ? 0xFF : measure.mes.lightMin;
		measure.mes.lightAvg =
				(measure.mes.lightSum / count) > 0xFF ?
						0xFF :
						(measure.mes.lightSum + count / 2)
								/ count;
		measure.mes.lightMax =
				measure.mes.lightMax > 0xFF ? 0xFF : measure.mes.lightMax;
		measure.mes.dustAvg = (measure.mes.dustSum + count / 2)
				/ count;
		measure.mes.battery = (vdd + 5) / 10;
		// Send to sigfox
		tfp_printf(
				"Data sent to sigfox: T=%d/%d/%d P=%u H=%u L=%u/%u/%u D=%u/%u/%u V=%u",
				measure.mes.temperatureMin, measure.mes.temperatureAvg,
				measure.mes.temperatureMax, measure.mes.pressure,
				measure.mes.humidity, measure.mes.lightMin,
				measure.mes.lightAvg, measure.mes.lightMax, measure.mes.dustMin,
				measure.mes.dustAvg, measure.mes.dustMax, measure.mes.battery);
		tfp_printf("   Raw: %X%X%X%X%X%X%X%X%X%X%X%X\r\n", measure.bmes[0],
				measure.bmes[1], measure.bmes[2], measure.bmes[3],
				measure.bmes[4], measure.bmes[5], measure.bmes[6],
				measure.bmes[7], measure.bmes[8], measure.bmes[9],
				measure.bmes[10], measure.bmes[11]);
		if (operationMode & OPERATION_SIGFOX_ENABLED) {
			GPIO->P[PRODUCT_LED_PORT].DOUTSET = 1 << PRODUCT_LED_BIT;
			TD_SIGFOX_Send(measure.bmes, 12, 2);
			GPIO->P[PRODUCT_LED_PORT].DOUTCLR = 1 << PRODUCT_LED_BIT;
		} else {
			tfp_printf("Sigfox transmission not enabled.\r\n");
		}
		// Reset counter if not immediate sending
		if (operationMode & OPERATION_SEND_IMMEDIATE) {
			operationMode ^= OPERATION_SEND_IMMEDIATE;
		} else {
			count = 0;

		}
	} else {
		TD_USER_Heartbeat();
	}

// Store counter
	TD_SCHEDULER_SetArg(Scheduler_Sensors_Id, count);
}

/**
 *@brief read BMP180
 *
 *@param[out] temperature
 *@param[out] pressure
 */
void TD_USER_MeasurePressure(int32_t *temperature, uint32_t *pressure) {
	if (operationMode & OPERATION_PRESS_ENABLED) {
		BMP180_CalculateTempPressure(BMP180_GetTempData(),
				BMP180_GetPressureData(), temperature, (int32_t*) pressure);
		tfp_printf("Bmp180 T: %d.%u  P: %u   ", *temperature / 10,
				*temperature - *temperature / 10 * 10, *pressure);
	} else {
		tfp_printf("Bmp180 T: N/A  P: N/A   ");
	}
}

/**
 *@brief read SI7029
 *
 *@param[out] temperature
 *@param[out] humidity
 *
 */
void TD_USER_MeasureHumidity(int32_t *temperature, uint32_t *humidity) {

	if (operationMode & OPERATION_HUMIDITY_ENABLE) {
		Si7020_MeasureRHAndTemp(humidity, temperature);

		tfp_printf("Si7020 T: %d.%u", *temperature / 1000,
				(*temperature - *temperature / 1000 * 1000) / 100);
		tfp_printf("  H: %u.%u", *humidity / 1000,
				(*humidity - *humidity / 1000 * 1000) / 100);
	} else {
		tfp_printf("Si7020 T: N/A  H: N/A");
	}
}

/**
 *@brief Read light intensity
 *
 *@param[out] light
 *
 */
void TD_USER_MeasureLight(uint16_t *light) {

	if (operationMode & OPERATION_LIGHT_ENABLED) {
// One shot conversion
		Isl29035_Start(ISL29035_CMD1_ASL_ONCE);

// Wait 10 ms (6,5ms required for ADC Res 12  bits)
		TD_RTC_Delay(32768 / 327);

// Read value
		if (!Isl29035_ReadValue(light)) {
			tfp_printf("Error while reading light value\r\n");
		}

		tfp_printf("   Isl29035 L: %u", *light);
	} else {
		tfp_printf("   Isl29035 L: N/A");
	}
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
	*value = (*value * 500L) / 4096L;
}

/**
 *@brief Measure dust in mg/m3
 *
 *@param[out] dust
 *		dust  value µg/m3 x 10
 */
void TD_USER_MeasureDust(uint32_t *dust) {

	uint8_t cnt = dustSamples;		// Number of measures
	uint32_t zeroValue = 0;			// ADC read with IR LED Off
	uint32_t value = 0;				// ADC current value
	uint32_t maxValue = 0;			// Average value of ADC

// Check if Dust measurement is enabled
	if (!(operationMode & OPERATION_DUST_ENABLED)) {
		*dust = 0;
		tfp_printf("   GP2Y1010 D: N/A");
		return;
	}

// Enable TIMER 0 to get accurate delay
	CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_TIMER0;
	TIMER0->CTRL = TIMER_CTRL_PRESC_DIV256;

// Read ADC to get Zero
	TD_USER_Measure_CH6(&zeroValue);

//Sets 5 v
	DUST_VCC_ON;

// Wait for amplifier to stabilize (0,1s)
// Wait 100 ms
	TIMER0->CNT = 0;
	TIMER0->CMD = TIMER_CMD_START;
	while (TIMER0->CNT < 5300 * 1)
		;
	TIMER0->CMD = TIMER_CMD_STOP;

// Perform MAX_DUST_SAMPLES measures
	while (cnt > 0) {
		// Set LED on
		DUST_LED_ON;

		// Wait 0,28 ms (27,5 precisely)
		TIMER0->CNT = 0;
		TIMER0->CMD = TIMER_CMD_START;
		while (TIMER0->CNT < 15)
			;
		TIMER0->CMD = TIMER_CMD_STOP;

		// Read value
		TD_USER_Measure_CH6(&value);

		// Switch lED off
		DUST_LED_OFF;

		// Keep max value once the number of samples skipped is reached
		if ((dustSamples - cnt >= DUST_SAMPLES_SKIP) && (value > maxValue)) {
			maxValue = value;
		}

		// Wait 10 ms
		TIMER0->CNT = 0;
		TIMER0->CMD = TIMER_CMD_START;
		while (TIMER0->CNT < 530)
			;
		TIMER0->CMD = TIMER_CMD_STOP;

		cnt--;
	}

// Stops Timer 0 clock
	CMU->HFPERCLKEN0 ^= CMU_HFPERCLKEN0_TIMER0;

// Remove offset
	if (maxValue > VOUT_NO_DUST) {
		maxValue -= VOUT_NO_DUST;
		// Scale value (in units of 10µg/m3)
		*dust = maxValue / 5;
	} else {
		*dust = 0;
	}

//Unset 5 v
	DUST_VCC_OFF;

	tfp_printf("   GP2Y1010 D: %d.%u%u", *dust / 100, *dust / 10,
			*dust - *dust / 100 * 100);
}

void TD_USER_Init_BMP180() {
	BMP180_GetCalData();
	tfp_printf("BMP180 Initialized.\r\n");
}

void TD_USER_Init_SI7020() {
	uint8_t chipId;
	uint8_t fwRev;
	Si7020_GetFirmwareRevision(&fwRev);
	if (Si7020_Detect(&chipId)) {
		tfp_printf("SI7020 with chip Id %X and firmware %X detected.\r\n",
				chipId, fwRev);
	} else {
		tfp_printf("No SI7020 detected.\r\n");
	}
}

void TD_USER_Init_ISL29035() {
	uint8_t chipId;
	if (Isl29035_Detect(&chipId)) {
		tfp_printf("ISL29035 with chip Id %X detected.\r\n", chipId);
	} else {
		tfp_printf("No ISL29035 detected.\r\n");
	}

// Reset brown out bit
	Isl29035_WriteRegister(ISL29035_ID_REG, 0b00000000);

// Set ADC resolution to 12 bits (4096 samples)
	Isl29035_SetRes(ISL29035_CMD2_ADC_12);

// Set Scale  to 1000 lux
	Isl29035_SetScale(ISL29035_CMD2_SCALE_1000);

// One shot acquisition
	Isl29035_Start(ISL29035_CMD1_ASL_ONCE);
}
