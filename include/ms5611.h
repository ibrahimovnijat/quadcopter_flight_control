/*
 * ms5611.h
 *
 *  Created on: July 29, 2018
 *      Author: Nijat Ibrahimov
 */

#ifndef MS5611_H_
#define MS5611_H_

#include "stm32f4xx_hal.h"
#include "serialCommProtocols.h"
#include <math.h>

#define BAR_DEVICE_ADDR 		0xEE
#define I2C_BAR_DATA_READ		0x01

#define BAR_PROM_ADDR           0xA6
#define BAR_ADC_READ_ADDR		0x00

#define BAR_CONVERT_D1_256		0x40
#define BAR_CONVERT_D1_512		0x42
#define BAR_CONVERT_D1_1024		0x44
#define BAR_CONVERT_D1_2048		0x46
#define BAR_CONVERT_D1_4096		0x48

#define BAR_CONVERT_D2_256		0x50
#define BAR_CONVERT_D2_512		0x52
#define BAR_CONVERT_D2_1024		0x54
#define BAR_CONVERT_D2_2048		0x56
#define BAR_CONVERT_D2_4096		0x58

#define BAR_FACTORY_SETUP		0xA0
#define BAR_CALIBR_C1			0xA2
#define BAR_CALIBR_C2			0xA4
#define BAR_CALIBR_C3			0xA6
#define BAR_CALIBR_C4			0xA8
#define BAR_CALIBR_C5			0xAA
#define BAR_CALIBR_C6			0xAC
#define BAR_CALIBR_CRC			0xAE

#define SEA_LEVEL_PRESSURE		1013.25f  		// in mBars, Sea level pressure

I2C_HandleTypeDef hi2c2;

extern uint16_t C[7];
extern int32_t prev_pressure, prev_pressure2;
extern float  prev_alt, prev_alt2, prev_alt3;


void reset_MS5611(void);
void read_MS5611_calibration_data(uint16_t* n_prom);
void init_MS5611(void);

uint32_t read_raw_pressure_MS5611(void);
uint32_t read_raw_temperature_MS5611(void);
int32_t* calc_pressure_temperature(void);
float get_altitude(int32_t pressure);
int32_t move_ave_pressure(int32_t pressure);
float move_ave_altitude(float altitude);

#endif /* MS5611_H_ */
