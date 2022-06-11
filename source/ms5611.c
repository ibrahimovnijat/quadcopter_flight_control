#include "ms5611.h"


void reset_MS5611(void)
{
	uint8_t data = 0x1E;
	I2C_Transmit(&hi2c2, BAR_DEVICE_ADDR, &data, 1);
	HAL_Delay(3);
}

void read_MS5611_calibration_data(uint16_t* n_prom)
{
	uint8_t calibration_address[7] = {BAR_CALIBR_C1, BAR_CALIBR_C2, BAR_CALIBR_C3, BAR_CALIBR_C4, BAR_CALIBR_C5, BAR_CALIBR_C6, BAR_CALIBR_CRC};
	uint8_t temp_calibr_data[2] = {0};
	
	for (uint8_t i = 0; i < 7; i++){
		I2C_Receive(&hi2c2, BAR_DEVICE_ADDR, calibration_address[i], (uint8_t*)temp_calibr_data, 2);
		n_prom[i] = ((uint16_t)(temp_calibr_data[0] << 8) | temp_calibr_data[1]);
		if (n_prom[i] > 65535){
			printf("MS5611 ERROR: C%d is out of range\n", (i+1));
//			while(1);
		}
		else{
			printf("C[%d] = %hu\n", (i+1), n_prom[i]);
		}
	}
	HAL_Delay(5);
}

void init_MS5611(void)
{
	reset_MS5611();
	read_MS5611_calibration_data((uint16_t*)C);
	HAL_Delay(10);
}

uint32_t read_raw_pressure_MS5611(void)
{
	uint8_t pressure_data[3] = {0};
	uint8_t data = BAR_CONVERT_D1_4096;  //BAR_CONVERT_D1_1024;		//10-bit oversampling rate
	I2C_Transmit(&hi2c2, BAR_DEVICE_ADDR, &data, 1);
	HAL_Delay(10);						   // 10 ms delay for 12-bit oversampling

	I2C_Receive(&hi2c2,  BAR_DEVICE_ADDR, BAR_ADC_READ_ADDR, (uint8_t*)pressure_data, 3);
	uint32_t raw_pressure = ((uint32_t)(pressure_data[0] << 16) | ((uint32_t)pressure_data[1] << 8) | pressure_data[2]);
	return raw_pressure;
}

uint32_t read_raw_temperature_MS5611(void)
{
	uint8_t temperature_data[3] = {0};
	uint8_t data = BAR_CONVERT_D2_4096;    //BAR_CONVERT_D2_1024;

	I2C_Transmit(&hi2c2, BAR_DEVICE_ADDR, &data, 1);
	HAL_Delay(20);

	I2C_Receive(&hi2c2,  BAR_DEVICE_ADDR, BAR_ADC_READ_ADDR, (uint8_t*)temperature_data, 3);
	uint32_t raw_temperature = ((uint32_t)(temperature_data[0] << 16) | (uint32_t)(temperature_data[1] << 8) | temperature_data[2]);

	return raw_temperature;
}

int32_t* calc_pressure_temperature(void)
{
    static int32_t returnVals[2];
	int32_t dT, T, T2, P;
	int64_t OFF, OFF2, SENS2, SENS;
	uint32_t D1, D2;

	D1 = read_raw_pressure_MS5611();
	D2 = read_raw_temperature_MS5611();

    dT = D2 - ((uint32_t)C[4] * 256);	//pow(2,8)
	if(dT < -16776960 || dT > 16777216){
		printf("MS5611 ERROR: dT is out of range\n");
	}

	OFF = (int64_t)C[1] * 65536 + (int64_t)C[2] * dT / 128; //pow(2, 16)
	if (OFF < -8589672450 || OFF > 12884705280){
		printf("MS5611 ERROR: OFF is out of range\n");
	}

	SENS = (int64_t) C[0] * 32768 + (int64_t) C[2] * dT / 256;   // pow(2,15)
	if (SENS < -4294836225 || SENS > 6442352640){
		printf("MS5611 ERROR: SENS is out of range\n");
	}

	T = 2000 + ((int64_t)dT * C[5]) / 8388608; 	//pow(2,23)

	if (T < 2000){
		T2 = (dT * dT) / 2147483648;	//pow(2,31)
		OFF2 = 5 * (T - 2000) * (T - 2000) / 2;
		SENS2 = 5 * (T - 2000) * (T - 2000) / 4;
		if (T < -1500){
			OFF2  = OFF2  + 7  * (T + 1500) * (T + 1500);
			SENS2 = SENS2 + 11 * (T + 1500) * (T + 1500) / 2;
		}
	}
	else{
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}

	T -= T2;
	OFF -= OFF2;
	SENS -= SENS2;

	P = (D1 * SENS/2097152 - OFF) / 32768 ;			//pow(2,21), pow(2,15)

	returnVals[0] = P;
	returnVals[1] = T;

	return returnVals;
}


float get_altitude(int32_t press)
{
	//float pressure = (float) press / 100.0f;
	float altitude_m = 44307.69 * (1 - pow((float)press/SEA_LEVEL_PRESSURE, 0.190284));
	return altitude_m;		//return altitude in meters
}

int32_t move_ave_pressure(int32_t pressure)
{
	int32_t pressure_filt = 0;

	pressure_filt = (pressure + prev_pressure + prev_pressure2) / 3.0;
	prev_pressure2 = prev_pressure;
	prev_pressure = pressure;

	return pressure_filt;
}

float move_ave_altitude(float alt)
{
	float alt_filt = 0.0f;
	alt_filt = (alt + prev_alt + prev_alt2 + prev_alt3) / 3.0f;
	prev_alt3 = prev_alt2;
	prev_alt2 = prev_alt;
	prev_alt = alt;
	return alt_filt;
}
