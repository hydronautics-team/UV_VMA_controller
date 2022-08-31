#include "MS5837-30BA.h"

#include <math.h>
#include <string.h>
// For printf
#include <stdio.h>
#include <stdlib.h>

uint8_t rx_buffer[3];
uint16_t C[7];
uint32_t D1, D2;
uint8_t RES_DEVICE = 0x1E;
uint8_t CONVERT_D1 = 0x4A;
uint8_t CONVERT_D2 = 0x5A;
uint8_t ADC_READ = 0x00;
uint8_t PROM_READ = 0xA0;
int32_t result;

HAL_StatusTypeDef ret;

int cmp(const void *a, const void *b) {
    return *(int*)a - *(int*)b;
}

	uint32_t pressure_init()
{
	HAL_I2C_Init(&hi2c2);
	HAL_Delay(200);

	uint8_t *p;
	p = &RES_DEVICE;
	ret = HAL_I2C_Master_Transmit(&hi2c2, DEVICE_ADDR, p, COMMAND_LENGTH, HAL_MAX_DELAY);
	if(ret == HAL_ERROR){
		return 0;
	}
	HAL_Delay(200);
	uint8_t prom_addr = 160;

	//receiving the coefs
	for (uint8_t i = 0; i < 8; i++) {
		uint8_t prom_buff[2];
		prom_addr += 2;
		p = &prom_addr;
		HAL_I2C_Master_Transmit(&hi2c2, DEVICE_ADDR, p, COMMAND_LENGTH, HAL_MAX_DELAY);
		HAL_Delay(1);
		HAL_I2C_Master_Receive(&hi2c2, DEVICE_ADDR, prom_buff, PROM_LENGTH, HAL_MAX_DELAY);
		C[i] = (prom_buff[0] << 8) | (prom_buff[1]);
		HAL_Delay(1);

	}
	return 1;
}

int32_t check_pressure()
{
	uint8_t *p;
	p = &CONVERT_D1;
	ret = HAL_I2C_Master_Transmit(&hi2c2, DEVICE_ADDR, p, COMMAND_LENGTH, HAL_MAX_DELAY);
	HAL_Delay(20);

	uint8_t adc_buff[3];
	//reading D1 data
	p = &ADC_READ;
	ret = HAL_I2C_Master_Transmit(&hi2c2, DEVICE_ADDR, p, COMMAND_LENGTH, HAL_MAX_DELAY);
	//HAL_Delay(300);
	HAL_I2C_Master_Receive(&hi2c2, DEVICE_ADDR, adc_buff, ADC_LENGTH, HAL_MAX_DELAY);

	D1 = (adc_buff[0] << 16) | (adc_buff[1] << 8) | (adc_buff[2]);

	//initializing D2 conversion
	p = &CONVERT_D2;
	HAL_I2C_Master_Transmit(&hi2c2, DEVICE_ADDR, p,
			COMMAND_LENGTH, HAL_MAX_DELAY);
	HAL_Delay(20);

	//reading D2 data
	p = &ADC_READ;
	HAL_I2C_Master_Transmit(&hi2c2, DEVICE_ADDR, p,
			COMMAND_LENGTH, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c2, DEVICE_ADDR, adc_buff,
			ADC_LENGTH, HAL_MAX_DELAY);

	D2 = (adc_buff[0] << 16) | (adc_buff[1] << 8) | (adc_buff[2]);
	return calculate();
}

uint32_t reset_pressure(){
	int32_t current_pressure[10];

	for(int i =0;i<10;i++)
	{
		current_pressure[i]=check_pressure();
		HAL_Delay(100);
	}
	qsort(current_pressure, 10, sizeof(int32_t), cmp );
	return current_pressure[4];
}

int32_t calculate()
{
	int64_t dT = 0;
	int64_t TEMP = 0;
	int64_t OFF = 0;
	int64_t SENS = 0;
	int64_t P = 0;

	int64_t SENSi = 0;
	int64_t OFFi = 0;
	int64_t Ti = 0;
	int64_t OFF2 = 0;
	int64_t SENS2 = 0;

	int64_t P2 = 0;
	int64_t TEMP2 = 0;

	//first order compensation
	dT = D2 - C[4]*256;
	TEMP = 2000 + dT*C[5]/8388608;
	OFF = C[1]*65536 + (C[3]*dT)/128;
	SENS = C[0]*32768 + (C[2]*dT)/256;
	P = ((D1 * SENS)/(2097152) - OFF)/8192;

	//second order compensation
	//	if (TEMP/100 < 20)
	//	{
	//		Ti = (3*dT*dT)/8589934592LL;
	//		OFFi = (3*(TEMP-2000)*(TEMP - 2000))/2;
	//		SENSi = (5*(TEMP - 2000)*(TEMP-2000))/8;
	//		if (TEMP/100 < -15)
	//		{
	//			OFFi = OFFi + 7*(TEMP + 1500l)*(TEMP + 1500l);
	//			SENSi = SENSi + 4*(TEMP + 1500l)*(TEMP + 1500l);
	//		}
	//	} else
	//	{
	Ti = (2*dT*dT)/1.37438953E11;
	OFFi = ((TEMP-2000)*(TEMP - 2000))/16;
	SENSi = 0;

	//	}

	OFF2 = OFF - OFFi;
	SENS2 = SENS - SENSi;

	TEMP2 = (TEMP - Ti) / 100; //C
	P2 = (((D1*SENS2)/2097152 - OFF2)/8192)/10; //mbar
	int32_t res[2];
	res[0] = P;
	res[1] = TEMP/100;
	//	char snum1[50];
	//	itoa(9999, snum1, 6);
	//	strcat(snum1,"\r\n");
	//	char snum1[50];
	//	itoa((int)P, snum1, 10);
	//	strcat(snum1,"\r\n");
	//	HAL_UART_Transmit(&huart2, (uint8_t*) snum1, strlen(snum1),6);



	return res[0]/10;
}
