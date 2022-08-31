#include "main.h"

extern UART_HandleTypeDef huart2;
extern uint32_t D1, D2;
extern uint16_t C[7];
extern I2C_HandleTypeDef hi2c2;

#define	COMMAND_LENGTH 1
#define ADC_LENGTH 3
#define PROM_LENGTH 2
#define SEND_TIME 1500
#define RECEIVE_TIME 1500
#define DEVICE_ADDR  0b11101100

int32_t calculate();
uint32_t pressure_init();
int32_t check_pressure();
uint32_t reset_pressure();
