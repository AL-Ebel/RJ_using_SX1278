#ifndef GPS_H
#define GPS_H

#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"

/* ================== INIT ================== */
void GPS_Init(UART_HandleTypeDef *huart);

/* ================== PROCESS ================== */
void GPS_Process(void);
void GPS_DMA_Process(void);
void GPS_Parse_Byte(uint8_t byte);

/* ================== GETTERS ================== */
double  GPS_GetDistanceMeters(void);
uint8_t GPS_GetFixStatus(void);
float GPS_GetFilteredSpeed(void);




#endif
