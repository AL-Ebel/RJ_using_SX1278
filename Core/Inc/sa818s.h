#ifndef SA818S_H
#define SA818S_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* ================== CONFIG ================== */
#define SA818_TIMEOUT 1000

/* ================== UART ================== */
// You are using huart1 globally → declare it properly
extern UART_HandleTypeDef huart1;

/* ================== GPIO ================== */
#define SA818_HL_PORT GPIOE   // CHANGE if needed
#define SA818_HL_PIN  GPIO_PIN_13

/* ================== API ================== */
HAL_StatusTypeDef SA818_Init(const char *initial_tone);
HAL_StatusTypeDef SA818_SetTone(const char *tone);
HAL_StatusTypeDef SA818_SetVolume(void);

extern const char *robot_tone;
extern const char *broadcast_tone;
extern uint8_t broadcast_active;

void SA818_SetHighPower(void);
void SA818_SetLowPower(void);

#endif

