/*
 * Motor.h
 *
 *  Created on: Feb 5, 2026
 *      Author: Amal
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_


#include "stm32f4xx_hal.h"


/* ================= MOTOR GPIO PINS ================= */
#define MOTOR_PORT       GPIOB
#define MOTOR_H_PORT	GPIOD
#define MOTOR_IN_R_PIN   GPIO_PIN_8
#define MOTOR_IN_L_PIN   GPIO_PIN_9
#define MOTOR_INH_PIN    GPIO_PIN_7

#define CMD_MOTOR_ON 	0x01
#define CMD_MOTOR_OFF	0x02
#define CMD_TELEMETRY_REQ	0x03

void MOTOR_Off(void);
void MOTOR_On(void);


uint16_t Motor_ReadVoltage(void);
uint16_t Motor_ReadCurrent(void);


#endif /* INC_MOTOR_H_ */
