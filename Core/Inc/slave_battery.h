/*
 * slave_battery.h
 *

 *  Created on: Apr 6, 2026
 *      Author: HP
 */

#ifndef INC_SLAVE_BATTERY_H_

#define BATT_CRITICAL_PCT 	15
#define SLAVE_ADC_RESOLUTION       4095.0f
#define SLAVE_ADC_REF_VOLTAGE      3.3f

#define SLAVE_BATT_VOLT_MAX        20.0f
#define SLAVE_BATT_VOLT_MIN        12.0f

#define SLAVE_ACS712_SENSITIVITY   0.066f
#define SLAVE_ACS712_ZERO_VOLTAGE  2.5f
#define SLAVE_ACS712_NOISE_THRESHOLD 0.05f

#define SLAVE_VDIV_RATIO			 5.703f
#define SLAVE_BATTERY_CAPACITY_MAH 4000.0f

void     Battery_Init(ADC_HandleTypeDef *hadc);
float    Slave_Battery_ReadVoltage(void);
uint8_t  Slave_Battery_CalcPercent(float voltage);
uint8_t  Slave_Battery_GetPercent(void);
float    Slave_ACS712_ReadCurrentAmps(void);
void     Slave_ACS712_Calibrate(void);

#endif /* INC_SLAVE_BATTERY_H_ */


