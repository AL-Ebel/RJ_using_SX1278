#include "main.h"
#include "slave_battery.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdint.h>

/* ================================================================
   Private state
   ================================================================ */
static ADC_HandleTypeDef *hadc_slave;
static float slave_acs712_zero_offset    = SLAVE_ACS712_ZERO_VOLTAGE;
static float slave_vdiv_ratio_calibrated = SLAVE_VDIV_RATIO;
static uint32_t slave_last_current_tick  = 0;


static void Select_Slave_CurrentChannel(void)
{
    ADC_ChannelConfTypeDef s = {0};
    s.Channel      = ADC_CHANNEL_8;   // PB0 → CURRENT (FIXED)
    s.Rank         = 1;
    s.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    HAL_ADC_ConfigChannel(hadc_slave, &s);
}

static void Select_Slave_VoltageChannel(void)
{
    ADC_ChannelConfTypeDef s = {0};
    s.Channel      = ADC_CHANNEL_9;   // PB1 → VOLTAGE (FIXED)
    s.Rank         = 1;
    s.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    HAL_ADC_ConfigChannel(hadc_slave, &s);
}
/* ================================================================
   INIT
   ================================================================ */

void Slave_Battery_Init(ADC_HandleTypeDef *hadc)
{
    hadc_slave = hadc;
    slave_last_current_tick = HAL_GetTick();
}

/* ================================================================
   VOLTAGE READ
   ================================================================ */

float Slave_Battery_ReadVoltage(void)
{
    static float last_voltage = 0.0f;
    static uint32_t last_read = 0;
    if(HAL_GetTick() - last_read < 1000)
        return last_voltage;

    last_read = HAL_GetTick();

    Select_Slave_VoltageChannel();

    /* Settling */
    for(int i = 0; i < 20; i++)
    {
        HAL_ADC_Start(hadc_slave);
        HAL_ADC_PollForConversion(hadc_slave, 100);
        HAL_ADC_GetValue(hadc_slave);
        HAL_ADC_Stop(hadc_slave);
    }
    uint32_t sum = 0;
    for(int i = 0; i < 32; i++)
    {
        HAL_ADC_Start(hadc_slave);
        HAL_ADC_PollForConversion(hadc_slave, 100);
        sum += HAL_ADC_GetValue(hadc_slave);
        HAL_ADC_Stop(hadc_slave);
    }

    float avg    = sum / 32.0f;
    float adc_v  = (avg / SLAVE_ADC_RESOLUTION) * SLAVE_ADC_REF_VOLTAGE;
    float batt_v = adc_v * slave_vdiv_ratio_calibrated;

//    if(batt_v > SLAVE_BATT_VOLT_MAX) batt_v = SLAVE_BATT_VOLT_MAX;

    /* Noise rejection */
    if(last_voltage > 0.0f && fabs(batt_v - last_voltage) > 0.3f)
        return last_voltage;

    last_voltage = batt_v;
    return batt_v;

}

/* ================================================================
   PERCENT CALCULATION
   ================================================================ */

uint8_t Slave_Battery_CalcPercent(float voltage)
{
    float pct = ((voltage - SLAVE_BATT_VOLT_MIN) /
                (SLAVE_BATT_VOLT_MAX - SLAVE_BATT_VOLT_MIN)) * 100.0f;

    if(pct < 0.0f)   pct = 0.0f;
    if(pct > 100.0f) pct = 100.0f;

    return (uint8_t)pct;
}

/* ================================================================
   CLEAN PUBLIC API
   ================================================================ */

uint8_t Slave_Battery_GetPercent(void)
{
    float voltage = Slave_Battery_ReadVoltage();
    return Slave_Battery_CalcPercent(voltage);

}

/* ================================================================
   CURRENT READ (ACS712)
   ================================================================ */

float Slave_ACS712_ReadCurrentAmps(void)
{
	Select_Slave_CurrentChannel();

    uint32_t sum = 0;
    for(int i = 0; i < 64; i++)
    {
        HAL_ADC_Start(hadc_slave);
        HAL_ADC_PollForConversion(hadc_slave, 100);
        sum += HAL_ADC_GetValue(hadc_slave);
        HAL_ADC_Stop(hadc_slave);
    }

    float raw     = sum / 64.0f;
    float voltage = (raw / SLAVE_ADC_RESOLUTION) * SLAVE_ADC_REF_VOLTAGE;
    float current = (voltage - slave_acs712_zero_offset) / SLAVE_ACS712_SENSITIVITY;

    if(current > -SLAVE_ACS712_NOISE_THRESHOLD &&
       current <  SLAVE_ACS712_NOISE_THRESHOLD)
        current = 0.0f;

    return current;
}

/* ================================================================
   CALIBRATION
   ================================================================ */

void Slave_ACS712_Calibrate(void)
{
    Select_Slave_CurrentChannel();

    uint32_t sum = 0;
    for(int i = 0; i < 200; i++)
    {
        HAL_ADC_Start(hadc_slave);
        HAL_ADC_PollForConversion(hadc_slave, 10);
        sum += HAL_ADC_GetValue(hadc_slave);
        HAL_ADC_Stop(hadc_slave);
        HAL_Delay(2);
    }

    float avg_raw  = sum / 200.0f;
    float measured = (avg_raw / SLAVE_ADC_RESOLUTION) * SLAVE_ADC_REF_VOLTAGE;

    if(measured >= 2.4f && measured <= 2.6f)
        slave_acs712_zero_offset = measured;

}
