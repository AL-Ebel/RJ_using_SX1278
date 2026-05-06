/*
 * slave_side.h
 *
 *  Created on: Feb 4, 2026
 *      Author: Amal
 */

#ifndef INC_SLAVE_SIDE_H_
#define INC_SLAVE_SIDE_H_

#include "stm32f4xx_hal.h"

/* ================= LORA GPIO PINS ================= */
#define MAX_ROBOTS 10
#define LORA_NSS_PORT GPIOB
#define LORA_NSS_PIN  GPIO_PIN_12

#define LORA_RST_PORT GPIOA
#define LORA_RST_PIN  GPIO_PIN_12
extern const char *robot_tone;

#define REG_FIFO                0x00
#define REG_OP_MODE             0x01
#define REG_FRF_MSB             0x06
#define REG_FRF_MID             0x07
#define REG_FRF_LSB             0x08
#define REG_PA_CONFIG           0x09
#define REG_FIFO_ADDR_PTR       0x0D
#define REG_FIFO_TX_BASE_ADDR   0x0E
#define REG_FIFO_RX_BASE_ADDR   0x0F
#define REG_IRQ_FLAGS           0x12
#define REG_RX_NB_BYTES         0x13
#define REG_PKT_RSSI_VALUE      0x1A
#define REG_MODEM_CONFIG_1      0x1D
#define REG_MODEM_CONFIG_2      0x1E
#define REG_PAYLOAD_LENGTH      0x22
#define REG_DIO_MAPPING_1       0x40
#define LONG_RANGE_MODE        0x80
#define REG_FIFO_RX_CURRENT_ADDR 0x10


#define SX1278_MODE_SLEEP           (LONG_RANGE_MODE | MODE_SLEEP)        // 0x80
#define SX1278_MODE_STDBY           (LONG_RANGE_MODE | MODE_STDBY)        // 0x81
#define SX1278_MODE_TX              (LONG_RANGE_MODE | MODE_TX)           // 0x83
#define SX1278_MODE_RX              (LONG_RANGE_MODE | MODE_RX_CONTINUOUS)// 0x85

#define IRQ_TX_DONE                 0x08
#define IRQ_RX_DONE                 0x40
#define IRQ_CRC_ERROR               0x20
#define IRQ_CLEAR_ALL               0xFF

#define TXEN_PORT	GPIOD
#define TXEN_PIN	GPIO_PIN_9

#define RXEN_PORT	GPIOD
#define RXEN_PIN	GPIO_PIN_8

/* ================= ROBOT IDENTIFICATION ================= */
#define ROBOT_UID        1002  // Change this for each robot
#define UID_BROADCAST    0xFFFF

#define CMD_BROADCAST_ENTER 0x20
#define CMD_BROADCAST_EXIT 0x21

/* ================= FUNCTION PROTOTYPES ================= */
void Slave_App_Init(void);
void Slave_App_Loop(void);
void Motor_Task(void);
void Telemetry_Task(void);
void SX1278_Process(void);
void SA818_Task(void);
void sx1278_write_reg(uint8_t addr, uint8_t val);
uint8_t sx1278_read_reg(uint8_t addr);
void SX1278_SetRx(void);
void SX1278_SetTx(void);
void SX1278_Send(uint8_t *data, uint8_t len);
void SX1278_HandleRx(void);
void Radio_Task(void);

#endif /* INC_SLAVE_SIDE_H_ */
