#include "main.h"
#include "slave_side.h"
#include "lora_config.h"
#include "protocol.h"
#include "Motor.h"
#include "gps.h"
#include "sa818s.h"
#include "slave_battery.h"
#include <string.h>
#include <stdio.h>

extern SPI_HandleTypeDef hspi2;
extern ADC_HandleTypeDef hadc1;
uint8_t tx_pending = 0;
uint32_t last_tx_time = 0;
uint32_t last_rx_time = 0;
volatile uint8_t sa818_pending = 0;
const char *requested_tone = NULL;
#define MOTOR_TIMEOUT 1000
#define TELEMETRY_INTERVAL 1000
#define EXPECTED_LEN 8
#define MOTOR_PULSE_TIME 400
#define MOTOR_FAILSAFE_TIMEOUT 1000
#define TELEMETRY_TX_DELAY_MS 300
volatile uint8_t radio_irq_flags = 0;

extern uint8_t sentence_ready;
extern uint8_t pps_flag;

static uint8_t motor_state = 0;
uint32_t tx_start_time = 0;
volatile uint8_t telemetry_request = 0;
static uint32_t telemetry_req_time = 0;

volatile uint8_t lora_tx_done = 0;
volatile uint8_t lora_rx_done = 0;
volatile uint8_t lora_crc_error = 0;

const char *robot_tone = "0002";
const char *broadcast_tone = "0038";
uint8_t broadcast_active = 0;
void process_command(uint8_t cmd, uint16_t param);

static uint8_t sa818_busy = 0;
static uint32_t sa818_time = 0;
static const char *sa818_next_tone = NULL;

/* ================= LOW LEVEL FUNCTIONS ================= */

static void cs_low(void)
{
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);
}

static void cs_high(void)
{
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
}

/* ================= SPI ================= */

uint8_t SX1278_Read(uint8_t addr)
{
    uint8_t tx[2] = { addr & 0x7F, 0x00 };
    uint8_t rx[2];

    cs_low();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 100);
    cs_high();

    return rx[1];
}

void SX1278_Write(uint8_t addr, uint8_t val)
{
    uint8_t tx[2] = { addr | 0x80, val };
    uint8_t rx[2];

    cs_low();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 100);
    cs_high();
}

/* ================= MODE CONTROL ================= */

static void SX1278_Standby(void)
{
	SX1278_Write(0x01, 0x80); // sleep
   HAL_Delay(1);

   SX1278_Write(0x01, 0x81); // standby
   HAL_Delay(1);
}
void SX1278_SetTx(void)
{
    SX1278_Standby();
    HAL_GPIO_WritePin(TXEN_PORT, TXEN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RXEN_PORT, RXEN_PIN, GPIO_PIN_RESET);
    SX1278_Write(0x01, 0x83); // TX
    HAL_Delay(2);
}

void SX1278_SetRx(void)
{
    SX1278_Standby();
    HAL_GPIO_WritePin(TXEN_PORT, TXEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RXEN_PORT, RXEN_PIN, GPIO_PIN_SET);
    SX1278_Write(0x01, 0x85); // RX continuous
    HAL_Delay(2);
}

/* ================= INIT ================= */

void SX1278_Init(void)
{
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(10);

    SX1278_Write(REG_OP_MODE, 0x80); // LoRa
    SX1278_Write(REG_OP_MODE, 0x81); // Standby

    SX1278_Write(REG_FRF_MSB, 0x6C);
    SX1278_Write(REG_FRF_MID, 0x80);
    SX1278_Write(REG_FRF_LSB, 0x00);

    SX1278_Write(REG_PA_CONFIG, 0xFF);
    SX1278_Write(REG_MODEM_CONFIG_1, 0x72);
    SX1278_Write(REG_MODEM_CONFIG_2, 0x74);

    SX1278_Write(REG_FIFO_TX_BASE_ADDR, 0x00);
    SX1278_Write(REG_FIFO_RX_BASE_ADDR, 0x00);

    SX1278_Write(REG_DIO_MAPPING_1, 0x00);

    SX1278_SetRx();
    __NOP();
}

/* ================= TX ================= */

void SX1278_Send(uint8_t *data, uint8_t len)
{
    SX1278_Standby();

    SX1278_Write(0x12, 0xFF);     // clear IRQ
    SX1278_Write(0x0D, 0x00);     // FIFO ptr

    for(uint8_t i = 0; i < len; i++)
    	SX1278_Write(0x00, data[i]);

    SX1278_Write(0x22, len);

    SX1278_SetTx();
}

void Slave_App_Init(void)
{
    SX1278_Init();
    Slave_ACS712_Calibrate();
}

void process_command(uint8_t cmd, uint16_t uid)
{
    /* ================= BROADCAST HANDLING ================= */
    if(uid == UID_BROADCAST)
    {
        if(!broadcast_active)
        {
            broadcast_active = 1;
            sa818_next_tone = broadcast_tone;
            sa818_busy = 1;
            sa818_time = HAL_GetTick();
        }
    }
    else
    {
        if(broadcast_active)
        {
            broadcast_active = 0;
            sa818_next_tone = robot_tone;
            sa818_busy = 1;
            sa818_time = HAL_GetTick();
        }
    }

    /* ================= APPLY CONTROL ================= */
    if(cmd == CMD_MOTOR_ON)
    {
        motor_state = 1;
        last_rx_time = HAL_GetTick();
    }
    else if(cmd == CMD_MOTOR_OFF)
    {
        motor_state = 0;
    }
    else if(cmd == CMD_TELEMETRY_REQ)
    {
        telemetry_req_time = HAL_GetTick();
        telemetry_request = 1;
    }
}

void SA818_Task(void)
{
    if(sa818_busy)
    {
        if(sa818_next_tone != NULL)
        {
            SA818_SetTone(sa818_next_tone);
            sa818_next_tone = NULL;
            sa818_time = HAL_GetTick();
        }
        else if(HAL_GetTick() - sa818_time > 200)
        {
            sa818_busy = 0;
        }
    }
}

void MOTOR_On(void)
{
    HAL_GPIO_WritePin(MOTOR_H_PORT, MOTOR_INH_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_IN_R_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_IN_L_PIN, GPIO_PIN_RESET);
}

void MOTOR_Off(void)
{
    HAL_GPIO_WritePin(MOTOR_H_PORT, MOTOR_INH_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_IN_R_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_IN_L_PIN, GPIO_PIN_RESET);
}

void Motor_Task(void)
{
    uint32_t now = HAL_GetTick();

    if((now - last_rx_time) > MOTOR_FAILSAFE_TIMEOUT)
    {
        motor_state = 0;
    }

    if(motor_state)
        MOTOR_On();
    else
        MOTOR_Off();
}

void send_gps_packet(void)
{
    if(tx_pending) return;

    uint8_t packet[8];
    uint16_t uid = ROBOT_UID;

    uint8_t fix = GPS_GetFixStatus();
    uint8_t speed = (uint8_t)GPS_GetFilteredSpeed();
    uint16_t dist_scaled = (uint16_t)(GPS_GetDistanceMeters() / 2.0f);
    uint8_t batt = Slave_Battery_GetPercent();

    packet[0] = uid >> 8;
    packet[1] = uid & 0xFF;
    packet[2] = fix;
    packet[3] = speed;
    packet[4] = dist_scaled >> 8;
    packet[5] = dist_scaled & 0xFF;
    packet[6] =  batt;

    uint8_t crc = 0;
    for(int i = 0; i < 7; i++)
        crc ^= packet[i];
    packet[7] = crc;

    // Use the SX1278_Send function
    SX1278_Send(packet, 8);
}

void Radio_Task(void)
{
    // Read IRQ flags
    uint8_t irq = SX1278_Read(0x12);

    if(irq)
    {
        // Clear IRQ flags
    	SX1278_Write(0x12, 0xFF);

        // TX done (bit 3)
        if(irq & 0x08)
        {
            tx_pending = 0;
            SX1278_SetRx();
        }

        // RX done (bit 6)
        if(irq & 0x40)
        {
            SX1278_HandleRx();
        }
    }

    // TX timeout
    if(tx_pending && (HAL_GetTick() - tx_start_time > 500))
    {
        tx_pending = 0;
        SX1278_SetRx();
    }
}

void SX1278_HandleRx(void)
{
    uint8_t len = SX1278_Read(0x13);   // RxNbBytes
    uint8_t addr = SX1278_Read(0x10);  // FifoRxCurrentAddr

    // Set FIFO pointer to current RX address
    SX1278_Write(0x0D, addr);

    uint8_t data[16];

    // Read data from FIFO
    for(uint8_t i = 0; i < len && i < 16; i++)
        data[i] = SX1278_Read(0x00);

    if(len == EXPECTED_LEN)
    {
        uint16_t sync = (data[0] << 8) | data[1];
        uint16_t uid = (data[2] << 8) | data[3];

        if(sync == LORA_SYNC_WORD &&
           (uid == ROBOT_UID || uid == UID_BROADCAST))
        {
            uint8_t crc = 0;
            for(int i = 0; i < 7; i++)
                crc ^= data[i];

            if(crc == data[7])
            {
                process_command(data[4], uid);
                last_rx_time = HAL_GetTick();
            }
        }
    }
}

void Telemetry_Task(void)
{
    static uint32_t last_tx = 0;

    if(tx_pending) return;

    // Periodic TX every 1 second
    if(HAL_GetTick() - last_tx > 1000)
    {
        last_tx = HAL_GetTick();
        send_gps_packet();
        return;
    }

    // Request-based TX
    if(telemetry_request)
    {
        if(HAL_GetTick() - telemetry_req_time < TELEMETRY_TX_DELAY_MS)
            return;

        telemetry_request = 0;
        send_gps_packet();
    }
}
