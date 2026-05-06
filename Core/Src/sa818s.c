

#include "sa818s.h"
extern UART_HandleTypeDef huart3;
static uint8_t rxBuffer[200];

static void SA818_Send(char *cmd)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)cmd, strlen(cmd), SA818_TIMEOUT);
}

HAL_StatusTypeDef SA818_WaitFor(char *expected)
{
    uint8_t ch;
    uint16_t idx = 0;
    uint32_t start = HAL_GetTick();

    memset(rxBuffer, 0, sizeof(rxBuffer));

    while ((HAL_GetTick() - start) < 2000)
    {
        if (HAL_UART_Receive(&huart3, &ch, 1, 10) == HAL_OK)
        {
            if (idx < sizeof(rxBuffer)-1)
            {
                rxBuffer[idx++] = ch;
                rxBuffer[idx] = 0;

                if (strstr((char*)rxBuffer, expected) != NULL)
                    return HAL_OK;
            }
        }
    }
    return HAL_TIMEOUT;
}


static HAL_StatusTypeDef SA818_Handshake(void)
{
    for(int i = 0; i < 10; i++)
    {
        SA818_Send("AT+DMOCONNECT\r\n");
        if(SA818_WaitFor("+DMOCONNECT:0") == HAL_OK)
            return HAL_OK;
        HAL_Delay(500);
    }
    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef SA818_SetGroup(void)
{
    for(int i = 0; i < 3; i++)
    {
        // squelch=1 on slave for better sensitivity at distance
        // TX/RX CTCSS=0000 here, real RX filter set by SA818_SetTone later
        SA818_Send("AT+DMOSETGROUP=0,433.6500,433.6500,0000,1,0000\r\n");
        if(SA818_WaitFor("+DMOSETGROUP:0") == HAL_OK)
            return HAL_OK;
        HAL_Delay(100);
    }
    return HAL_TIMEOUT;
}

 HAL_StatusTypeDef SA818_SetVolume(void)
{
    for(int i = 0; i < 3; i++)
    {
        SA818_Send("AT+DMOSETVOLUME=8\r\n");
        if(SA818_WaitFor("+DMOSETVOLUME:0") == HAL_OK)
           return HAL_OK;
        HAL_Delay(100);
    }
    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef SA818_enableTail(void)
{
    for(int i = 0; i < 3; i++)
    {
        SA818_Send("AT+SETTAIL=1\r\n");
        if(SA818_WaitFor("+DMOSETTAIL:0") == HAL_OK)
            return HAL_OK;
        HAL_Delay(100);
    }
    return HAL_TIMEOUT;
}

HAL_StatusTypeDef SA818_SetTone(const char *tone)
{
    char cmd[80];

    // TX=0000 slave doesn't need TX tone
    // RX=tone sets the CTCSS filter to only open for master's tone
    // squelch=1 consistent with SetGroup
    snprintf(cmd, sizeof(cmd),
        "AT+DMOSETGROUP=0,433.6500,433.6500,0000,1,%s\r\n",
        tone);

    for(int i = 0; i < 3; i++)
    {
        SA818_Send(cmd);
        if(SA818_WaitFor("+DMOSETGROUP:0") == HAL_OK)
            return HAL_OK;
        HAL_Delay(200);
    }
    return HAL_ERROR;
}



HAL_StatusTypeDef SA818_Init(const char *initial_tone)
{
    HAL_Delay(300);
    if(SA818_Handshake()           != HAL_OK) { while(1); }
    if(SA818_SetGroup()            != HAL_OK) { while(1); }
    if(SA818_SetVolume()           != HAL_OK) { while(1); }
    if(SA818_enableTail()          != HAL_OK) { while(1); }
    if(SA818_SetTone(initial_tone) != HAL_OK) { while(1); }
    HAL_Delay(100);


    return HAL_OK;
}

void SA818_SetHighPower(void)
{
    HAL_GPIO_WritePin(SA818_HL_PORT, SA818_HL_PIN, GPIO_PIN_SET);
}

void SA818_SetLowPower(void)
{
    HAL_GPIO_WritePin(SA818_HL_PORT, SA818_HL_PIN, GPIO_PIN_RESET);
}
