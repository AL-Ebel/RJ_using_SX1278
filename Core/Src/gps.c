#include "gps.h"
#include "slave_side.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_hal.h"

/* ================== CONFIG ================== */
#define GPS_DMA_BUFFER_SIZE 256
#define ALPHA 0.2f

/* ================== UART ================== */
static UART_HandleTypeDef *gps_uart;

/* ================== DMA ================== */
uint8_t gps_dma_buffer[GPS_DMA_BUFFER_SIZE];
static uint16_t old_pos = 0;
extern uint8_t current_robot_index;
/* ================== Parser ================== */
static char line[128];
static volatile uint8_t indexx = 0;
static volatile uint8_t capturing = 0;
volatile uint8_t sentence_ready = 0;

/* ================== GPS RAW ================== */
static double gps_latitude = 0.0;
static double gps_longitude = 0.0;

static uint8_t gps_fix_status = 0;
static uint8_t gps_satellites = 0;
static uint8_t gps_best_snr = 0;

/* ================== MOTION ================== */
static double filtered_lat = 0.0;
static double filtered_lon = 0.0;

static double prev_lat = 0.0;
static double prev_lon = 0.0;

static double robot_distance[MAX_ROBOTS] = {0};
static float speed_kmh_filtered = 0.0;

static uint8_t first_fix = 1;
static uint32_t last_motion_tick = 0;
volatile uint8_t pps_flag = 0;

/* ============================================================ */
/* ================== HELPER FUNCTIONS ========================= */
/* ============================================================ */

static char* goto_field(char *s, uint8_t field)
{
    for (uint8_t i = 0; i < field; i++)
    {
        s = strchr(s, ',');
        if (!s) return NULL;
        s++;
    }
    return s;
}

static float haversine(double lat1, double lon1, double lat2, double lon2)
{
    const float R = 6371000.0f;

    float dLat = (lat2 - lat1) * 0.0174533f;
    float dLon = (lon2 - lon1) * 0.0174533f;

    float a = sinf(dLat/2)*sinf(dLat/2) +
              cosf(lat1*0.0174533f) * cosf(lat2*0.0174533f) *
              sinf(dLon/2)*sinf(dLon/2);

    float c = 2 * atan2f(sqrtf(a), sqrtf(1-a));
    return R * c;
}

/* ============================================================ */
/* ================== PARSERS ================================= */
/* ============================================================ */

static void parse_rmc(char *sentence)
{
    char *status = goto_field(sentence, 2);
    char *lat    = goto_field(sentence, 3);
    char *lat_d  = goto_field(sentence, 4);
    char *lon    = goto_field(sentence, 5);
    char *lon_d  = goto_field(sentence, 6);

    if (!(status && status[0] == 'A'))
        return;

    if (lat && lat_d)
    {
        double raw = atof(lat);
        int deg = (int)(raw / 100);
        double min = raw - deg * 100;
        gps_latitude = deg + min / 60.0;

        if (lat_d[0] == 'S') gps_latitude = -gps_latitude;
    }

    if (lon && lon_d)
    {
        double raw = atof(lon);
        int deg = (int)(raw / 100);
        double min = raw - deg * 100;
        gps_longitude = deg + min / 60.0;

        if (lon_d[0] == 'W') gps_longitude = -gps_longitude;
    }
}

static void parse_gga(char *sentence)
{
    char *fix = goto_field(sentence, 6);
    char *sat = goto_field(sentence, 7);

    if (fix) gps_fix_status = (uint8_t)atoi(fix);
    if (sat) gps_satellites = (uint8_t)atoi(sat);
}

static void parse_gsv(char *sentence)
{
    char *p = sentence;
    uint8_t field = 0;

    while((p = strchr(p, ',')) != NULL)
    {
        field++;
        p++;

        if(field >= 7 && (field - 7) % 4 == 0)
        {
            uint8_t snr = (uint8_t)atoi(p);
            if(snr > gps_best_snr)
                gps_best_snr = snr;
        }
    }
}

/* ============================================================ */
/* ================== INIT ==================================== */
/* ============================================================ */

void GPS_Init(UART_HandleTypeDef *huart)
{
    gps_uart = huart;
    HAL_UART_Receive_DMA(gps_uart, gps_dma_buffer, GPS_DMA_BUFFER_SIZE);
}

/* ============================================================ */
/* ================== MAIN PROCESS ============================ */
/* ============================================================ */

void GPS_Process(void)
{
    if (!sentence_ready) return;
    sentence_ready = 0;

    char temp[128];
    strncpy(temp, line, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';

    if (strncmp(temp, "$GNRMC", 6) == 0) parse_rmc(temp);
    else if (strncmp(temp, "$GNGGA", 6) == 0) parse_gga(temp);
    else if (strncmp(temp, "$GNGSV", 6) == 0) parse_gsv(temp);

    /* ===== MOTION TRACKING ===== */

    if (gps_fix_status == 0 || gps_satellites < 5)
        return;

    if (first_fix)
    {
        filtered_lat = gps_latitude;
        filtered_lon = gps_longitude;

        prev_lat = filtered_lat;
        prev_lon = filtered_lon;

        last_motion_tick = HAL_GetTick();
        first_fix = 0;
        return;
    }

    /* Filter */
    filtered_lat += ALPHA * (gps_latitude - filtered_lat);
    filtered_lon += ALPHA * (gps_longitude - filtered_lon);

    /* Distance */
    float d = haversine(prev_lat, prev_lon, filtered_lat, filtered_lon);

    uint32_t now = HAL_GetTick();
    float dt = (now - last_motion_tick) / 1000.0f;

    if (d > 0.5f && d < 10.0f && dt > 0.5f && dt < 2.0f)
    {
    	robot_distance[current_robot_index] += d;

        float speed_mps = d / dt;

        /* Speed clamp */
        if (speed_mps < 60.0f)
            speed_kmh_filtered = speed_mps * 3.6f;
        else
            speed_kmh_filtered = 0.0f;

        prev_lat = filtered_lat;
        prev_lon = filtered_lon;
        last_motion_tick = now;
    }
}

/* ============================================================ */
/* ================== DMA ===================================== */
/* ============================================================ */

void GPS_DMA_Process(void)
{
    uint16_t new_pos = GPS_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(gps_uart->hdmarx);

    if (new_pos != old_pos)
    {
        if (new_pos > old_pos)
        {
            for (uint16_t i = old_pos; i < new_pos; i++)
                GPS_Parse_Byte(gps_dma_buffer[i]);
        }
        else
        {
            for (uint16_t i = old_pos; i < GPS_DMA_BUFFER_SIZE; i++)
                GPS_Parse_Byte(gps_dma_buffer[i]);

            for (uint16_t i = 0; i < new_pos; i++)
                GPS_Parse_Byte(gps_dma_buffer[i]);
        }

        old_pos = new_pos;
    }
}

void GPS_Parse_Byte(uint8_t byte)
{
    if (byte == '$')
    {
        indexx = 0;
        capturing = 1;
}

    if (capturing)
    {
        if (indexx < sizeof(line) - 1)
        {
            line[indexx++] = byte;
        }

        if (byte == '\n')
        {
            line[indexx] = '\0';

            if (indexx > 5 && line[0] == '$')
            {
                sentence_ready = 1;
            }

            indexx = 0;
            capturing = 0;
        }
    }
}

/* PPS rising edge */
void GPS_PPS_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13)
    {
        pps_flag = 1;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == gps_uart)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        // DO NOT restart DMA here
    }
}

/* ============================================================ */
/* ================== GETTERS ================================= */
/* ============================================================ */float GPS_GetFilteredSpeed(void)
{ return speed_kmh_filtered; }

double GPS_GetDistanceMeters(void)
{
    return robot_distance[current_robot_index];
}
uint8_t GPS_GetSatelliteCount(void) { return gps_satellites; }
uint8_t GPS_GetFixStatus(void) { return gps_fix_status; }
uint8_t GPS_GetBestSNR(void) { return gps_best_snr; }
