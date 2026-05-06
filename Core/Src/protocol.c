/*
 * protocol.c
 *
 *  Created on: Feb 12, 2026
 *      Author: HP
 */


#include "protocol.h"

uint8_t protocol_crc(uint8_t *data, uint8_t len)
{
    uint8_t c = 0;
    for(int i = 0; i < len; i++)
        c ^= data[i];
    return c;
}
