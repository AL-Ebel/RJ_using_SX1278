/*
 * protocol.h
 *
 *  Created on: Feb 12, 2026
 *      Author: HP
 */

#ifndef INC_PROTOCOL_H_
#define INC_PROTOCOL_H_

#include <stdint.h>

#define FRAME_PREAMBLE 0xAA
#define FRAME_VERSION  0x01

#define FRAME_TYPE_CMD  0x01
#define FRAME_TYPE_TLM  0x02

#define CMD_BROADCAST_ENTER   0x20
#define CMD_BROADCAST_EXIT    0x21

typedef struct __attribute__((packed))
{
    uint8_t  preamble;   // 1 byte
    uint8_t  version;    // 1 byte
    uint16_t uid;        // 2 bytes
    uint8_t  type;       // 1 byte
    uint8_t  payload;    // 1 byte
    uint8_t  crc;        // 1 byte
} RobotFrame_t;  // Total = 7 bytes!


uint8_t protocol_crc(uint8_t *data, uint8_t len);

#endif /* INC_PROTOCOL_H_ */
