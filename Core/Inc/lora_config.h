/*
 * lora_config.h
 *
 *  Created on: Feb 12, 2026
 *      Author: HP
 */

#ifndef INC_LORA_CONFIG_H_
#define INC_LORA_CONFIG_H_


#define LORA_FREQUENCY     433000000UL

#define UID_BROADCAST    0xFFFF

#define LORA_SYNC_WORD     0x1424   // Private network
#define LORA_SF            0x07     // SF9
#define LORA_BW            0x04     // 125kHz
#define LORA_CR            0x01     // 4/5

#define LORA_PREAMBLE_MSB  0x00
#define LORA_PREAMBLE_LSB  0x08     // 12 symbols

#endif /* INC_LORA_CONFIG_H_ */
