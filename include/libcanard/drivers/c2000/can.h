#ifndef __CAN_H__
#define __CAN_H__

#include <stdint.h>

#ifndef uint8_t
#define uint8_t uint_least8_t
#endif

#ifndef int8_t
#define int8_t int_least8_t
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CAN_5KBPS
#define CAN_5KBPS    1
#define CAN_10KBPS   2
#define CAN_20KBPS   3
#define CAN_40KBPS   4
#define CAN_50KBPS   5
#define CAN_80KBPS   6
#define CAN_100KBPS  7
#define CAN_125KBPS  8
#define CAN_200KBPS  9
#define CAN_250KBPS  10
#define CAN_500KBPS  11
#define CAN_1000KBPS  12
#endif

int8_t can_init(uint32_t speed);
uint8_t can_read(uint32_t *id, void *data, uint8_t *len);
uint8_t can_read_N(uint32_t *id, void *data, uint8_t *len, uint8_t num);
uint8_t can_write(uint32_t id, void *data, uint8_t len);
uint8_t can_writeN(uint32_t id, void *data, uint8_t len, uint8_t num);


#ifdef __cplusplus
}
#endif

#endif
