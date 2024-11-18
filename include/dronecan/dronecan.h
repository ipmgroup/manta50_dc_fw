#ifndef _DRONECAN_H_
#define _DRONECAN_H_

#include <canard_c2000.h>  // CAN backend driver for C2000 DSP

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <can.h>
#include "main.h"
#include "dronecan_msgs.h"
#include <math.h>

typedef struct {
    uint64_t last_tx_delay_time;
    uint64_t tx_delay_duration_us;
    int error;
} DeviceContext;

typedef enum {
    PHASE_IDLE,
    PHASE_SAVING
} ProfileState;

typedef union {
    float_t value;
    struct {
        uint32_t mantissa : 23;
        uint32_t exponent : 8;
        uint32_t sign : 1;
    } parts;
} FloatUnion;

void canard_init();
void canard_update();

#endif
