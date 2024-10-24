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

//typedef struct {
//    HAL_Handle* halHandle;
//    MOTOR_Vars_t* motorVars;
//    USER_Params* userParams;
//    CanardInstance canard;
//    uint8_t canard_memory_pool[400];
//    uint8_t previousUserErrorCode;
//    uint8_t previousCtrlState;
//    uint8_t previousEstState;
//    bool Motor_Auto_ID;
//    uint8_t req_index;
//    uint8_t setter;
//} DroneCAN;

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

// static void setcw(uint16_t *data)

#endif
