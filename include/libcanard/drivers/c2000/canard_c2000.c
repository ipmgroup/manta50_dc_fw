/*
 * Copyright (c) 2018 Somebody else...
 * Copyright (c) 2016 UAVCAN Team
 *
 * Distributed under the MIT License, available in the file LICENSE.
 *
 */

#define HAS_CAN_CONFIG_H

#include <canard_c2000.h>
#include <string.h>

#include <can.h>

int canardC2000Init(uint32_t bitrate) {
    // TODO
    (void)bitrate;
    return 0;
}

int canardC2000Close(void) {
    return 0;
}

int canardC2000Transmit(const CanardCANFrame* frame) {
    return can_write(frame->id, (void*)frame->data, frame->data_len);
}

int canardC2000TransmitN(const CanardCANFrame* frame, uint8_t num) {
    return can_writeN(frame->id, (void*)frame->data, frame->data_len, num);
}

int canardC2000Receive(CanardCANFrame* out_frame) {
    uint8_t res = can_read(&out_frame->id, (void*)out_frame->data, &out_frame->data_len);

    if (res == 1) {
        // TODO Make it properly!
        if (out_frame->id > 0b0000011111111111)
            out_frame->id |= CANARD_CAN_FRAME_EFF;
    }

    return res;
}

int canardC2000ReceiveN(CanardCANFrame* out_frame, uint8_t num) {
    uint8_t res = can_read_N(&out_frame->id, (void*)out_frame->data, &out_frame->data_len, num);
    if (res == 1) {
        if (out_frame->id > 0b0000011111111111)
            out_frame->id |= CANARD_CAN_FRAME_EFF;
    }
    return res;
}

int canardC2000ConfigureAcceptanceFilters(uint8_t node_id) {
    // TODO
    (void)node_id;
    return 0;
}
