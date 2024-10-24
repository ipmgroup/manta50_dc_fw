/*
 * Copyright (c) 2018 Somebody else...
 * Copyright (c) 2016 UAVCAN Team
 *
 * Distributed under the MIT License, available in the file LICENSE.
 *
 */

#ifndef CANARD_C2000_H
#define CANARD_C2000_H

#include <canard.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @ingroup canard_c2000_interface
 * @brief Deinitialize CAN interface on C2000 DSP.
 * @warning Not implemented
 *
 * @retval 1     Initialisation successful
 * @retval -1    Error, bitrate not supported
 */
int canardC2000Close(void);

/**
 * TODO Implement properly!
 *
 * @ingroup canard_c2000_interface
 * @brief Transmits a CanardCANFrame to the CAN device.
 * @warning Not implemented properly
 *
 * @param [in] frame  Canard CAN frame which contains the data to send
 *
 * @retval     0      No CAN send buffer free
 * @retval     -1     Error, data could not be sent
 * @retval     1      Data sent successful
 */
int canardC2000Transmit(const CanardCANFrame* frame);
int canardC2000TransmitN(const CanardCANFrame* frame, uint8_t num);

/**
 * TODO Implement properly!
 *
 * @ingroup canard_c2000_interface
 * @brief Receives a CanardCANFrame from the CAN device.
 * @warning Not implemented properly
 *
 * @param [out] out_frame  Canard CAN frame which contains data received
 *
 * @retval      0          No new CAN data to be read
 * @retval      -1         Error, data could not be read
 * @retval      1          Data read successful
 */
int canardC2000Receive(CanardCANFrame* out_frame);
int canardC2000ReceiveN(CanardCANFrame* out_frame, uint8_t num);

/**
 * TODO Implement!
 *
 * @ingroup canard_c2000_interface
 * @brief Set hardware acceptance filters for specific node ID
 * @warning Not implemented
 *
 * @param [in] id  node ID for hardware filter
 *
 * @retval      -1         Error, filters could no be set
 * @retval      1          Set filter successful
 */
int canard2000ConfigureAcceptanceFilters(uint8_t node_id);

#ifdef __cplusplus
}
#endif

#endif
