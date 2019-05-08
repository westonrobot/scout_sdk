/*
 * Copyright (c) 2016 UAVCAN Team
 *
 * Distributed under the MIT License, available in the file LICENSE.
 *
 */

#ifndef SOCKETCAN_H
#define SOCKETCAN_H

// #include <canard.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/// This will be changed when the support for CAN FD is added
#define CANARD_CAN_FRAME_MAX_DATA_LEN               8

/**
 * This data type holds a standard CAN 2.0B data frame with 29-bit ID.
 */
typedef struct
{
    /**
     * Refer to the following definitions:
     *  - CANARD_CAN_FRAME_EFF
     *  - CANARD_CAN_FRAME_RTR
     *  - CANARD_CAN_FRAME_ERR
     */
    uint32_t id;
    uint8_t data[CANARD_CAN_FRAME_MAX_DATA_LEN];
    uint8_t data_len;
} CanardCANFrame;

typedef struct
{
    int fd;
} SocketCANInstance;

/**
 * Initializes the SocketCAN instance.
 */
int socketcanInit(SocketCANInstance* out_ins, const char* can_iface_name);

/**
 * Deinitializes the SocketCAN instance.
 */
int socketcanClose(SocketCANInstance* ins);

/**
 * Transmits a CanardCANFrame to the CAN socket.
 * Use negative timeout to block infinitely.
 */
int socketcanTransmit(SocketCANInstance* ins, const CanardCANFrame* frame, int timeout_msec);

/**
 * Receives a CanardCANFrame from the CAN socket.
 * Use negative timeout to block infinitely.
 */
int socketcanReceive(SocketCANInstance* ins, CanardCANFrame* out_frame, int timeout_msec);

/**
 * Returns the file descriptor of the CAN socket.
 */
int socketcanGetSocketFileDescriptor(const SocketCANInstance* ins);

#ifdef __cplusplus
}
#endif

#endif
