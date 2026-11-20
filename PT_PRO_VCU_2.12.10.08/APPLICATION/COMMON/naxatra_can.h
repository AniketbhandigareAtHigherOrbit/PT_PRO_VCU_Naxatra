#ifndef NAXATRA_CAN_H
#define NAXATRA_CAN_H
#include "vehicle_state.h"
#define NAXATRA_STATUS1_ID (0x411UL)
#define NAXATRA_STATUS2_ID (0x412UL)
#define NAXATRA_DQ_CURRENT_ID (0x233UL)
#define NAXATRA_COMMAND_ID_ROT (0x415UL)
#define NAXATRA_COMMAND_ID_LFT (0x414UL)
#define NAXATRA_COMMAND_ID_RIG (0x416UL)
/* Preserve the existing 20 ms VCU command task period. The R0 DBC has
 * no GenMsgCycleTime attribute; change this only after timing validation. */
#define NAXATRA_TX_PERIOD_MS (20U)
/* One physical CAN channel per controller. Feedback IDs are identical
 * (0x411/0x412/0x233), so RX must distinguish each controller by channel. */
#define NAXATRA_LEFT_CHANNEL (2U)
#define NAXATRA_RIGHT_CHANNEL (0U)
#define NAXATRA_ROTOR_CHANNEL (1U)
/* NXT_controller_20112026_R0.dbc: 0x414/0x415/0x416, byte 5/6,
 * 0.1 second/count.
 * Keep the machine's existing 0.2-second tuning until ramp testing. */
#define NAXATRA_ACCEL_TIME_DS (2U)
#define NAXATRA_DECEL_TIME_DS (2U)
/* VcuDcFdbCurrent, command byte 7. Field-weakening disabled by default. */
#define NAXATRA_DC_FDB_CURRENT_RAW (0U)
#define NAXATRA_STARTUP_ZERO_MS (500U)

/* 0x411 Tx411 bit definitions in the R0 DBC. */
#define NAXATRA_GEAR_MASK                (0xE0U)
#define NAXATRA_GEAR_SHIFT               (5U)
#define NAXATRA_GEAR_PARK                (0U)
#define NAXATRA_GEAR_1                   (1U)
#define NAXATRA_GEAR_3                   (3U)
#define NAXATRA_REVERSE_STATUS_MASK      (0x10U)
/* Controller faults: byte 4, all bits now defined (bit 0 = lost phase). */
#define NAXATRA_CONTROLLER_FAULT_MASK    (0xFFU)
/* Motor flags: byte 5 bits 7..4; bit 0 is CAN reverse lost. */
#define NAXATRA_MOTOR_FAULT_MASK         (0xF0U)
#define NAXATRA_CAN_REVERSE_LOST_MASK    (0x01U)
void Naxatra_CAN_SendCommand(ControllerIndex_t index, uint16_t rpm,
                           uint8_t enable, uint8_t direction, uint8_t brake);
void Naxatra_CAN_BuildCommand(uint8_t *data, uint16_t rpm,
                             uint8_t enable, uint8_t direction, uint8_t brake);

#endif
