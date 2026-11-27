#ifndef NAXATRA_CAN_H
#define NAXATRA_CAN_H
#include "vehicle_state.h"
#define NAXATRA_STATUS1_ID (0x411UL)
#define NAXATRA_STATUS2_ID (0x412UL)
#define NAXATRA_COMMAND_ID (0x415UL)
#define NAXATRA_TX_PERIOD_MS (20U)
/* Independent physical buses: standard IDs contain no controller address. */
#define NAXATRA_LEFT_CHANNEL (2U)
#define NAXATRA_RIGHT_CHANNEL (0U)
#define NAXATRA_ROTOR_CHANNEL (1U)
/* Log-supported correction to overlapping v05 DBC fields: ramps at bytes 5/6.
 * Confirm independently with Naxatra; both were 0x14 in working log frames. */
#define NAXATRA_ACCEL_TIME_DS (2U)
#define NAXATRA_DECEL_TIME_DS (2U)
#define NAXATRA_STARTUP_ZERO_MS (500U)
void Naxatra_CAN_SendCommand(ControllerIndex_t index, uint16_t rpm,
                           uint8_t enable, uint8_t direction, uint8_t brake);
void Naxatra_CAN_BuildCommand(uint8_t *data, uint16_t rpm,
                             uint8_t enable, uint8_t direction, uint8_t brake);

#endif
