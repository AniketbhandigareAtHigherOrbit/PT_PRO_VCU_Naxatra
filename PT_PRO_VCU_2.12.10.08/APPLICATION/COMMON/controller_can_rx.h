#ifndef CONTROLLER_CAN_RX_H
#define CONTROLLER_CAN_RX_H

#include "app_typedef.h"
#include "vehicle_state.h"

/*
 * A coherent copy of the controller feedback used by the wheel-direction FSM.
 * msg1_sequence and msg2_sequence increment only after a complete valid frame
 * has been decoded. The snapshot reader uses an internal sequence lock so a
 * task cannot observe a partly updated 16-bit RPM/current value.
 */
typedef struct
{
    uint16_t speed_rpm;
    int16_t  motor_current_A;
    uint16_t battery_voltage_dV;
    int32_t  bus_current_dA;
    uint16_t phase_current_RMS_A;
    uint16_t error_code;

    int16_t  controller_temp_C;
    int16_t  motor_temp_C;
    uint8_t  feedback_dir;
    uint8_t  command_dir;
    uint8_t  switch_status;

    uint8_t  msg1_received;
    uint8_t  msg2_received;
    uint8_t  msg1_sequence;
    uint8_t  msg2_sequence;
    uint16_t msg1_age_ms;
    uint16_t msg2_age_ms;
    /* Naxatra rotor 0x233 Id/Iq measurements and independent freshness. */
    int16_t  iq_current_A;
    int16_t  id_current_A;
    uint8_t  msg3_received;
    uint8_t  msg3_sequence;
    uint16_t msg3_age_ms;
} ControllerFeedbackSnapshot_t;

void Controller_CAN_RxCallback(uint8_t CanNum_u8,
                               uint8_t Channel_u8,
                               uint8_t *DataBuff_pu8,
                               uint8_t Dlc_u8,
                               uint8_t CanMode_u8,
                               uint8_t Ide_u8,
                               uint32_t CanId_u32);

/* Call once every machine-drive task, using the real task period. */
void Controller_CAN_TimeoutTask(uint16_t task_period_ms);

/* Returns 1 only when index/output are valid. Freshness is reported in ages. */
uint8_t Controller_CAN_GetSnapshot(ControllerIndex_t index,
                                   ControllerFeedbackSnapshot_t *snapshot);

#endif /* CONTROLLER_CAN_RX_H */
