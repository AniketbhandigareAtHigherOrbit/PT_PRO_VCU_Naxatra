#include "controller_can_rx.h"
#include "vehicle_config.h"
#include "naxatra_can.h"

/* ========================================================= */
#define CTRL_MSG1_BASE_ID      (0x0CF11E00UL)
#define CTRL_MSG2_BASE_ID      (0x0CF11F00UL)

#define CTRL_ID_LEFT           (4U)
#define CTRL_ID_RIGHT          (6U)
#define CTRL_ID_ROTOR          (5U)

#ifndef VC_CONTROLLER_MSG1_TIMEOUT_MS
#define VC_CONTROLLER_MSG1_TIMEOUT_MS   (300U)
#endif

#ifndef VC_CONTROLLER_MSG2_TIMEOUT_MS
#define VC_CONTROLLER_MSG2_TIMEOUT_MS   (600U)
#endif

#define CTRL_RX_SNAPSHOT_RETRIES         (4U)
#define CTRL_AGE_SATURATION_MS           (60000U)

/*
 * Sequence-lock guards:
 * even = stable, odd = writer updating.
 * The separate update sequence is used by the drive FSM to detect a genuinely
 * new feedback frame instead of counting the same feedback every 20 ms.
 */
static volatile uint8_t  s_msg1_guard[CTRL_COUNT];
static volatile uint8_t  s_msg2_guard[CTRL_COUNT];
static volatile uint8_t  s_msg3_guard[CTRL_COUNT];
static volatile uint8_t  s_msg1_update_sequence[CTRL_COUNT];
static volatile uint8_t  s_msg2_update_sequence[CTRL_COUNT];
static volatile uint8_t  s_msg3_update_sequence[CTRL_COUNT];
static volatile uint16_t s_msg1_age_ms[CTRL_COUNT];
static volatile uint16_t s_msg2_age_ms[CTRL_COUNT];
static volatile uint16_t s_msg3_age_ms[CTRL_COUNT];
static volatile uint8_t  s_msg1_received[CTRL_COUNT];
static volatile uint8_t  s_msg2_received[CTRL_COUNT];
static volatile uint8_t  s_msg3_received[CTRL_COUNT];

/* ========================================================= */
static uint16_t Saturating_Add_ms(uint16_t value, uint16_t increment)
{
    uint32_t next_value;

    next_value = (uint32_t)value + (uint32_t)increment;

    if (next_value >= (uint32_t)CTRL_AGE_SATURATION_MS)
    {
        return CTRL_AGE_SATURATION_MS;
    }

    return (uint16_t)next_value;
}

/* The R0 DBC specifies signed Motorola 16-bit Id/Iq signals (0x233).
 * Subtract in 32-bit arithmetic to avoid an implementation-defined
 * out-of-range uint16_t-to-int16_t conversion. */
static int16_t Decode_S16_BE(const uint8_t *data)
{
    uint16_t raw = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);

    if ((raw & 0x8000U) != 0U)
    {
        return (int16_t)((int32_t)raw - 65536L);
    }

    return (int16_t)raw;
}

static uint8_t Get_ControllerIndex(uint32_t can_id)
{
    uint8_t dev_id;

    dev_id = (uint8_t)(can_id & 0xFFU);

    switch (dev_id)
    {
        case CTRL_ID_LEFT:
            return (uint8_t)CTRL_WHEEL_LEFT;

        case CTRL_ID_RIGHT:
            return (uint8_t)CTRL_WHEEL_RIGHT;

        case CTRL_ID_ROTOR:
            return (uint8_t)CTRL_ROTOR;

        default:
            return (uint8_t)CTRL_COUNT;
    }
}

static void Update_ControllerAlive(uint8_t ctrl_index)
{
    /*
     * MSG1 contains speed and is the essential motion-feedback frame.
     * MSG2 freshness is checked separately by direction acknowledgement.
     */
    if ((s_msg1_received[ctrl_index] != 0U) &&
        (s_msg1_age_ms[ctrl_index] <= VC_CONTROLLER_MSG1_TIMEOUT_MS))
    {
        VS_Controller[ctrl_index].alive = 1U;
    }
    else
    {
        VS_Controller[ctrl_index].alive = 0U;
    }
}

/* ========================================================= */
void Controller_CAN_RxCallback(uint8_t CanNum_u8,
                               uint8_t Channel_u8,
                               uint8_t *DataBuff_pu8,
                               uint8_t Dlc_u8,
                               uint8_t CanMode_u8,
                               uint8_t Ide_u8,
                               uint32_t CanId_u32)
{
    uint8_t  ctrl_index;
    uint32_t msg_base;
    uint16_t speed_rpm;
    uint16_t current_raw;
    uint16_t voltage_raw;
    uint16_t error_code;

    (void)CanNum_u8;

    /*
     * Reject malformed and short frames before touching state.
     * Do not reject only from Ide_u8: on this PAL integration the callback is
     * reached through a casted filter callback and some SDK revisions do not
     * forward the IDE flag consistently. The 29-bit CAN-ID base check below is
     * the authoritative validation.
     */
    if ((DataBuff_pu8 == 0) || (Dlc_u8 < 8U))
    {
        return;
    }

    /* R0 DBC: standard-ID 0x411/0x412/0x233 on CAN1 for the rotor only.
     * Wheel feedback below remains on the legacy extended-ID protocol. */
    if ((Channel_u8 == NAXATRA_ROTOR_CHANNEL) &&
        ((CanId_u32 == NAXATRA_STATUS1_ID) ||
         (CanId_u32 == NAXATRA_STATUS2_ID) ||
         (CanId_u32 == NAXATRA_DQ_CURRENT_ID)))
    {
        ctrl_index = (uint8_t)CTRL_ROTOR;
        if (CanId_u32 == NAXATRA_STATUS1_ID)
        {
            s_msg1_guard[ctrl_index]++;
            VS_Controller[ctrl_index].speed_rpm =
                (uint16_t)(((uint16_t)DataBuff_pu8[0] << 8U) |
                           DataBuff_pu8[1]);
            VS_Controller[ctrl_index].gear_status = DataBuff_pu8[2];
            VS_Controller[ctrl_index].status_feedback_1 = DataBuff_pu8[3];
            VS_Controller[ctrl_index].status_feedback_2 = DataBuff_pu8[4];
            VS_Controller[ctrl_index].status_feedback_3 = DataBuff_pu8[5];
            VS_Controller[ctrl_index].power_mode = DataBuff_pu8[6];
            VS_Controller[ctrl_index].handle_opening = DataBuff_pu8[7];
            /* Low byte: all eight controller-fault bits. R0 assigns bit 0
             * to LostPhaseFault (previous firmware discarded it).
             * High byte: four motor faults plus CANRevLost at bit 0.
             * Other byte-5 low bits are operating states, not faults. */
            VS_Controller[ctrl_index].error_code =
                (uint16_t)((DataBuff_pu8[4] & NAXATRA_CONTROLLER_FAULT_MASK) |
                    ((uint16_t)(DataBuff_pu8[5] &
                      (NAXATRA_MOTOR_FAULT_MASK |
                       NAXATRA_CAN_REVERSE_LOST_MASK)) << 8U));

            /* The gear field, not RPM alone, indicates Park. A stopped
             * rotor can still have first gear selected. Treat unsupported
             * gear codes as unknown/STOP for direction diagnostics. */
            if (((DataBuff_pu8[2] & NAXATRA_GEAR_MASK) >>
                   NAXATRA_GEAR_SHIFT) == NAXATRA_GEAR_PARK)
            {
                VS_Controller[ctrl_index].feedback_dir = DIR_STOP;
            }
            else if (((DataBuff_pu8[2] & NAXATRA_GEAR_MASK) >>
                       NAXATRA_GEAR_SHIFT) > NAXATRA_GEAR_3)
            {
                VS_Controller[ctrl_index].feedback_dir = DIR_STOP;
            }
            else if ((DataBuff_pu8[3] & NAXATRA_REVERSE_STATUS_MASK) != 0U)
            {
                VS_Controller[ctrl_index].feedback_dir = DIR_REVERSE;
            }
            else
            {
                VS_Controller[ctrl_index].feedback_dir = DIR_FORWARD;
            }
            VS_Controller[ctrl_index].switch_status = DataBuff_pu8[3];
            s_msg1_age_ms[ctrl_index] = 0U;
            s_msg1_received[ctrl_index] = 1U;
            s_msg1_update_sequence[ctrl_index]++;
            s_msg1_guard[ctrl_index]++;
            Update_ControllerAlive(ctrl_index);
        }
        else if (CanId_u32 == NAXATRA_STATUS2_ID)
        {
            s_msg2_guard[ctrl_index]++;
            VS_Controller[ctrl_index].controller_temp_C =
                (int16_t)DataBuff_pu8[0] - 40;
            VS_Controller[ctrl_index].motor_temp_C =
                (int16_t)DataBuff_pu8[1] - 40;
            current_raw = (uint16_t)(((uint16_t)DataBuff_pu8[2] << 8U) |
                                     DataBuff_pu8[3]);
            VS_Controller[ctrl_index].bus_current_dA =
                (int32_t)current_raw - 10000L;
            voltage_raw = (uint16_t)(((uint16_t)DataBuff_pu8[4] << 8U) |
                                     DataBuff_pu8[5]);
            VS_Controller[ctrl_index].battery_voltage_dV =
                (voltage_raw <= 6553U) ?
                (uint16_t)(voltage_raw * 10U) : 65535U;
            current_raw = (uint16_t)(((uint16_t)DataBuff_pu8[6] << 8U) |
                                     DataBuff_pu8[7]);
            VS_Controller[ctrl_index].phase_current_RMS_A = current_raw;
            VS_Controller[ctrl_index].motor_current_A = (int16_t)current_raw;
            s_msg2_age_ms[ctrl_index] = 0U;
            s_msg2_received[ctrl_index] = 1U;
            s_msg2_update_sequence[ctrl_index]++;
            s_msg2_guard[ctrl_index]++;
        }
        else /* 0x233: signed Iq [0..1] and Id [2..3], big-endian, A. */
        {
            s_msg3_guard[ctrl_index]++;
            VS_Controller[ctrl_index].iq_current_A =
                Decode_S16_BE(&DataBuff_pu8[0]);
            VS_Controller[ctrl_index].id_current_A =
                Decode_S16_BE(&DataBuff_pu8[2]);
            s_msg3_age_ms[ctrl_index] = 0U;
            s_msg3_received[ctrl_index] = 1U;
            s_msg3_update_sequence[ctrl_index]++;
            s_msg3_guard[ctrl_index]++;
        }
        return;
    }

    msg_base = (CanId_u32 & 0xFFFFFF00UL);

    if ((msg_base != CTRL_MSG1_BASE_ID) &&
        (msg_base != CTRL_MSG2_BASE_ID))
    {
        return;
    }

    ctrl_index = Get_ControllerIndex(CanId_u32);

    if (ctrl_index >= (uint8_t)CTRL_COUNT)
    {
        return;
    }

    /* =====================================================
     * MESSAGE 1: 0x0CF11E00 + DeviceID
     * ===================================================== */
    if (msg_base == CTRL_MSG1_BASE_ID)
    {
        speed_rpm =
            (uint16_t)(((uint16_t)DataBuff_pu8[1] << 8U) |
                       (uint16_t)DataBuff_pu8[0]);

        current_raw =
            (uint16_t)(((uint16_t)DataBuff_pu8[3] << 8U) |
                       (uint16_t)DataBuff_pu8[2]);

        voltage_raw =
            (uint16_t)(((uint16_t)DataBuff_pu8[5] << 8U) |
                       (uint16_t)DataBuff_pu8[4]);

        error_code =
            (uint16_t)(((uint16_t)DataBuff_pu8[7] << 8U) |
                       (uint16_t)DataBuff_pu8[6]);

        /* Begin sequence-locked update. */
        s_msg1_guard[ctrl_index]++;

        VS_Controller[ctrl_index].speed_rpm = speed_rpm;
        VS_Controller[ctrl_index].motor_current_A =
            (int16_t)(current_raw / 10U);
        VS_Controller[ctrl_index].battery_voltage_dV = voltage_raw;
        VS_Controller[ctrl_index].error_code = error_code;

        s_msg1_age_ms[ctrl_index] = 0U;
        s_msg1_received[ctrl_index] = 1U;
        s_msg1_update_sequence[ctrl_index]++;

        s_msg1_guard[ctrl_index]++;

        Update_ControllerAlive(ctrl_index);
    }
    /* =====================================================
     * MESSAGE 2: 0x0CF11F00 + DeviceID
     * ===================================================== */
    else
    {
        uint8_t controller_temp;
        uint8_t motor_temp;

        /* Clamp below-offset raw values instead of allowing uint8 underflow. */
        controller_temp =
            (DataBuff_pu8[2] >= 40U) ?
            (uint8_t)(DataBuff_pu8[2] - 40U) : 0U;

        motor_temp =
            (DataBuff_pu8[3] >= 30U) ?
            (uint8_t)(DataBuff_pu8[3] - 30U) : 0U;

        /* Begin sequence-locked update. */
        s_msg2_guard[ctrl_index]++;

        VS_Controller[ctrl_index].controller_temp_C =
            controller_temp;
        VS_Controller[ctrl_index].motor_temp_C =
            motor_temp;

        VS_Controller[ctrl_index].feedback_dir =
            (uint8_t)(DataBuff_pu8[4] & 0x03U);

        VS_Controller[ctrl_index].command_dir =
            (uint8_t)((DataBuff_pu8[4] >> 2U) & 0x03U);

        VS_Controller[ctrl_index].switch_status = DataBuff_pu8[5];

        s_msg2_age_ms[ctrl_index] = 0U;
        s_msg2_received[ctrl_index] = 1U;
        s_msg2_update_sequence[ctrl_index]++;

        s_msg2_guard[ctrl_index]++;
    }
}

/* ========================================================= */
void Controller_CAN_TimeoutTask(uint16_t task_period_ms)
{
    uint8_t index;

    for (index = 0U; index < (uint8_t)CTRL_COUNT; index++)
    {
        /*
         * Age counters intentionally remain outside the data sequence lock.
         * The CAN ISR is the only writer of guarded feedback payloads. A race
         * on age can change freshness by at most one task period, whereas
         * allowing two writers on the sequence guard could corrupt the lock.
         */
        s_msg1_age_ms[index] =
            Saturating_Add_ms(s_msg1_age_ms[index], task_period_ms);

        s_msg2_age_ms[index] =
            Saturating_Add_ms(s_msg2_age_ms[index], task_period_ms);

        s_msg3_age_ms[index] =
            Saturating_Add_ms(s_msg3_age_ms[index], task_period_ms);

        Update_ControllerAlive(index);
    }
}

/* ========================================================= */
uint8_t Controller_CAN_GetSnapshot(ControllerIndex_t index,
                                   ControllerFeedbackSnapshot_t *snapshot)
{
    uint8_t msg1_guard_before;
    uint8_t msg1_guard_after;
    uint8_t msg2_guard_before;
    uint8_t msg2_guard_after;
    uint8_t msg3_guard_before;
    uint8_t msg3_guard_after;
    uint8_t attempt;

    if ((snapshot == 0) || ((uint8_t)index >= (uint8_t)CTRL_COUNT))
    {
        return 0U;
    }

    for (attempt = 0U; attempt < CTRL_RX_SNAPSHOT_RETRIES; attempt++)
    {
        msg1_guard_before = s_msg1_guard[index];

        if ((msg1_guard_before & 0x01U) != 0U)
        {
            continue;
        }

        snapshot->speed_rpm = VS_Controller[index].speed_rpm;
        if (index != CTRL_ROTOR)
        {
            snapshot->motor_current_A = VS_Controller[index].motor_current_A;
            snapshot->battery_voltage_dV =
                VS_Controller[index].battery_voltage_dV;
        }
        else
        {
            snapshot->feedback_dir = VS_Controller[index].feedback_dir;
            snapshot->switch_status = VS_Controller[index].switch_status;
        }
        snapshot->error_code = VS_Controller[index].error_code;
        snapshot->msg1_received = s_msg1_received[index];
        snapshot->msg1_sequence = s_msg1_update_sequence[index];
        snapshot->msg1_age_ms = s_msg1_age_ms[index];

        msg1_guard_after = s_msg1_guard[index];

        if ((msg1_guard_before != msg1_guard_after) ||
            ((msg1_guard_after & 0x01U) != 0U))
        {
            continue;
        }

        msg2_guard_before = s_msg2_guard[index];

        if ((msg2_guard_before & 0x01U) != 0U)
        {
            continue;
        }

        snapshot->controller_temp_C =
            VS_Controller[index].controller_temp_C;
        snapshot->motor_temp_C = VS_Controller[index].motor_temp_C;
        snapshot->bus_current_dA = VS_Controller[index].bus_current_dA;
        snapshot->phase_current_RMS_A =
            VS_Controller[index].phase_current_RMS_A;
        if (index == CTRL_ROTOR)
        {
            /* The rotor receives current and voltage in 0x412, while
             * speed/error/feedback direction arrive in 0x411. Read each
             * under the matching sequence guard. */
            snapshot->motor_current_A = VS_Controller[index].motor_current_A;
            snapshot->battery_voltage_dV =
                VS_Controller[index].battery_voltage_dV;
        }
        else
        {
            snapshot->feedback_dir = VS_Controller[index].feedback_dir;
            snapshot->switch_status = VS_Controller[index].switch_status;
        }
        /* Rotor command_dir is task-owned and is one-byte atomic. */
        snapshot->command_dir = VS_Controller[index].command_dir;
        snapshot->msg2_received = s_msg2_received[index];
        snapshot->msg2_sequence = s_msg2_update_sequence[index];
        snapshot->msg2_age_ms = s_msg2_age_ms[index];

        msg2_guard_after = s_msg2_guard[index];

        if ((msg2_guard_before != msg2_guard_after) ||
            ((msg2_guard_after & 0x01U) != 0U))
        {
            continue;
        }

        msg3_guard_before = s_msg3_guard[index];

        if ((msg3_guard_before & 0x01U) != 0U)
        {
            continue;
        }

        snapshot->iq_current_A = VS_Controller[index].iq_current_A;
        snapshot->id_current_A = VS_Controller[index].id_current_A;
        snapshot->msg3_received = s_msg3_received[index];
        snapshot->msg3_sequence = s_msg3_update_sequence[index];
        snapshot->msg3_age_ms = s_msg3_age_ms[index];

        msg3_guard_after = s_msg3_guard[index];

        if ((msg3_guard_before == msg3_guard_after) &&
            ((msg3_guard_after & 0x01U) == 0U))
        {
            return 1U;
        }
    }

    return 0U;
}
