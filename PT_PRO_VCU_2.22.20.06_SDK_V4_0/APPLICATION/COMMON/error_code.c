#include "error_code.h"
#include "Can_Broadcast.h"
#include <stddef.h>
#include "vehicle_state.h"
#include "vehicle_config.h"
#include "machine_drive_logic.h"

#define EC_MAX_FRAMES_PER_CYCLE   3U   // limit per 100ms cycle (tune this)

/* ================= Globals ================= */
EV_State_t g_ev_state = { {0U} };

/* previous state + pending CAN event */
static uint8_t s_ev_prev_active[EV_MAX];
static uint8_t s_ev_pending_tx[EV_MAX];
static uint8_t s_event_seq = 0U;

/* timers */
static uint16_t s_wheel_no_response_ms     = 0U;

/* ================= Local macros ================= */
#define EV_SET(code)  do { if (((code) > EV_NONE) && ((code) < EV_MAX)) { g_ev_state.active[(code)] = 1U; } } while (0)

static uint16_t ec_abs_u16_diff(uint16_t a, uint16_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

static uint16_t ec_battery_voltage_dv(const BatteryState_t *bat)
{
    if (bat->Voltage_dV != 0U)
    {
        return bat->Voltage_dV;
    }

    return (uint16_t)(bat->BatteryVoltage_raw / 10U);
}

static uint8_t ec_is_valid_rotor_dir(uint8_t dir)
{
    return (uint8_t)((dir == DIR_STOP) || (dir == DIR_FORWARD) || (dir == DIR_REVERSE));
}

/* ================= Internal ================= */
static void ev_clear_active_only(void)
{
    uint16_t i;
    for (i = 0U; i < (uint16_t)EV_MAX; i++)
    {
        g_ev_state.active[i] = 0U;
    }
}

static void ev_capture_new_events(void)
{
    EV_Code_t code;

    for (code = (EV_Code_t)1; code < EV_MAX; code++)
    {
        if ((g_ev_state.active[code] != 0U) && (s_ev_prev_active[code] == 0U))
        {
            s_ev_pending_tx[code] = 1U;
        }

        s_ev_prev_active[code] = g_ev_state.active[code];
    }
}


/* ================= API ================= */
void EC_Init(void)
{
    uint16_t i;
    ev_clear_active_only();

    for (i = 0U; i < (uint16_t)EV_MAX; i++)
    {
        s_ev_prev_active[i] = 0U;
        s_ev_pending_tx[i] = 0U;
    }

    s_event_seq = 0U;
    s_wheel_no_response_ms = 0U;

}

void EC_ClearAll(void)
{
    uint16_t i;
    ev_clear_active_only(); 

    for (i = 0U; i < (uint16_t)EV_MAX; i++)
    {
        s_ev_prev_active[i] = 0U;
        s_ev_pending_tx[i] = 0U;
    }

    s_wheel_no_response_ms = 0U;
}

void EC_Update(uint16_t period_ms)
{
    static uint16_t startup_grace_ms = 0U;
    uint8_t wheel_cmd_present;
    uint8_t left_alive;
    uint8_t right_alive;
    uint8_t rotor_alive;
    uint8_t bms1_alive;
    uint8_t bms2_alive;
    uint8_t startup_done;
    uint8_t any_ctrl_ot;
    uint8_t any_motor_ot;
    uint8_t dir_mismatch;
    uint16_t fb_left_rpm;
    uint16_t fb_right_rpm;
    uint16_t bms1_voltage_dv;
    //uint16_t bms2_voltage_dv;

    if (period_ms == 0U)
    {
        period_ms = EC_TASK_PERIOD_DEFAULT_MS;
    }

    if (startup_grace_ms < 3000U)
    {
        startup_grace_ms = (uint16_t)(startup_grace_ms + period_ms);
    }

    startup_done = (uint8_t)(startup_grace_ms >= 3000U);

    ev_clear_active_only();

    left_alive  = VS_Controller[CTRL_WHEEL_LEFT].alive;
    right_alive = VS_Controller[CTRL_WHEEL_RIGHT].alive;
    rotor_alive = VS_Controller[CTRL_ROTOR].alive;
    bms1_alive = VS_Battery1.Alive;
    bms2_alive = VS_Battery2.Alive;
    bms1_voltage_dv = ec_battery_voltage_dv(&VS_Battery1);
    //bms2_voltage_dv = ec_battery_voltage_dv(&VS_Battery2);
    fb_left_rpm  = VS_Controller[CTRL_WHEEL_LEFT].speed_rpm;
    fb_right_rpm = VS_Controller[CTRL_WHEEL_RIGHT].speed_rpm;

    wheel_cmd_present = (uint8_t)(
        (VS_RPM_Left > EC_WHEEL_ACTUAL_MIN_RPM) ||
        (VS_RPM_Right > EC_WHEEL_ACTUAL_MIN_RPM) 
    );

    /*
    rotor_cmd_present = (uint8_t)(
        (VS_RPM_Rotor > EC_ROTOR_ACTUAL_MIN_RPM) ||
        (VS_RotorEnable != 0U) ||
        (VS_Rotormodestate > 0U)
    );

     EV001 
    if (VS_KillSwitch == 0U)
    {
        EV_SET(EV001_KILL_SWITCH_OFF);
    }

     EV002 
    if (VS_KillRecoveryActive != 0U)
    {
        EV_SET(EV002_KILL_RECOVERY_ACTIVE);
    }
*/
    /* EV003 
    if ((startup_done != 0U) &&
        (wheel_cmd_present != 0U) &&
        //(VS_MotorEnable == 0U) &&
        (VS_KillSwitch != 0U))
    {
        if (s_motor_enable_mismatch_ms < 60000U)
        {
            s_motor_enable_mismatch_ms = (uint16_t)(s_motor_enable_mismatch_ms + period_ms);
        }
    }
    else
    {
        s_motor_enable_mismatch_ms = 0U;
    }

    if (s_motor_enable_mismatch_ms >= 500U)
    {
        EV_SET(EV003_MOTOR_ENABLE_MISMATCH);
    }
        */

    /* EV005 */
    if ((VS_TurnLeft != 0U) && (VS_TurnRight != 0U))
    {
        EV_SET(EV003_INVALID_DUAL_TURN_CMD);
    }

    /* EV006 
    if ((ec_is_valid_wheel_dir(VS_Direction) == 0U) ||
        (ec_is_valid_wheel_dir(VS_CurrentDirection) == 0U) ||
        (ec_is_valid_wheel_dir(VS_RequestedDirection) == 0U) ||
        (ec_is_valid_dir_state((uint8_t)VS_DirState) == 0U))
    {
        EC_SET(EV006_INVALID_INTERNAL_DIRECTION_STATE);
    } */

    /* EV007 / EV036 */
    if (ec_is_valid_rotor_dir(VS_RotorDirection) == 0U)
    {
        EV_SET(EV004_INVALID_ROTOR_DIRECTION_STATE);
    }

    if (VS_Rotormodestate > 3U)
    {
        EV_SET(EV005_INVALID_ROTOR_MODE_STATE);
    }

    /* EV009 / EV010 / EV011 after startup grace */
    if (startup_done != 0U)
    {
        if (left_alive == 0U)
        {
            EV_SET(EV006_LEFT_CTRL_CAN_LOST);
        }

        if (right_alive == 0U)
        {
            EV_SET(EV007_RIGHT_CTRL_CAN_LOST);
        }

        if (rotor_alive == 0U)
        {
            EV_SET(EV008_ROTOR_CTRL_CAN_LOST);
        }
    }

    /* EV012 / EV013 only after timeout is meaningful */
    if (startup_done != 0U)
    {
        if ((bms1_alive == 0U) && (VS_Battery1.RxTimeoutMs >= 3000U))
        {
            EV_SET(EV009_BATTERY1_CAN_LOST);
        }

        if ((bms2_alive == 0U) && (VS_Battery2.RxTimeoutMs >= 3000U))
        {
            EV_SET(EV010_BATTERY2_CAN_LOST);
        }
    }

    /* EV014 
    any_node_offline = 0U;

    if ((g_ec_state.active[EV009_LEFT_CTRL_CAN_LOST] != 0U) ||
        (g_ec_state.active[EV010_RIGHT_CTRL_CAN_LOST] != 0U) ||
        (g_ec_state.active[EV011_ROTOR_CTRL_CAN_LOST] != 0U) ||
        (g_ec_state.active[EV012_BATTERY1_CAN_LOST] != 0U) ||
        (g_ec_state.active[EV013_BATTERY2_CAN_LOST] != 0U))
    {
        any_node_offline = 1U;
    }

    if (any_node_offline != 0U)
    {
        EV_SET(EV014_REQUIRED_NODE_OFFLINE);
    }
        */

    /* EV015 / EV016 */
    if (VS_PsmRotorLock != 0U)
    {
        EV_SET(EV011_LOW_SOC_ROTOR_LOCK_ACTIVE);
    }

    if (VS_PsmSocConfirmTimer_ms > 0U)
    {
        EV_SET(EV012_LOW_SOC_CONFIRMED);
    }

    /* EV017 / EV018 only when BMS alive and voltage valid */
    if ((bms1_alive != 0U) &&
        (bms1_voltage_dv > 0U) &&
        (bms1_voltage_dv < EC_BATT_UV_THRESHOLD_DV))
    {
        EV_SET(EV013_BATTERY1_UNDERVOLTAGE);
    }

    /*if ((bms2_alive != 0U) &&
        (bms2_voltage_dv > 0U) &&
        (bms2_voltage_dv < EC_BATT_UV_THRESHOLD_DV))
    {
        EV_SET(EV014_BATTERY2_UNDERVOLTAGE);
    }

     EV019 / EV020 only when BMS alive 
    if ((bms1_alive != 0U) &&
        (
           // (VS_Battery1.BatteryTemp_C >= EC_BATT_OT_THRESHOLD_C) ||
         (VS_Battery1.Ambient_Temp_C >= EC_BATT_OT_THRESHOLD_C) ||
         (VS_Battery1.MOSFET_Temp_C >= EC_BATT_OT_THRESHOLD_C)))
    {
        EV_SET(EV015_BATTERY1_OVERTEMP);
    }

    if ((bms2_alive != 0U) &&
        (
            //(VS_Battery2.BatteryTemp_C >= EC_BATT_OT_THRESHOLD_C) ||
         (VS_Battery2.Ambient_Temp_C >= EC_BATT_OT_THRESHOLD_C) ||
         (VS_Battery2.MOSFET_Temp_C >= EC_BATT_OT_THRESHOLD_C)))
    {
        EV_SET(EV016_BATTERY2_OVERTEMP);
    }
    
    
    if ((bms1_alive != 0U) || (bms2_alive != 0U))
    {
        ec_check_battery_cell_temps();
    }*/

    /* EV021 / EV022 only when BMS alive */
    if ((bms1_alive != 0U) &&
        ((VS_Battery1.AlarmBits_1 != 0U) ||
         (VS_Battery1.AlarmBits_2 != 0U) ||
         (VS_Battery1.ProtBits_1  != 0U) ||
         (VS_Battery1.ProtBits_2  != 0U) ||
         (VS_Battery1.FaultBits_1 != 0U)))
    {
        EV_SET(EV017_BATTERY1_BMS_ALARM_ACTIVE);
    }

    if ((bms2_alive != 0U) &&
        ((VS_Battery2.AlarmBits_1 != 0U) ||
         (VS_Battery2.AlarmBits_2 != 0U) ||
         (VS_Battery2.ProtBits_1  != 0U) ||
         (VS_Battery2.ProtBits_2  != 0U) ||
         (VS_Battery2.FaultBits_1 != 0U)))
    {
        EV_SET(EV018_BATTERY2_BMS_ALARM_ACTIVE);
    }

    /* EV023 only when both batteries are alive and valid 
    if ((bms1_alive != 0U) &&
        (bms2_alive != 0U) &&
        (VS_Battery1.SOC > 0U) &&
        (VS_Battery2.SOC > 0U))
    {
        if ((VS_Battery1.SOC, VS_Battery2.SOC) >= EC_BATT_MISMATCH_SOC_DIFF_PERCENT))
        {
            EV_SET(EV019_BATTERY_PACK_MISMATCH);
        }
    }
        */
    
    /* EV024 / EV025 / EV026 only when controller alive */
    if ((left_alive != 0U) && (VS_Controller[CTRL_WHEEL_LEFT].error_code != 0U))
    {
        EV_SET(EV020_LEFT_CTRL_FAULT_ACTIVE);
    }

    if ((right_alive != 0U) && (VS_Controller[CTRL_WHEEL_RIGHT].error_code != 0U))
    {
        EV_SET(EV021_RIGHT_CTRL_FAULT_ACTIVE);
    }

    if ((rotor_alive != 0U) && (VS_Controller[CTRL_ROTOR].error_code != 0U))
    {
        EV_SET(EV022_ROTOR_CTRL_FAULT_ACTIVE);
    }

    /* EV027 */
    any_ctrl_ot = 0U;

    if (((left_alive != 0U)  && (VS_Controller[CTRL_WHEEL_LEFT].controller_temp_C >= EC_CTRL_OT_THRESHOLD_C)) ||
        ((right_alive != 0U) && (VS_Controller[CTRL_WHEEL_RIGHT].controller_temp_C >= EC_CTRL_OT_THRESHOLD_C)) ||
        ((rotor_alive != 0U) && (VS_Controller[CTRL_ROTOR].controller_temp_C >= EC_CTRL_OT_THRESHOLD_C)))
    {
        any_ctrl_ot = 1U;
    }

    if (any_ctrl_ot != 0U)
    {
        EV_SET(EV023_CONTROLLER_OVERTEMP);
    }

    /* EV024 */
    any_motor_ot = 0U;

    if (((left_alive != 0U)  && (VS_Controller[CTRL_WHEEL_LEFT].motor_temp_C >= EC_MOTOR_OT_THRESHOLD_C)) ||
        ((right_alive != 0U) && (VS_Controller[CTRL_WHEEL_RIGHT].motor_temp_C >= EC_MOTOR_OT_THRESHOLD_C)) ||
        ((rotor_alive != 0U) && (VS_Controller[CTRL_ROTOR].motor_temp_C >= EC_MOTOR_OT_THRESHOLD_C)))
    {
        any_motor_ot = 1U;
    }

    if (any_motor_ot != 0U)
    {
        EV_SET(EV024_MOTOR_OVERTEMP);
    }

    /* EV025 only while command exists */
    dir_mismatch = 0U;

    if ((left_alive != 0U) &&
        (VS_Controller[CTRL_WHEEL_LEFT].command_dir != DIR_STOP) &&
        (VS_Controller[CTRL_WHEEL_LEFT].feedback_dir != DIR_STOP) &&
        (VS_Controller[CTRL_WHEEL_LEFT].feedback_dir != VS_Controller[CTRL_WHEEL_LEFT].command_dir))
    {
        dir_mismatch = 1U;
    }

    if ((right_alive != 0U) &&
        (VS_Controller[CTRL_WHEEL_RIGHT].command_dir != DIR_STOP) &&
        (VS_Controller[CTRL_WHEEL_RIGHT].feedback_dir != DIR_STOP) &&
        (VS_Controller[CTRL_WHEEL_RIGHT].feedback_dir != VS_Controller[CTRL_WHEEL_RIGHT].command_dir))
    {
        dir_mismatch = 1U;
    }

    if ((rotor_alive != 0U) &&
        (VS_Controller[CTRL_ROTOR].command_dir != DIR_STOP) &&
        (VS_Controller[CTRL_ROTOR].feedback_dir != DIR_STOP) &&
        (VS_Controller[CTRL_ROTOR].feedback_dir != VS_Controller[CTRL_ROTOR].command_dir))
    {
        dir_mismatch = 1U;
    }

    if (dir_mismatch != 0U)
    {
        EV_SET(EV025_CTRL_FEEDBACK_DIR_MISMATCH);
    }

    /* EV026 */
    if ((startup_done != 0U) &&
        (wheel_cmd_present != 0U) &&
        (VS_MotorEnable != 0U) &&
        (VS_KillSwitch != 0U) &&
        (left_alive != 0U) &&
        (right_alive != 0U) &&
        (fb_left_rpm < EC_WHEEL_ACTUAL_MIN_RPM) &&
        (fb_right_rpm < EC_WHEEL_ACTUAL_MIN_RPM))
    {
        if (s_wheel_no_response_ms < 60000U)
        {
            s_wheel_no_response_ms = (uint16_t)(s_wheel_no_response_ms + period_ms);
        }
    }
    else
    {
        s_wheel_no_response_ms = 0U;
    }

    if (s_wheel_no_response_ms >= EC_WHEEL_NO_RESPONSE_MS)
    {
        EV_SET(EV026_WHEEL_RPM_NOT_RESPONDING);
    }

    /* EV027 */
    if ((startup_done != 0U) &&
        (wheel_cmd_present != 0U) &&
        (VS_TurnLeft == 0U) &&
        (VS_TurnRight == 0U) &&
        (left_alive != 0U) &&
        (right_alive != 0U) &&
        (fb_left_rpm > EC_WHEEL_ACTUAL_MIN_RPM) &&
        (fb_right_rpm > EC_WHEEL_ACTUAL_MIN_RPM) &&
        (ec_abs_u16_diff(fb_left_rpm, fb_right_rpm) >= EC_WHEEL_IMBALANCE_RPM))
    {
        EV_SET(EV027_LEFT_RIGHT_WHEEL_RPM_IMBALANCE);
    }

    /* EV032 
    if ((startup_done != 0U) &&
        (rotor_cmd_present != 0U) &&
        (VS_KillSwitch != 0U) &&
        (rotor_alive != 0U) &&
        (fb_rotor_rpm < EC_ROTOR_ACTUAL_MIN_RPM))
    {
        if (s_rotor_no_response_ms < 60000U)
        {
            s_rotor_no_response_ms = (uint16_t)(s_rotor_no_response_ms + period_ms);
        }
    }
    else
    {
        s_rotor_no_response_ms = 0U;
    }

    if (s_rotor_no_response_ms >= EC_ROTOR_NO_RESPONSE_MS)
    {
        EC_SET(EV032_ROTOR_RPM_NOT_RESPONDING);
    } */

    /* EV034 
    if ((VS_Throttle_Wheel > EC_WHEEL_THROTTLE_ACTIVE_THRESHOLD) &&
        (VS_DirState != DIR_STATE_IDLE))
    {
        EC_SET(EV034_THROTTLE_IGNORED_DURING_DIR_TRANSITION);
    }*/

    /* EV028 */
    if ((startup_done != 0U) &&
        (((bms1_alive != 0U) && (bms2_alive == 0U)) ||
         ((bms1_alive == 0U) && (bms2_alive != 0U))))
    {
        EV_SET(EV028_ONLY_ONE_BATTERY_ALIVE);
    }

    /* EV029 */
    if ((VS_PsmRotorLock != 0U) &&
        (VS_PsmRotorLockDueToSocDiff != 0U))
    {
        EV_SET(EV029_SOC_DIFF_PSM_ROTORLOCK);
    }

    /* EV038 
    any_overcurrent = 0U;

    if ((VS_Battery1.BatteryCurrent_raw >= EC_OVERCURRENT_THRESHOLD_A) ||
        (VS_Battery2.BatteryCurrent_raw >= EC_OVERCURRENT_THRESHOLD_A)  )
    {
        any_overcurrent = 1U;
    }

    if (any_overcurrent != 0U)
    {
        if (s_overcurrent_ms < 60000U)
        {
            s_overcurrent_ms = (uint16_t)(s_overcurrent_ms + period_ms);
        }
    }
    else
    {
        s_overcurrent_ms = 0U;
    }

    if (s_overcurrent_ms >= EC_OVERCURRENT_DEBOUNCE_MS)
    {
        EC_SET(EV038_OVERCURRENT_EVENT);
    } 
    */
    /* EV039 
    if (MachineDrive_IsBatteryFuseLatched() != 0U)
    {
        EV_SET(EV030_BATTERY_FUSE_LATCHED);
    }*/

    /* EV040 / EV041 summary only 
    if (ec_drive_inhibited_now() != 0U)
    {
        EC_SET(EV040_DRIVE_INHIBITED);
    }

    if (ec_rotor_inhibited_now() != 0U)
    {
        EC_SET(EV041_ROTOR_INHIBITED);
    } */
    
    
    ev_capture_new_events();
}

uint8_t EC_IsActive(EV_Code_t code)
{
    if ((code <= EV_NONE) || (code >= EV_MAX))
    {
        return 0U;
    }

    return g_ev_state.active[code];
}

uint32_t EV_GetCanIdForError(EV_Code_t code)
{
    if ((code <= EV_NONE) || (code >= EV_MAX))
    {
        return 0U;
    }

    if (EC_CAN_USE_EXTENDED_ID != 0U)
    {
        return (EC_CAN_EXT_ID_BASE + (uint32_t)code);
    }

    return (uint32_t)(EV_CAN_STD_ID_BASE + (uint32_t)code);
}

uint8_t EV_HasPendingCanEvent(void)
{
    EV_Code_t code;

    for (code = (EV_Code_t)1; code < EV_MAX; code++)
    {
        if (s_ev_pending_tx[code] != 0U)
        {
            return 1U;
        }
    }

    return 0U;
}

uint8_t EV_GetNextPendingCanFrame(EV_CanFrame_t *frame)
{
    EV_Code_t code;

    if (frame == NULL)
    {
        return 0U;
    }

    for (code = (EV_Code_t)1; code < EV_MAX; code++)
    {
        if (s_ev_pending_tx[code] != 0U)
        {
            s_ev_pending_tx[code] = 0U;

            frame->id = EV_GetCanIdForError(code);
            frame->dlc = 8U;
            frame->is_extended = (uint8_t)EC_CAN_USE_EXTENDED_ID;

            frame->data[0] = (uint8_t)code;
            frame->data[1] = 0U;
            frame->data[2] = 1U;
            frame->data[3] = s_event_seq++;
            frame->data[4] = VS_FaultFlag;
            frame->data[5] = (uint8_t)VS_DriveMode;
            frame->data[6] = VS_CurrentDirection;
            frame->data[7] = 0U;

            return 1U;
        }
    }

    return 0U;
}

uint8_t EC_GetNextPendingCanFrame(EV_CanFrame_t *frame)
{
    static uint8_t ctrl_index = 0U;

    static const uint32_t s_ec_can_ids[3] =
    {
        EC_LEFT_CAN_ID,
        EC_RIGHT_CAN_ID,
        EC_ROTOR_CAN_ID
    };

    uint8_t i;
    uint16_t error_word;

    if(frame == NULL)
    {
        return 0U;
    }

    frame->dlc = 8U;
    frame->is_extended = 0U;
    for(i = 0U; i < 8U; i++)
    {
        frame->data[i] = 0U;
    }

    for(i = 0U; i < 3U; i++)
    {
        error_word = VS_Controller[ctrl_index].error_code;

        if(error_word != 0U)
        {
            frame->id = s_ec_can_ids[ctrl_index];

            frame->data[0] =
                (uint8_t)(error_word & 0xFFU);

            frame->data[1] =
                (uint8_t)((error_word >> 8U) & 0xFFU);

            frame->data[2] = 0U;

            ctrl_index++;

            if(ctrl_index >= 3U)
            {
                ctrl_index = 0U;
            }

            return 1U;
        }

        ctrl_index++;

        if(ctrl_index >= 3U)
        {
            ctrl_index = 0U;
        }
    }

    return 0U;
}

uint8_t EB_GetNextPendingCanFrame(EV_CanFrame_t *frame)
{
    static uint8_t batt_index = 0U;

    uint8_t i;

    if(frame == NULL)
    {
        return 0U;
    }

    frame->dlc = 8U;

    frame->is_extended = 0U;
    for(i = 0U; i < 8U; i++)
    {
        frame->data[i] = 0U;
    }

    for(i = 0U; i < 10U; i++)
    {   
        frame->data[0] = 0U;
        switch(batt_index)
        {
            /* ================= BATTERY 1 ================= */

            case 0U:

                if(VS_Battery1.AlarmBits_1 != 0U)
                {
                    frame->id = EB1_ALARM1_CAN_ID;

                    frame->data[0] =
                        VS_Battery1.AlarmBits_1;
                }
                else
                {
                    batt_index++;
                    break;
                }

                break;

            case 1U:

                if(VS_Battery1.AlarmBits_2 != 0U)
                {
                    frame->id = EB1_ALARM2_CAN_ID;

                    frame->data[0] =
                        VS_Battery1.AlarmBits_2;
                }
                else
                {
                    batt_index++;
                    break;
                }

                break;

            case 2U:

                if(VS_Battery1.ProtBits_1 != 0U)
                {
                    frame->id = EB1_PROT1_CAN_ID;

                    frame->data[0] =
                        VS_Battery1.ProtBits_1;
                }
                else
                {
                    batt_index++;
                    break;
                }

                break;

            case 3U:

                if(VS_Battery1.ProtBits_2 != 0U)
                {
                    frame->id = EB1_PROT2_CAN_ID;

                    frame->data[0] =
                        VS_Battery1.ProtBits_2;
                }
                else
                {
                    batt_index++;
                    break;
                }

                break;

            case 4U:

                if(VS_Battery1.FaultBits_1 != 0U)
                {
                    frame->id = EB1_FAULT1_CAN_ID;

                    frame->data[0] =
                        VS_Battery1.FaultBits_1;
                }
                else
                {
                    batt_index++;
                    break;
                }

                break;

            /* ================= BATTERY 2 ================= */

            case 5U:

                if(VS_Battery2.AlarmBits_1 != 0U)
                {
                    frame->id = EB2_ALARM1_CAN_ID;

                    frame->data[0] =
                        VS_Battery2.AlarmBits_1;
                }
                else
                {
                    batt_index++;
                    break;
                }

                break;

            case 6U:

                if(VS_Battery2.AlarmBits_2 != 0U)
                {
                    frame->id = EB2_ALARM2_CAN_ID;

                    frame->data[0] =
                        VS_Battery2.AlarmBits_2;
                }
                else
                {
                    batt_index++;
                    break;
                }

                break;

            case 7U:

                if(VS_Battery2.ProtBits_1 != 0U)
                {
                    frame->id = EB2_PROT1_CAN_ID;

                    frame->data[0] =
                        VS_Battery2.ProtBits_1;
                }
                else
                {
                    batt_index++;
                    break;
                }

                break;

            case 8U:

                if(VS_Battery2.ProtBits_2 != 0U)
                {
                    frame->id = EB2_PROT2_CAN_ID;

                    frame->data[0] =
                        VS_Battery2.ProtBits_2;
                }
                else
                {
                    batt_index++;
                    break;
                }

                break;

            case 9U:

                if(VS_Battery2.FaultBits_1 != 0U)
                {
                    frame->id = EB2_FAULT1_CAN_ID;

                    frame->data[0] =
                        VS_Battery2.FaultBits_1;
                }
                else
                {
                    batt_index++;
                    break;
                }

                break;

            default:

                batt_index = 0U;

                break;
        }

        if(frame->data[0] != 0U)
        {
            frame->data[1] = 0U;

            batt_index++;

            if(batt_index >= 10U)
            {
                batt_index = 0U;
            }

            return 1U;
        }

        batt_index++;

        if(batt_index >= 10U)
        {
            batt_index = 0U;
        }
    }

    return 0U;
}

void EC_QueueAllActiveErrors(void)
{
   /* EC_Code_t code;

    for (code = (EC_Code_t)1; code < EV_MAX; code++)
    {
        if (g_ec_state.active[code] != 0U)
        {
            EC_QueueCanEvent(code);
        }
    }*/
}

void Send_CanFrame(const EV_CanFrame_t *frame)
{
    uint8_t tx_data[8];
    uint8_t i;

    if (frame == NULL)
    {
        return;
    }

    for (i = 0U; i < frame->dlc; i++)
    {
        tx_data[i] = frame->data[i];
    }

    CanMsgTransmit(
        CAN_0,
        tx_data,
        frame->dlc,
        0U,
        frame->is_extended,
        frame->id
    );
}
