#include "error_code.h"
#include "Can_Broadcast.h"
#include <stddef.h>
#include "vehicle_state.h"
#include "vehicle_config.h"
#include "controller_can_rx.h"

/* Diagnostic qualification only; does not delay machine protection. */
#define EV_CONFIRM_MS             (10000U)
#define EV_STARTUP_GRACE_MS       (3000U)
#define EV_SOC_LOW_MASK           (0x03U)
#define EV_SOC_DIFF_MASK          (0x04U)
#define EV_PENDING_ACTIVE         (0x01U)
#define EV_PENDING_REPEAT         (0x04U)

EV_State_t g_ev_state = { {0U} };
static uint8_t s_ev_pending_tx[EV_MAX];
static uint8_t s_ev_detected[EV_MAX];
static uint16_t s_ev_confirm_ms[EV_MAX];
static uint16_t s_startup_grace_ms;
static uint8_t s_event_seq;
static uint8_t s_ev_next;
/* Round-robin cursors enumerate individual fault bits, not just nodes. */
static uint8_t s_ctrl_next;
static uint8_t s_batt_next;
#define EC_CTRL_FAULT_BITS        (16U)
#define EC_BATT_FAULT_FIELDS      (10U)
#define EC_BATT_FAULT_BITS        (8U)

#define EV_SET(code) do { s_ev_detected[(code)] = 1U; } while (0)

/* Explicit TX allow-list: keep the originally commented EVs silent.
 * When enabling a new diagnostic, update this list as well. */
static uint8_t ec_event_enabled(EV_Code_t code)
{
    switch (code)
    {
        case EV003_INVALID_DUAL_TURN_CMD:
        case EV004_INVALID_ROTOR_DIRECTION_STATE:
        case EV005_INVALID_ROTOR_MODE_STATE:
        case EV006_LEFT_CTRL_CAN_LOST:
        case EV007_RIGHT_CTRL_CAN_LOST:
        case EV008_ROTOR_CTRL_CAN_LOST:
        case EV009_BATTERY1_CAN_LOST:
        case EV010_BATTERY2_CAN_LOST:
        case EV011_LOW_SOC_ROTOR_LOCK_ACTIVE:
        case EV012_LOW_SOC_CONFIRMED:
        case EV013_BATTERY1_UNDERVOLTAGE:
        case EV017_BATTERY1_BMS_ALARM_ACTIVE:
        case EV018_BATTERY2_BMS_ALARM_ACTIVE:
        case EV020_LEFT_CTRL_FAULT_ACTIVE:
        case EV021_RIGHT_CTRL_FAULT_ACTIVE:
        case EV022_ROTOR_CTRL_FAULT_ACTIVE:
        case EV023_CONTROLLER_OVERTEMP:
        case EV024_MOTOR_OVERTEMP:
        case EV025_CTRL_FEEDBACK_DIR_MISMATCH:
        case EV026_WHEEL_RPM_NOT_RESPONDING:
        case EV027_LEFT_RIGHT_WHEEL_RPM_IMBALANCE:
        case EV028_ONLY_ONE_BATTERY_ALIVE:
        case EV029_SOC_DIFF_PSM_ROTORLOCK:
            return 1U;
        default:
            return 0U;
    }
}

static uint16_t ec_add_ms(uint16_t value, uint16_t increment, uint16_t limit)
{
    uint32_t next = (uint32_t)value + (uint32_t)increment;
    return (next >= (uint32_t)limit) ? limit : (uint16_t)next;
}

static uint16_t ec_abs_u16_diff(uint16_t a, uint16_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

static uint8_t ec_battery_alive(const BatteryState_t *bat)
{
    return (uint8_t)((bat->Alive != 0U) && (bat->RxTimeoutMs != 0U));
}

static uint8_t ec_battery_alarm(const BatteryState_t *bat)
{
    return (uint8_t)((bat->AlarmBits_1 != 0U) || (bat->AlarmBits_2 != 0U) ||
                     (bat->ProtBits_1 != 0U) || (bat->ProtBits_2 != 0U) ||
                     (bat->FaultBits_1 != 0U));
}

static uint8_t ec_is_drive_dir(uint8_t dir)
{
    return (uint8_t)((dir == DIR_FORWARD) || (dir == DIR_REVERSE));
}

static uint8_t ec_msg1_fresh(const ControllerFeedbackSnapshot_t *snap)
{
    return (uint8_t)((snap->msg1_received != 0U) &&
                     (snap->msg1_age_ms <= VC_CONTROLLER_MSG1_TIMEOUT_MS));
}

static uint8_t ec_msg2_fresh(const ControllerFeedbackSnapshot_t *snap)
{
    return (uint8_t)((snap->msg2_received != 0U) &&
                     (snap->msg2_age_ms <= VC_CONTROLLER_MSG2_TIMEOUT_MS));
}

static void ec_reset_events(void)
{
    uint8_t i;
    for (i = 0U; i < (uint8_t)EV_MAX; i++)
    {
        g_ev_state.active[i] = 0U;
        s_ev_pending_tx[i] = 0U;
        s_ev_detected[i] = 0U;
        s_ev_confirm_ms[i] = 0U;
    }
    s_ev_next = 1U;
}

void EC_Init(void)
{
    ec_reset_events();
    s_startup_grace_ms = 0U;
    s_event_seq = 0U;
    s_ctrl_next = 0U;
    s_batt_next = 0U;
}

void EC_ClearAll(void)
{
    uint8_t i;
    /* IoT treats receipt of an EV CAN ID as an error, regardless of data.
     * Clearing a condition must cancel TX, not emit a clear frame. */
    for (i = 1U; i < (uint8_t)EV_MAX; i++)
    {
        s_ev_pending_tx[i] = 0U;
        g_ev_state.active[i] = 0U;
        s_ev_detected[i] = 0U;
        s_ev_confirm_ms[i] = 0U;
    }
}

static void ec_qualify_events(uint16_t period_ms)
{
    uint8_t i;
    uint8_t active;
    for (i = 1U; i < (uint8_t)EV_MAX; i++)
    {
        if ((ec_event_enabled((EV_Code_t)i) != 0U) &&
            (s_ev_detected[i] != 0U))
        {
            s_ev_confirm_ms[i] = ec_add_ms(s_ev_confirm_ms[i], period_ms,
                                          EV_CONFIRM_MS);
        }
        else
        {
            s_ev_confirm_ms[i] = 0U;
        }
        active = (uint8_t)(s_ev_confirm_ms[i] >= EV_CONFIRM_MS);
        if (active != g_ev_state.active[i])
        {
            g_ev_state.active[i] = active;
            if (active != 0U)
            {
                s_ev_pending_tx[i] |= EV_PENDING_ACTIVE;
            }
            else
            {
                s_ev_pending_tx[i] = 0U;
            }
        }
    }
}

void EC_Update(uint16_t period_ms)
{
    ControllerFeedbackSnapshot_t snap[CTRL_COUNT];
    uint8_t msg1_ok[CTRL_COUNT] = {0U};
    uint8_t msg2_ok[CTRL_COUNT] = {0U};
    uint8_t i;
    uint8_t bms1_alive;
    uint8_t bms2_alive;
    //uint16_t bms2_voltage_dv;
    uint8_t startup_done;
    uint8_t wheel_cmd_present;
    uint8_t wheel_running_cmd;
    uint8_t lock_cause;
    uint8_t expected_dir;
    uint8_t direction_fresh;

    if (period_ms == 0U) { period_ms = EC_TASK_PERIOD_DEFAULT_MS; }
    s_startup_grace_ms = ec_add_ms(s_startup_grace_ms, period_ms,
                                   EV_STARTUP_GRACE_MS);
    startup_done = (uint8_t)(s_startup_grace_ms >= EV_STARTUP_GRACE_MS);
    for (i = 0U; i < (uint8_t)EV_MAX; i++) { s_ev_detected[i] = 0U; }
    for (i = 0U; i < (uint8_t)CTRL_COUNT; i++)
    {
        if (Controller_CAN_GetSnapshot((ControllerIndex_t)i, &snap[i]) != 0U)
        {
            msg1_ok[i] = ec_msg1_fresh(&snap[i]);
            msg2_ok[i] = ec_msg2_fresh(&snap[i]);
        }
    }
    bms1_alive = ec_battery_alive(&VS_Battery1);
    bms2_alive = ec_battery_alive(&VS_Battery2);
    //bms2_voltage_dv = ec_battery_voltage_dv(&VS_Battery2);
    lock_cause = s_psm_soc_lock_cause;
    wheel_cmd_present = (uint8_t)((VS_RPM_Left > EC_WHEEL_ACTUAL_MIN_RPM) ||
                                  (VS_RPM_Right > EC_WHEEL_ACTUAL_MIN_RPM));
    /* AppTest sends enable=1 during normal driving; VS_MotorEnable is unused. */
    wheel_running_cmd = (uint8_t)((wheel_cmd_present != 0U) &&
                                  (VS_KillSwitch != 0U) &&
                                  (VS_DirState == DIR_STATE_IDLE) &&
                                  (ec_is_drive_dir(VS_WheelCommandDirection) != 0U));

    /* Original disabled error logic retained below as comments. */
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

    if ((VS_TurnLeft != 0U) && (VS_TurnRight != 0U))
    { EV_SET(EV003_INVALID_DUAL_TURN_CMD); }
    /* EV006 
    if ((ec_is_valid_wheel_dir(VS_Direction) == 0U) ||
        (ec_is_valid_wheel_dir(VS_CurrentDirection) == 0U) ||
        (ec_is_valid_wheel_dir(VS_RequestedDirection) == 0U) ||
        (ec_is_valid_dir_state((uint8_t)VS_DirState) == 0U))
    {
        EC_SET(EV006_INVALID_INTERNAL_DIRECTION_STATE);
    } */

    if ((VS_RotorDirection != DIR_STOP) &&
        (ec_is_drive_dir(VS_RotorDirection) == 0U))
    { EV_SET(EV004_INVALID_ROTOR_DIRECTION_STATE); }
    if (VS_Rotormodestate > 3U) { EV_SET(EV005_INVALID_ROTOR_MODE_STATE); }

    if (startup_done != 0U)
    {
        if (VS_Controller[CTRL_WHEEL_LEFT].alive == 0U)
        { EV_SET(EV006_LEFT_CTRL_CAN_LOST); }
        if (VS_Controller[CTRL_WHEEL_RIGHT].alive == 0U)
        { EV_SET(EV007_RIGHT_CTRL_CAN_LOST); }
        if (VS_Controller[CTRL_ROTOR].alive == 0U)
        { EV_SET(EV008_ROTOR_CTRL_CAN_LOST); }
        /* RxTimeoutMs counts DOWN to zero. Both packs expected in this build. */
        if ((bms1_alive == 0U) && (VS_Battery1.RxTimeoutMs == 0U))
        { EV_SET(EV009_BATTERY1_CAN_LOST); }
        if ((bms2_alive == 0U) && (VS_Battery2.RxTimeoutMs == 0U))
        { EV_SET(EV010_BATTERY2_CAN_LOST); }
        if (bms1_alive != bms2_alive) { EV_SET(EV028_ONLY_ONE_BATTERY_ALIVE); }
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

    if (VS_PsmRotorLock != 0U)
    {
        EV_SET(EV011_LOW_SOC_ROTOR_LOCK_ACTIVE);
        //if ((lock_cause & EV_SOC_LOW_MASK) != 0U)
        //{ EV_SET(EV012_LOW_SOC_CONFIRMED); }
        if ((lock_cause & EV_SOC_DIFF_MASK) != 0U)
        { EV_SET(EV029_SOC_DIFF_PSM_ROTORLOCK); }
    }
    /* Voltage_dV is cleared by the existing battery timeout task. */
    if ((bms1_alive != 0U) && (VS_Battery1.Voltage_dV > 0U) &&
        (VS_Battery1.Voltage_dV < EC_BATT_UV_THRESHOLD_DV))
    { EV_SET(EV013_BATTERY1_UNDERVOLTAGE); }
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

    if ((bms1_alive != 0U) && (ec_battery_alarm(&VS_Battery1) != 0U))
    { EV_SET(EV017_BATTERY1_BMS_ALARM_ACTIVE); }
    if ((bms2_alive != 0U) && (ec_battery_alarm(&VS_Battery2) != 0U))
    { EV_SET(EV018_BATTERY2_BMS_ALARM_ACTIVE); }

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

    for (i = 0U; i < (uint8_t)CTRL_COUNT; i++)
    {
        if ((msg1_ok[i] != 0U) && (snap[i].error_code != 0U))
        {
            if (i == CTRL_WHEEL_LEFT) { EV_SET(EV020_LEFT_CTRL_FAULT_ACTIVE); }
            else if (i == CTRL_WHEEL_RIGHT) { EV_SET(EV021_RIGHT_CTRL_FAULT_ACTIVE); }
            else { EV_SET(EV022_ROTOR_CTRL_FAULT_ACTIVE); }
        }
        if (msg2_ok[i] != 0U)
        {
            if (snap[i].controller_temp_C >= (int16_t)EC_CTRL_OT_THRESHOLD_C)
            { EV_SET(EV023_CONTROLLER_OVERTEMP); }
            if (snap[i].motor_temp_C >= (int16_t)EC_MOTOR_OT_THRESHOLD_C)
            { EV_SET(EV024_MOTOR_OVERTEMP); }
        }
        /* Naxatra direction is decoded in MSG1, wheel direction in MSG2. */
        if (i == CTRL_ROTOR)
        {
            expected_dir = VS_RotorDirection;
            direction_fresh = msg1_ok[i];
        }
        else
        {
            expected_dir = VS_WheelCommandDirection;
            direction_fresh = msg2_ok[i];
            if (VS_DirState != DIR_STATE_IDLE) { direction_fresh = 0U; }
        }
        if ((VS_KillSwitch != 0U) && (msg1_ok[i] != 0U) &&
            (direction_fresh != 0U) && (ec_is_drive_dir(expected_dir) != 0U) &&
            (ec_is_drive_dir(snap[i].feedback_dir) != 0U) &&
            (snap[i].feedback_dir != expected_dir))
        { EV_SET(EV025_CTRL_FEEDBACK_DIR_MISMATCH); }
    }

    /* Existing EV026 semantics: BOTH wheels fail to move with a drive command.
     * The common 10s qualifier replaces its shorter 1.5s diagnostic timer. */
    if ((startup_done != 0U) && (wheel_running_cmd != 0U) &&
        (VS_RPM_Left > EC_WHEEL_ACTUAL_MIN_RPM) &&
        (VS_RPM_Right > EC_WHEEL_ACTUAL_MIN_RPM) &&
        (msg1_ok[CTRL_WHEEL_LEFT] != 0U) && (msg1_ok[CTRL_WHEEL_RIGHT] != 0U) &&
        (snap[CTRL_WHEEL_LEFT].speed_rpm < EC_WHEEL_ACTUAL_MIN_RPM) &&
        (snap[CTRL_WHEEL_RIGHT].speed_rpm < EC_WHEEL_ACTUAL_MIN_RPM))
    { EV_SET(EV026_WHEEL_RPM_NOT_RESPONDING); }

    if ((startup_done != 0U) && (wheel_running_cmd != 0U) &&
        (VS_TurnLeft == 0U) && (VS_TurnRight == 0U) &&
        (msg1_ok[CTRL_WHEEL_LEFT] != 0U) && (msg1_ok[CTRL_WHEEL_RIGHT] != 0U) &&
        (snap[CTRL_WHEEL_LEFT].speed_rpm > EC_WHEEL_ACTUAL_MIN_RPM) &&
        (snap[CTRL_WHEEL_RIGHT].speed_rpm > EC_WHEEL_ACTUAL_MIN_RPM) &&
        (ec_abs_u16_diff(snap[CTRL_WHEEL_LEFT].speed_rpm,
                         snap[CTRL_WHEEL_RIGHT].speed_rpm) >= EC_WHEEL_IMBALANCE_RPM))
    { EV_SET(EV027_LEFT_RIGHT_WHEEL_RPM_IMBALANCE); }

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

    ec_qualify_events(period_ms);
}

uint8_t EC_IsActive(EV_Code_t code)
{
    if (ec_event_enabled(code) == 0U) { return 0U; }
    return g_ev_state.active[code];
}

uint32_t EV_GetCanIdForError(EV_Code_t code)
{
    if ((code <= EV_NONE) || (code >= EV_MAX)) { return 0U; }
    /* IoT EV_CODE_MAP uses decimal tag digits in the hexadecimal ID:
     * EV009=0x769, EV010=0x770, EV020=0x780, EV030=0x790.
     * Do not add the binary enum directly to 0x760. */
    if (EC_CAN_USE_EXTENDED_ID != 0U)
    { return EC_CAN_EXT_ID_BASE + (uint32_t)code; }
    return (uint32_t)EV_CAN_STD_ID_BASE +
           (((uint32_t)code / 10UL) * 0x10UL) +
           ((uint32_t)code % 10UL);
}

uint8_t EV_HasPendingCanEvent(void)
{
    uint8_t i;
    for (i = 1U; i < (uint8_t)EV_MAX; i++)
    { if ((EC_IsActive((EV_Code_t)i) != 0U) && (s_ev_pending_tx[i] != 0U)) { return 1U; } }
    return 0U;
}

static void ec_init_frame(EV_CanFrame_t *frame, uint32_t id)
{
    uint8_t i;
    frame->id = id;
    frame->dlc = 8U;
    frame->is_extended = 0U;
    for (i = 0U; i < 8U; i++) { frame->data[i] = 0U; }
}

uint8_t EV_GetNextPendingCanFrame(EV_CanFrame_t *frame)
{
    uint8_t i;
    uint8_t code;
    if (frame == NULL) { return 0U; }
    for (i = 1U; i < (uint8_t)EV_MAX; i++)
    {
        code = s_ev_next++;
        if (s_ev_next >= (uint8_t)EV_MAX) { s_ev_next = 1U; }
        if (EC_IsActive((EV_Code_t)code) == 0U)
        {
            s_ev_pending_tx[code] = 0U;
            continue;
        }
        if (s_ev_pending_tx[code] != 0U)
        {
            ec_init_frame(frame, EV_GetCanIdForError((EV_Code_t)code));
            frame->is_extended = (uint8_t)EC_CAN_USE_EXTENDED_ID;
            frame->data[0] = code;
            frame->data[2] = 1U;
            s_ev_pending_tx[code] = 0U;
            frame->data[3] = s_event_seq++;
            frame->data[4] = VS_FaultFlag;
            frame->data[5] = (uint8_t)VS_DriveMode;
            frame->data[6] = VS_CurrentDirection;
            return 1U;
        }
    }
    return 0U;
}

uint8_t EC_GetNextPendingCanFrame(EV_CanFrame_t *frame)
{
    static const uint32_t ids[CTRL_COUNT] = {EC_LEFT_CAN_ID, EC_RIGHT_CAN_ID, EC_ROTOR_CAN_ID};
    ControllerFeedbackSnapshot_t snap;
    uint16_t errors[CTRL_COUNT] = {0U};
    uint16_t mask;
    uint8_t i;
    uint8_t slot;
    uint8_t index;
    uint8_t bit;
    if (frame == NULL) { return 0U; }

    /* Read each fresh controller once per call. The server uses exact
     * one-hot payload keys, so transmit only one currently active bit. */
    for (i = 0U; i < (uint8_t)CTRL_COUNT; i++)
    {
        if ((Controller_CAN_GetSnapshot((ControllerIndex_t)i, &snap) != 0U) &&
            (ec_msg1_fresh(&snap) != 0U))
        {
            errors[i] = snap.error_code;
        }
    }
    for (i = 0U; i < (uint8_t)(CTRL_COUNT * EC_CTRL_FAULT_BITS); i++)
    {
        slot = s_ctrl_next++;
        if (s_ctrl_next >= (uint8_t)(CTRL_COUNT * EC_CTRL_FAULT_BITS))
        { s_ctrl_next = 0U; }
        /* Interleave controllers to avoid delaying another node behind
         * multiple active bits on one controller. */
        index = (uint8_t)(slot % (uint8_t)CTRL_COUNT);
        bit = (uint8_t)(slot / (uint8_t)CTRL_COUNT);
        mask = (uint16_t)(1UL << bit);
        if ((errors[index] & mask) != 0U)
        {
            ec_init_frame(frame, ids[index]);
            frame->data[0] = (uint8_t)(mask & 0xFFU);
            frame->data[1] = (uint8_t)(mask >> 8U);
            return 1U;
        }
    }
    return 0U;
}

static uint8_t ec_battery_error_byte(const BatteryState_t *bat, uint8_t index)
{
    switch (index)
    {
        case 0U: return bat->AlarmBits_1;
        case 1U: return bat->AlarmBits_2;
        case 2U: return bat->ProtBits_1;
        case 3U: return bat->ProtBits_2;
        case 4U: return bat->FaultBits_1;
        default: return 0U;
    }
}

uint8_t EB_GetNextPendingCanFrame(EV_CanFrame_t *frame)
{
    static const uint32_t ids[EC_BATT_FAULT_FIELDS] = {
        EB1_ALARM1_CAN_ID, EB1_ALARM2_CAN_ID, EB1_PROT1_CAN_ID,
        EB1_PROT2_CAN_ID, EB1_FAULT1_CAN_ID, EB2_ALARM1_CAN_ID,
        EB2_ALARM2_CAN_ID, EB2_PROT1_CAN_ID, EB2_PROT2_CAN_ID, EB2_FAULT1_CAN_ID
    };
    /* Exactly the bits defined by the server's CLASSIFIED_ERROR_MAP.
     * Reserved/unmapped bits must not produce UNKNOWN_NULL events. */
    static const uint8_t valid_masks[5U] = {0xFFU, 0x1FU, 0x7FU, 0xFFU, 0x0FU};
    const BatteryState_t *bat;
    uint8_t errors[EC_BATT_FAULT_FIELDS] = {0U};
    uint8_t i;
    uint8_t index;
    uint8_t field;
    uint8_t slot;
    uint8_t bit;
    uint8_t mask;
    if (frame == NULL) { return 0U; }

    for (i = 0U; i < EC_BATT_FAULT_FIELDS; i++)
    {
        bat = (i < 5U) ? &VS_Battery1 : &VS_Battery2;
        if (ec_battery_alive(bat) != 0U)
        {
            field = (uint8_t)(i % 5U);
            errors[i] = (uint8_t)(ec_battery_error_byte(bat, field) & valid_masks[field]);
        }
    }
    for (i = 0U; i < (EC_BATT_FAULT_FIELDS * EC_BATT_FAULT_BITS); i++)
    {
        slot = s_batt_next++;
        if (s_batt_next >= (EC_BATT_FAULT_FIELDS * EC_BATT_FAULT_BITS))
        { s_batt_next = 0U; }
        index = (uint8_t)(slot % EC_BATT_FAULT_FIELDS);
        bit = (uint8_t)(slot / EC_BATT_FAULT_FIELDS);
        mask = (uint8_t)(1UL << bit);
        if ((errors[index] & mask) != 0U)
        {
            ec_init_frame(frame, ids[index]);
            frame->data[0] = mask;
            return 1U;
        }
    }
    return 0U;
}

void EC_QueueAllActiveErrors(void)
{
    uint8_t i;
    for (i = 1U; i < (uint8_t)EV_MAX; i++)
    { if (EC_IsActive((EV_Code_t)i) != 0U) { s_ev_pending_tx[i] |= EV_PENDING_REPEAT; } }
}

void Send_CanFrame(const EV_CanFrame_t *frame)
{
    uint8_t tx_data[8];
    uint8_t i;
    if ((frame == NULL) || (frame->dlc > 8U)) { return; }
    for (i = 0U; i < frame->dlc; i++) { tx_data[i] = frame->data[i]; }
    CanMsgTransmit(CAN_0, tx_data, frame->dlc, 0U, frame->is_extended, frame->id);
}
