#include "wheel_direction_sync.h"
#include "controller_can_rx.h"
#include "vehicle_state.h"
#include "vehicle_config.h"

#ifndef MACHINE_DRIVE_TASK_PERIOD_MS
#define MACHINE_DRIVE_TASK_PERIOD_MS                  (20U)
#endif

/* ================= Internal runtime ================= */
static uint8_t  s_applied_direction = DIR_STOP;
static uint16_t s_state_timer_ms = 0U;
static uint16_t s_pickup_common_rpm = 0U;
static uint16_t s_filtered_left_rpm = 0U;
static uint16_t s_filtered_right_rpm = 0U;

static uint8_t  s_zero_confirm_pairs = 0U;
static uint8_t  s_ack_confirm_pairs = 0U;
static uint8_t  s_pickup_stable_pairs = 0U;
static uint8_t  s_fault_clear_pairs = 0U;
static uint8_t  s_rampup_hard_diff_pairs = 0U;
static uint8_t  s_direction_mismatch_pairs = 0U;

static uint8_t  s_pickup_freeze = 0U;
static uint8_t  s_single_wheel_condition = 0U;
static uint8_t  s_pickup_retry_count = 0U;
static uint8_t  s_pickup_retry_hold_active = 0U;
static uint8_t  s_pickup_seen_fresh_pair = 0U;

static uint8_t  s_left_direction_ok = 1U;
static uint8_t  s_right_direction_ok = 1U;

static uint8_t  s_auto_resync_count = 0U;
static uint8_t  s_active_brake_handover = 0U;
static uint8_t  s_slope_rollback_confirm_pairs = 0U;
static uint8_t  s_slope_creep_confirm_pairs = 0U;
static uint8_t  s_slope_trend_valid = 0U;
static uint16_t s_slope_window_start_rpm = 0U;

static uint16_t s_pickup_single_wheel_timer_ms = 0U;
static uint16_t s_pickup_retry_hold_timer_ms = 0U;
static uint16_t s_pickup_feedback_wait_ms = 0U;
static uint16_t s_fault_clear_timer_ms = 0U;

static uint8_t s_last_left_msg1_sequence = 0U;
static uint8_t s_last_right_msg1_sequence = 0U;
static uint8_t s_last_left_msg2_sequence = 0U;
static uint8_t s_last_right_msg2_sequence = 0U;

/* Forward declarations used by slope-handover helpers. */
static uint8_t Msg1_IsFresh(const ControllerFeedbackSnapshot_t *snapshot);
static uint8_t Msg2_IsFresh(const ControllerFeedbackSnapshot_t *snapshot);

/* ================= Helpers ================= */
static uint8_t Direction_IsValid(uint8_t direction)
{
    return (uint8_t)(((direction == DIR_FORWARD) ||
                      (direction == DIR_REVERSE)) ? 1U : 0U);
}

static uint16_t Min_U16(uint16_t a, uint16_t b)
{
    return (a < b) ? a : b;
}

static uint16_t Max_U16(uint16_t a, uint16_t b)
{
    return (a > b) ? a : b;
}

static uint16_t AbsDiff_U16(uint16_t a, uint16_t b)
{
    return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static void Set_Active_Brake_Handover(uint8_t enable)
{
#if VC_DIR_ACTIVE_BRAKE_HANDOVER_ENABLE
    s_active_brake_handover = (enable != 0U) ? 1U : 0U;
#else
    (void)enable;
    s_active_brake_handover = 0U;
#endif

    VS_WheelActiveBrakeHandover = s_active_brake_handover;
}

static void Set_Slope_Assist_Active(uint8_t enable)
{
#if VC_DIR_SLOPE_HANDOVER_ENABLE
    VS_WheelSlopeAssistActive = (enable != 0U) ? 1U : 0U;
#else
    (void)enable;
    VS_WheelSlopeAssistActive = 0U;
#endif
}

static uint16_t Get_Transition_Hold_RPM(void)
{
#if VC_DIR_ACTIVE_BRAKE_HANDOVER_ENABLE
    if (s_active_brake_handover != 0U)
    {
        return VC_DIR_ACTIVE_BRAKE_HOLD_RPM;
    }
#endif

    return 0U;
}

static uint8_t Active_Brake_Handover_Required(
    uint16_t base_rpm,
    const ControllerFeedbackSnapshot_t *left,
    const ControllerFeedbackSnapshot_t *right,
    uint8_t snapshots_available)
{
#if VC_DIR_ACTIVE_BRAKE_HANDOVER_ENABLE
    if (Direction_IsValid(VS_CurrentDirection) == 0U)
    {
        return 0U;
    }

    if ((VS_RPM_Wheel_Current > VC_DIR_ACTIVE_BRAKE_HOLD_RPM) ||
        (base_rpm > 0U))
    {
        return 1U;
    }

    if ((snapshots_available != 0U) &&
        ((left->speed_rpm > VC_DIR_ACTIVE_BRAKE_DETECT_RPM) ||
         (right->speed_rpm > VC_DIR_ACTIVE_BRAKE_DETECT_RPM)))
    {
        return 1U;
    }
#else
    (void)base_rpm;
    (void)left;
    (void)right;
    (void)snapshots_available;
#endif

    return 0U;
}

/*
 * Returns 1 when the wheel feedback indicates that gravity/load has already
 * rolled the machine in the newly requested direction while the VCU is still
 * holding the old direction at the active-brake floor. In this condition,
 * waiting for a strict zero-speed sample can remove the operator's ability to
 * complete the direction change on a slope.
 */
static uint8_t Slope_Rollback_Direction_Detected(
    const ControllerFeedbackSnapshot_t *left,
    const ControllerFeedbackSnapshot_t *right,
    uint8_t requested_direction)
{
#if VC_DIR_SLOPE_HANDOVER_ENABLE
    uint8_t left_dir_requested;
    uint8_t right_dir_requested;
    uint16_t max_rpm;
    uint16_t diff_rpm;

    if ((Msg1_IsFresh(left) == 0U) ||
        (Msg1_IsFresh(right) == 0U) ||
        (Msg2_IsFresh(left) == 0U) ||
        (Msg2_IsFresh(right) == 0U))
    {
        return 0U;
    }

    max_rpm = Max_U16(left->speed_rpm, right->speed_rpm);
    diff_rpm = AbsDiff_U16(left->speed_rpm, right->speed_rpm);

    if ((max_rpm < VC_DIR_SLOPE_ROLLBACK_MIN_RPM) ||
        (max_rpm > VC_DIR_SLOPE_ROLLBACK_MAX_RPM) ||
        (diff_rpm > VC_DIR_SLOPE_ROLLBACK_MAX_DIFF_RPM))
    {
        return 0U;
    }

    left_dir_requested =
        (uint8_t)((left->feedback_dir == requested_direction) ? 1U : 0U);
    right_dir_requested =
        (uint8_t)((right->feedback_dir == requested_direction) ? 1U : 0U);

    /*
     * Require both wheel feedback directions to agree. This avoids switching
     * direction from a single noisy/stale direction bit while the other wheel
     * is still rotating in the old direction.
     */
    return (uint8_t)(((left_dir_requested != 0U) &&
                      (right_dir_requested != 0U)) ? 1U : 0U);
#else
    (void)left;
    (void)right;
    (void)requested_direction;
    return 0U;
#endif
}

static uint8_t Slope_Persistent_Creep_Detected(
    const ControllerFeedbackSnapshot_t *left,
    const ControllerFeedbackSnapshot_t *right,
    uint16_t state_timer_ms,
    uint8_t current_direction,
    uint8_t requested_direction)
{
#if VC_DIR_SLOPE_HANDOVER_ENABLE
    uint16_t max_rpm;
    uint16_t diff_rpm;
    uint16_t total_decel_rpm;
    uint8_t left_dir_valid;
    uint8_t right_dir_valid;

    if ((Msg1_IsFresh(left) == 0U) || (Msg1_IsFresh(right) == 0U))
    {
        return 0U;
    }

    max_rpm = Max_U16(left->speed_rpm, right->speed_rpm);
    diff_rpm = AbsDiff_U16(left->speed_rpm, right->speed_rpm);

    /*
     * Candidate window for external-load/slope creep. A normal flat-ground
     * deceleration is expected to keep losing meaningful RPM over the window;
     * a slope/load tends to plateau or re-accelerate against the 2-RPM brake
     * floor. We deliberately make this classifier conservative so flat-ground
     * direction changes prefer the strict near-zero path.
     */
    if ((state_timer_ms < VC_DIR_SLOPE_CREEP_DETECT_DELAY_MS) ||
        (max_rpm < VC_DIR_SLOPE_CREEP_MIN_RPM) ||
        (max_rpm > VC_DIR_SLOPE_CREEP_MAX_RPM) ||
        (diff_rpm > VC_DIR_SLOPE_CREEP_MAX_DIFF_RPM))
    {
        s_slope_trend_valid = 0U;
        s_slope_creep_confirm_pairs = 0U;
        return 0U;
    }

    /*
     * If both fresh direction values are valid, they must agree. This blocks a
     * handover while the left/right wheels are mechanically rotating in
     * different directions. STOP/unknown remains acceptable near zero.
     */
    if ((Msg2_IsFresh(left) != 0U) && (Msg2_IsFresh(right) != 0U))
    {
        left_dir_valid = Direction_IsValid(left->feedback_dir);
        right_dir_valid = Direction_IsValid(right->feedback_dir);

        if ((left_dir_valid != 0U) && (right_dir_valid != 0U))
        {
            if (left->feedback_dir != right->feedback_dir)
            {
                s_slope_trend_valid = 0U;
                s_slope_creep_confirm_pairs = 0U;
                return 0U;
            }

            if ((left->feedback_dir != current_direction) &&
                (left->feedback_dir != requested_direction))
            {
                s_slope_trend_valid = 0U;
                s_slope_creep_confirm_pairs = 0U;
                return 0U;
            }
        }
    }

    if (s_slope_trend_valid == 0U)
    {
        s_slope_window_start_rpm = max_rpm;
        s_slope_trend_valid = 1U;
        s_slope_creep_confirm_pairs = 0U;
        return 0U;
    }

    if (s_slope_creep_confirm_pairs <
        VC_DIR_SLOPE_CREEP_WINDOW_FRESH_PAIRS)
    {
        s_slope_creep_confirm_pairs++;
    }

    if (s_slope_creep_confirm_pairs <
        VC_DIR_SLOPE_CREEP_WINDOW_FRESH_PAIRS)
    {
        return 0U;
    }

    if (s_slope_window_start_rpm > max_rpm)
    {
        total_decel_rpm =
            (uint16_t)(s_slope_window_start_rpm - max_rpm);
    }
    else
    {
        /* Same or higher RPM = no decay, therefore strong load evidence. */
        total_decel_rpm = 0U;
    }

    if (total_decel_rpm <= VC_DIR_SLOPE_CREEP_MAX_TOTAL_DECEL_RPM)
    {
        return 1U;
    }

    /*
     * The machine is still decelerating normally: classify this observation
     * window as flat/normal and start a fresh window from the current speed.
     */
    s_slope_window_start_rpm = max_rpm;
    s_slope_creep_confirm_pairs = 0U;
    return 0U;
#else
    (void)left;
    (void)right;
    (void)state_timer_ms;
    (void)current_direction;
    (void)requested_direction;
    return 0U;
#endif
}

static uint16_t Saturating_Add_ms(uint16_t value,
                                  uint16_t increment,
                                  uint16_t limit)
{
    uint32_t next_value;

    next_value = (uint32_t)value + (uint32_t)increment;

    if (next_value >= (uint32_t)limit)
    {
        return limit;
    }

    return (uint16_t)next_value;
}

static uint16_t Apply_RPM_Slew_Local(uint16_t target,
                                     uint16_t current,
                                     uint16_t step)
{
    if (target > current)
    {
        uint32_t next_value;

        next_value = (uint32_t)current + (uint32_t)step;
        return (next_value >= (uint32_t)target) ?
               target : (uint16_t)next_value;
    }

    if (current > target)
    {
        if ((uint16_t)(current - target) <= step)
        {
            return target;
        }

        return (uint16_t)(current - step);
    }

    return target;
}

static void Set_Equal_Wheel_Command(uint16_t rpm, uint8_t direction)
{
    VS_RPM_Wheel_Current = rpm;
    VS_RPM_Left = rpm;
    VS_RPM_Right = rpm;
    VS_WheelCommandDirection = direction;
    VS_Direction = direction;
    VS_WheelPickupCommonRPM = rpm;
}

static uint8_t Msg1_IsFresh(const ControllerFeedbackSnapshot_t *snapshot)
{
    return (uint8_t)(((snapshot->msg1_received != 0U) &&
                      (snapshot->msg1_age_ms <=
                       VC_CONTROLLER_MSG1_TIMEOUT_MS)) ? 1U : 0U);
}

static uint8_t Msg2_IsFresh(const ControllerFeedbackSnapshot_t *snapshot)
{
    return (uint8_t)(((snapshot->msg2_received != 0U) &&
                      (snapshot->msg2_age_ms <=
                       VC_CONTROLLER_MSG2_TIMEOUT_MS)) ? 1U : 0U);
}

static uint8_t New_Msg1_Pair(const ControllerFeedbackSnapshot_t *left,
                             const ControllerFeedbackSnapshot_t *right)
{
    if ((Msg1_IsFresh(left) == 0U) || (Msg1_IsFresh(right) == 0U))
    {
        return 0U;
    }

    if ((left->msg1_sequence != s_last_left_msg1_sequence) &&
        (right->msg1_sequence != s_last_right_msg1_sequence))
    {
        s_last_left_msg1_sequence = left->msg1_sequence;
        s_last_right_msg1_sequence = right->msg1_sequence;
        return 1U;
    }

    return 0U;
}

static uint8_t New_Msg2_Pair(const ControllerFeedbackSnapshot_t *left,
                             const ControllerFeedbackSnapshot_t *right)
{
    if ((Msg2_IsFresh(left) == 0U) || (Msg2_IsFresh(right) == 0U))
    {
        return 0U;
    }

    if ((left->msg2_sequence != s_last_left_msg2_sequence) &&
        (right->msg2_sequence != s_last_right_msg2_sequence))
    {
        s_last_left_msg2_sequence = left->msg2_sequence;
        s_last_right_msg2_sequence = right->msg2_sequence;
        return 1U;
    }

    return 0U;
}

static void Set_Msg1_Baseline(const ControllerFeedbackSnapshot_t *left,
                              const ControllerFeedbackSnapshot_t *right)
{
    s_last_left_msg1_sequence = left->msg1_sequence;
    s_last_right_msg1_sequence = right->msg1_sequence;
}

static void Set_Msg2_Baseline(const ControllerFeedbackSnapshot_t *left,
                              const ControllerFeedbackSnapshot_t *right)
{
    s_last_left_msg2_sequence = left->msg2_sequence;
    s_last_right_msg2_sequence = right->msg2_sequence;
}

static uint8_t Snapshot_Pair_Available(ControllerFeedbackSnapshot_t *left,
                                       ControllerFeedbackSnapshot_t *right)
{
    uint8_t left_ok;
    uint8_t right_ok;

    left_ok = Controller_CAN_GetSnapshot(CTRL_WHEEL_LEFT, left);
    right_ok = Controller_CAN_GetSnapshot(CTRL_WHEEL_RIGHT, right);

    return (uint8_t)(((left_ok != 0U) && (right_ok != 0U)) ? 1U : 0U);
}

static uint8_t Feedback_Direction_OK_Or_Unknown(uint8_t feedback_direction,
                                                uint8_t requested_direction)
{
    /*
     * Some controllers report STOP/unknown while stationary or during the first
     * few feedback cycles. A valid opposite direction is treated as a fault;
     * STOP/unknown is allowed temporarily and is rechecked after movement.
     */
    if (Direction_IsValid(feedback_direction) == 0U)
    {
        return 1U;
    }

    return (uint8_t)((feedback_direction == requested_direction) ? 1U : 0U);
}

static void Reset_State_Counters(void)
{
    s_state_timer_ms = 0U;
    s_zero_confirm_pairs = 0U;
    s_ack_confirm_pairs = 0U;
    s_pickup_stable_pairs = 0U;
    s_fault_clear_pairs = 0U;
    s_rampup_hard_diff_pairs = 0U;
    s_direction_mismatch_pairs = 0U;
    s_pickup_single_wheel_timer_ms = 0U;
    s_pickup_retry_hold_timer_ms = 0U;
    s_pickup_feedback_wait_ms = 0U;
    s_fault_clear_timer_ms = 0U;
    s_pickup_freeze = 0U;
    s_single_wheel_condition = 0U;
    s_pickup_retry_hold_active = 0U;
    s_pickup_seen_fresh_pair = 0U;
    s_left_direction_ok = 1U;
    s_right_direction_ok = 1U;
    s_slope_rollback_confirm_pairs = 0U;
    s_slope_creep_confirm_pairs = 0U;
    s_slope_trend_valid = 0U;
    s_slope_window_start_rpm = 0U;
    VS_DirectionStateTimer_ms = 0U;
}

static void Enter_State(DirectionState_En_t new_state)
{
    VS_DirState = new_state;
    Reset_State_Counters();
}

static void Enter_Fault(uint8_t fault_code)
{
    VS_WheelSyncFault = fault_code;
    s_applied_direction = DIR_STOP;
    s_pickup_common_rpm = 0U;
    Set_Equal_Wheel_Command(0U, DIR_STOP);
    Enter_State(DIR_STATE_FAULT);
}

static void Prepare_Pickup(uint16_t base_rpm,
                           uint8_t direction,
                           const ControllerFeedbackSnapshot_t *left,
                           const ControllerFeedbackSnapshot_t *right,
                           uint8_t snapshots_available)
{
    uint16_t start_rpm;

    s_pickup_retry_count = 0U;
    s_pickup_retry_hold_active = 0U;
    s_pickup_retry_hold_timer_ms = 0U;
    s_pickup_feedback_wait_ms = 0U;
    s_pickup_seen_fresh_pair = 0U;
    s_pickup_freeze = 0U;
    s_single_wheel_condition = 0U;
    s_pickup_single_wheel_timer_ms = 0U;
    s_pickup_stable_pairs = 0U;
    s_direction_mismatch_pairs = 0U;
    s_left_direction_ok = 1U;
    s_right_direction_ok = 1U;

    start_rpm = Min_U16(base_rpm, VC_DIR_PICKUP_START_RPM);

    if ((s_active_brake_handover != 0U) &&
        (base_rpm >= VC_DIR_ACTIVE_BRAKE_HOLD_RPM) &&
        (start_rpm < VC_DIR_ACTIVE_BRAKE_HOLD_RPM))
    {
        start_rpm = VC_DIR_ACTIVE_BRAKE_HOLD_RPM;
    }

    s_pickup_common_rpm = start_rpm;
    s_filtered_left_rpm = 0U;
    s_filtered_right_rpm = 0U;
    s_applied_direction = direction;

    if (snapshots_available != 0U)
    {
        Set_Msg1_Baseline(left, right);
        Set_Msg2_Baseline(left, right);
    }

    Set_Equal_Wheel_Command(s_pickup_common_rpm, direction);
    Enter_State(DIR_STATE_SYNC_PICKUP);
}

static void Start_Auto_Resync(uint8_t terminal_fault_code)
{
    /*
     * A transient left/right acceleration split is recoverable. Do not enter
     * the latched fault state on the first occurrence because DIR_STATE_FAULT
     * intentionally waits for zero throttle/safety-lever release.
     *
     * Instead, keep both commands equal, ramp both wheels back to zero in the
     * currently applied direction, and execute the synchronized pickup again.
     */
    if (s_auto_resync_count < VC_DIR_AUTO_RESYNC_MAX_ATTEMPTS)
    {
        s_auto_resync_count++;
        VS_WheelSyncRecoveryActive = 1U;
        VS_WheelSyncRecoveryCount = s_auto_resync_count;
        VS_WheelSyncFault = WHEEL_SYNC_FAULT_NONE;

        s_applied_direction = VS_CurrentDirection;
        Set_Active_Brake_Handover(1U);
        s_pickup_freeze = 0U;
        s_pickup_retry_hold_active = 0U;
        s_pickup_retry_count = 0U;

        Enter_State(DIR_STATE_RAMP_DOWN);
    }
    else
    {
        VS_WheelSyncRecoveryActive = 0U;
        Enter_Fault(terminal_fault_code);
    }
}

static void Start_Pickup_Retry(const ControllerFeedbackSnapshot_t *left,
                               const ControllerFeedbackSnapshot_t *right,
                               uint8_t snapshots_available)
{
    if (s_pickup_retry_count >= VC_DIR_PICKUP_MAX_RETRIES)
    {
        Start_Auto_Resync(WHEEL_SYNC_FAULT_SINGLE_WHEEL_PICKUP);
        return;
    }

    s_pickup_retry_count++;
    s_pickup_retry_hold_active = 1U;
    s_pickup_retry_hold_timer_ms = 0U;
    s_pickup_common_rpm = Get_Transition_Hold_RPM();
    s_pickup_freeze = 1U;
    s_single_wheel_condition = 0U;
    s_pickup_single_wheel_timer_ms = 0U;
    s_pickup_stable_pairs = 0U;
    s_direction_mismatch_pairs = 0U;
    s_left_direction_ok = 1U;
    s_right_direction_ok = 1U;
    s_filtered_left_rpm = 0U;
    s_filtered_right_rpm = 0U;

    if (snapshots_available != 0U)
    {
        Set_Msg1_Baseline(left, right);
        Set_Msg2_Baseline(left, right);
    }

    Set_Equal_Wheel_Command(Get_Transition_Hold_RPM(),
                            s_applied_direction);
}

static void Complete_Direction_Apply(uint16_t base_rpm,
                                     const ControllerFeedbackSnapshot_t *left,
                                     const ControllerFeedbackSnapshot_t *right,
                                     uint8_t snapshots_available)
{
    VS_CurrentDirection = s_applied_direction;
    VS_Direction = s_applied_direction;
    VS_WheelCommandDirection = s_applied_direction;

    if (base_rpm > 0U)
    {
        Prepare_Pickup(base_rpm,
                       s_applied_direction,
                       left,
                       right,
                       snapshots_available);
    }
    else
    {
        Set_Active_Brake_Handover(0U);
        Set_Slope_Assist_Active(0U);
        Set_Equal_Wheel_Command(0U, s_applied_direction);
        Enter_State(DIR_STATE_IDLE);
    }
}

/* ================= Public API ================= */
void WheelDirectionSync_Reset(void)
{
    s_applied_direction = DIR_STOP;
    s_pickup_common_rpm = 0U;
    s_filtered_left_rpm = 0U;
    s_filtered_right_rpm = 0U;
    s_pickup_retry_count = 0U;
    s_auto_resync_count = 0U;
    Set_Active_Brake_Handover(0U);
    Set_Slope_Assist_Active(0U);

    s_last_left_msg1_sequence = 0U;
    s_last_right_msg1_sequence = 0U;
    s_last_left_msg2_sequence = 0U;
    s_last_right_msg2_sequence = 0U;

    VS_CurrentDirection = DIR_STOP;
    VS_RequestedDirection = DIR_STOP;
    VS_Direction = DIR_STOP;
    VS_WheelCommandDirection = DIR_STOP;
    VS_WheelSyncFault = WHEEL_SYNC_FAULT_NONE;
    VS_WheelFeedbackDiff_RPM = 0U;
    VS_WheelPickupCommonRPM = 0U;
    VS_WheelSyncRecoveryActive = 0U;
    VS_WheelSyncRecoveryCount = 0U;

    Set_Equal_Wheel_Command(0U, DIR_STOP);
    Enter_State(DIR_STATE_IDLE);
}

uint8_t WheelDirectionSync_IsTransitionActive(void)
{
    return (uint8_t)((VS_DirState != DIR_STATE_IDLE) ? 1U : 0U);
}

void WheelDirectionSync_Step(uint16_t base_rpm,
                             uint8_t requested_direction)
{
    ControllerFeedbackSnapshot_t left;
    ControllerFeedbackSnapshot_t right;
    uint8_t snapshots_available;
    uint8_t msg1_pair_new;
    uint8_t msg2_pair_new;
    uint8_t left_moving;
    uint8_t right_moving;
    uint8_t left_direction_ok;
    uint8_t right_direction_ok;
    uint16_t feedback_diff;
    uint16_t pickup_target;
    uint16_t retry_start_rpm;
    uint16_t transition_hold_rpm;
    uint16_t direction_apply_hold_ms;

    VS_RequestedDirection = requested_direction;

    snapshots_available = Snapshot_Pair_Available(&left, &right);

    if (snapshots_available != 0U)
    {
        VS_WheelFeedbackDiff_RPM =
            AbsDiff_U16(left.speed_rpm, right.speed_rpm);
    }
    else
    {
        VS_WheelFeedbackDiff_RPM = 0U;
    }

    if ((VS_KillSwitch == 0U) ||
        (Direction_IsValid(requested_direction) == 0U))
    {
        WheelDirectionSync_Reset();
        return;
    }

    msg1_pair_new = 0U;
    msg2_pair_new = 0U;

    if (snapshots_available != 0U)
    {
        msg1_pair_new = New_Msg1_Pair(&left, &right);
        msg2_pair_new = New_Msg2_Pair(&left, &right);
    }

    switch (VS_DirState)
    {
        case DIR_STATE_IDLE:
        {
            VS_WheelSyncFault = WHEEL_SYNC_FAULT_NONE;
            VS_DirectionStateTimer_ms = 0U;

            if (Direction_IsValid(VS_CurrentDirection) == 0U)
            {
                /*
                 * Initial arming does not wait for a zero-RPM command-direction
                 * echo. Some controllers update that echo only after torque is
                 * requested. Apply the direction at zero, hold, then use the
                 * synchronized low-RPM pickup to validate real motion.
                 */
                Set_Active_Brake_Handover(0U);
                Set_Slope_Assist_Active(0U);
                s_applied_direction = requested_direction;
                Set_Equal_Wheel_Command(0U, s_applied_direction);

                if (snapshots_available != 0U)
                {
                    Set_Msg2_Baseline(&left, &right);
                }

                Enter_State(DIR_STATE_APPLY_DIRECTION);
                break;
            }

            VS_WheelCommandDirection = VS_CurrentDirection;
            VS_Direction = VS_CurrentDirection;
            Set_Active_Brake_Handover(0U);
            Set_Slope_Assist_Active(0U);

            if (requested_direction != VS_CurrentDirection)
            {
                Set_Active_Brake_Handover(
                    Active_Brake_Handover_Required(base_rpm,
                                                   &left,
                                                   &right,
                                                   snapshots_available));
                s_auto_resync_count = 0U;
                VS_WheelSyncRecoveryActive = 0U;
                VS_WheelSyncRecoveryCount = 0U;
                Enter_State(DIR_STATE_RAMP_DOWN);
                break;
            }

            if ((base_rpm > 0U) && (VS_RPM_Wheel_Current == 0U))
            {
                /*
                 * Do not fault before sending any wheel command. Feedback may
                 * start only after the controller receives a non-zero command.
                 */
                Prepare_Pickup(base_rpm,
                               VS_CurrentDirection,
                               &left,
                               &right,
                               snapshots_available);
                break;
            }

            VS_RPM_Wheel_Current =
                Apply_RPM_Slew_Local(base_rpm,
                                     VS_RPM_Wheel_Current,
                                     VC_RPM_SLEW_RATE);
            VS_RPM_Left = VS_RPM_Wheel_Current;
            VS_RPM_Right = VS_RPM_Wheel_Current;
            VS_WheelPickupCommonRPM = VS_RPM_Wheel_Current;
            break;
        }

        case DIR_STATE_RAMP_DOWN:
        {
            VS_WheelCommandDirection =
                Direction_IsValid(VS_CurrentDirection) ?
                VS_CurrentDirection : s_applied_direction;

            /*
             * Dynamic direction changes retain the low-RPM speed-loop command
             * that provides electronic hill-hold/braking. We ramp to the
             * configured active-brake floor instead of opening the torque loop
             * with a zero-RPM command. Initial arming from rest still uses zero.
             */
            transition_hold_rpm = Get_Transition_Hold_RPM();

            VS_RPM_Wheel_Current =
                Apply_RPM_Slew_Local(transition_hold_rpm,
                                     VS_RPM_Wheel_Current,
                                     VC_DIR_RAMP_STEP_RPM);
            Set_Equal_Wheel_Command(VS_RPM_Wheel_Current,
                                    VS_WheelCommandDirection);

            if (VS_RPM_Wheel_Current == transition_hold_rpm)
            {
                if (snapshots_available != 0U)
                {
                    Set_Msg1_Baseline(&left, &right);
                }
                Enter_State(DIR_STATE_WAIT_ZERO);
            }
            break;
        }

        case DIR_STATE_WAIT_ZERO:
        {
            uint8_t flat_zero_ready;
            uint8_t slope_rollback_ready;
            uint8_t slope_creep_ready;
            uint8_t near_zero_timeout_ready;
            uint16_t max_feedback_rpm;
            uint16_t feedback_diff_rpm;

            transition_hold_rpm = Get_Transition_Hold_RPM();

            /*
             * Keep the OLD direction + VC_ACTIVE_BRK_MIN_RPM while deciding
             * whether this is a normal flat-ground stop or a slope/load case.
             * There is no coast command during this observation period.
             */
            Set_Equal_Wheel_Command(transition_hold_rpm,
                                    Direction_IsValid(VS_CurrentDirection) ?
                                    VS_CurrentDirection : s_applied_direction);

            /*
             * If the operator changes the switch back before the handover,
             * cancel the reversal immediately. Do not wait for a timeout and
             * do not apply the opposite direction for even one cycle.
             */
            if (requested_direction == VS_CurrentDirection)
            {
                Set_Slope_Assist_Active(0U);

                if (base_rpm > 0U)
                {
                    Prepare_Pickup(base_rpm,
                                   VS_CurrentDirection,
                                   &left,
                                   &right,
                                   snapshots_available);
                }
                else
                {
                    Set_Active_Brake_Handover(0U);
                    Set_Slope_Assist_Active(0U);
                    Set_Equal_Wheel_Command(0U, VS_CurrentDirection);
                    Enter_State(DIR_STATE_IDLE);
                }
                break;
            }

            s_state_timer_ms =
                Saturating_Add_ms(s_state_timer_ms,
                                  MACHINE_DRIVE_TASK_PERIOD_MS,
                                  VC_DIR_WAIT_ZERO_TIMEOUT_MS);
            VS_DirectionStateTimer_ms = s_state_timer_ms;

            flat_zero_ready = 0U;
            slope_rollback_ready = 0U;
            slope_creep_ready = 0U;
            near_zero_timeout_ready = 0U;
            max_feedback_rpm = 0U;
            feedback_diff_rpm = 0U;

            if (snapshots_available != 0U)
            {
                max_feedback_rpm = Max_U16(left.speed_rpm,
                                           right.speed_rpm);
                feedback_diff_rpm = AbsDiff_U16(left.speed_rpm,
                                                right.speed_rpm);
            }

            if ((snapshots_available != 0U) && (msg1_pair_new != 0U))
            {
                /*
                 * FLAT-GROUND / NORMAL PATH:
                 * Never reverse while normal deceleration is still happening.
                 * Require both wheels to be genuinely near zero for multiple
                 * fresh feedback pairs. This is the key jerk-prevention path.
                 */
                if ((left.speed_rpm <= VC_DIR_ZERO_FEEDBACK_RPM) &&
                    (right.speed_rpm <= VC_DIR_ZERO_FEEDBACK_RPM))
                {
                    if (s_zero_confirm_pairs <
                        VC_DIR_ZERO_CONFIRM_FRESH_PAIRS)
                    {
                        s_zero_confirm_pairs++;
                    }
                }
                else
                {
                    s_zero_confirm_pairs = 0U;
                }

#if VC_DIR_SLOPE_HANDOVER_ENABLE
                /*
                 * SLOPE/LOAD PATH B:
                 * The old slope fix used only time + RPM (500 ms / 120 RPM).
                 * That could fire on flat ground while the machine was still
                 * normally decelerating and produce the short random-side jerk.
                 *
                 * Now a low-speed handover is allowed only after the feedback
                 * speed stops decelerating for several genuinely fresh pairs.
                 * A still-decreasing speed is classified as FLAT/NORMAL and is
                 * never force-reversed.
                 */
                if ((s_active_brake_handover != 0U) &&
                    (Slope_Persistent_Creep_Detected(&left,
                                                     &right,
                                                     s_state_timer_ms,
                                                     VS_CurrentDirection,
                                                     requested_direction) != 0U))
                {
                    slope_creep_ready = 1U;
                }
#endif
            }

#if VC_DIR_SLOPE_HANDOVER_ENABLE
            if ((snapshots_available != 0U) && (msg2_pair_new != 0U) &&
                (s_active_brake_handover != 0U))
            {
                /*
                 * SLOPE/LOAD PATH A:
                 * Gravity has already rolled BOTH wheels in the requested
                 * direction. In that case, waiting for a zero-speed sample is
                 * counterproductive. Use a bounded no-coast handover.
                 */
                if (Slope_Rollback_Direction_Detected(&left,
                                                      &right,
                                                      requested_direction) != 0U)
                {
                    if (s_slope_rollback_confirm_pairs <
                        VC_DIR_SLOPE_ROLLBACK_CONFIRM_FRESH_PAIRS)
                    {
                        s_slope_rollback_confirm_pairs++;
                    }
                }
                else
                {
                    s_slope_rollback_confirm_pairs = 0U;
                }

                if (s_slope_rollback_confirm_pairs >=
                    VC_DIR_SLOPE_ROLLBACK_CONFIRM_FRESH_PAIRS)
                {
                    slope_rollback_ready = 1U;
                }
            }
#endif

            if (s_zero_confirm_pairs >= VC_DIR_ZERO_CONFIRM_FRESH_PAIRS)
            {
                flat_zero_ready = 1U;
            }

            /*
             * Last-resort almost-zero case. This is intentionally restricted
             * to a tiny speed/difference window; unlike the previous 220-RPM
             * timeout fallback it cannot reverse a normally decelerating
             * machine at meaningful speed.
             */
            if ((s_state_timer_ms >= VC_DIR_WAIT_ZERO_TIMEOUT_MS) &&
                (snapshots_available != 0U) &&
                (Msg1_IsFresh(&left) != 0U) &&
                (Msg1_IsFresh(&right) != 0U) &&
                (max_feedback_rpm <= VC_DIR_TIMEOUT_NEAR_ZERO_RPM) &&
                (feedback_diff_rpm <= VC_DIR_TIMEOUT_NEAR_ZERO_MAX_DIFF_RPM))
            {
                near_zero_timeout_ready = 1U;
            }

            if ((flat_zero_ready != 0U) ||
                (slope_rollback_ready != 0U) ||
                (slope_creep_ready != 0U) ||
                (near_zero_timeout_ready != 0U))
            {
                /*
                 * Adaptive handover result:
                 *   - flat_zero_ready / near_zero_timeout_ready:
                 *       jerk-free strict-stop handover.
                 *   - slope_rollback_ready / slope_creep_ready:
                 *       slope-safe no-coast handover.
                 *
                 * Both cases keep the wheel commands EQUAL. Dynamic reversals
                 * retain the active-brake floor; no left/right correction is
                 * introduced here.
                 */
                if ((slope_rollback_ready != 0U) ||
                    (slope_creep_ready != 0U))
                {
                    Set_Slope_Assist_Active(1U);
                }
                else
                {
                    Set_Slope_Assist_Active(0U);
                }

                VS_WheelSyncFault = WHEEL_SYNC_FAULT_NONE;
                s_applied_direction = requested_direction;

                if (s_active_brake_handover != 0U)
                {
                    Set_Equal_Wheel_Command(transition_hold_rpm,
                                            s_applied_direction);
                }
                else
                {
                    /* Static/startup path only. */
                    Set_Equal_Wheel_Command(0U, DIR_STOP);
                    Enter_State(DIR_STATE_NEUTRAL);
                    break;
                }

                if (snapshots_available != 0U)
                {
                    Set_Msg2_Baseline(&left, &right);
                }

                Enter_State(DIR_STATE_APPLY_DIRECTION);
            }
            else if (s_state_timer_ms >= VC_DIR_WAIT_ZERO_TIMEOUT_MS)
            {
                if (s_active_brake_handover != 0U)
                {
                    /*
                     * Safety default = BRAKE, not reverse and not coast.
                     * If neither flat-stop nor slope evidence is trustworthy,
                     * hold the old direction at VC_ACTIVE_BRK_MIN_RPM and wait.
                     * This prevents both failure modes:
                     *   1) no 0-RPM/DIR_STOP slope release, and
                     *   2) no blind high-residual-speed reversal on flat ground.
                     */
                    VS_WheelSyncFault = WHEEL_SYNC_FAULT_ZERO_TIMEOUT;
                    s_state_timer_ms = VC_DIR_WAIT_ZERO_TIMEOUT_MS;
                    VS_DirectionStateTimer_ms = s_state_timer_ms;
                }
                else
                {
                    Enter_Fault(WHEEL_SYNC_FAULT_ZERO_TIMEOUT);
                }
            }
            break;
        }

        case DIR_STATE_NEUTRAL:
        {
            Set_Equal_Wheel_Command(0U, DIR_STOP);

            s_state_timer_ms =
                Saturating_Add_ms(s_state_timer_ms,
                                  MACHINE_DRIVE_TASK_PERIOD_MS,
                                  VC_DIR_TRUE_NEUTRAL_HOLD_MS);
            VS_DirectionStateTimer_ms = s_state_timer_ms;

            if (s_state_timer_ms >= VC_DIR_TRUE_NEUTRAL_HOLD_MS)
            {
                s_applied_direction = requested_direction;

                if (snapshots_available != 0U)
                {
                    Set_Msg2_Baseline(&left, &right);
                }

                Enter_State(DIR_STATE_APPLY_DIRECTION);
            }
            break;
        }

        case DIR_STATE_APPLY_DIRECTION:
        {
            if (s_applied_direction != requested_direction)
            {
                s_applied_direction = requested_direction;
                s_state_timer_ms = 0U;

                if (snapshots_available != 0U)
                {
                    Set_Msg2_Baseline(&left, &right);
                }
            }

            transition_hold_rpm = Get_Transition_Hold_RPM();
            direction_apply_hold_ms =
                (s_active_brake_handover != 0U) ?
                VC_DIR_ACTIVE_BRAKE_APPLY_HOLD_MS :
                VC_DIR_APPLY_ZERO_HOLD_MS;

            Set_Equal_Wheel_Command(transition_hold_rpm,
                                    s_applied_direction);

            s_state_timer_ms =
                Saturating_Add_ms(s_state_timer_ms,
                                  MACHINE_DRIVE_TASK_PERIOD_MS,
                                  direction_apply_hold_ms);
            VS_DirectionStateTimer_ms = s_state_timer_ms;

            if (s_state_timer_ms >= direction_apply_hold_ms)
            {
                if (snapshots_available != 0U)
                {
                    Set_Msg2_Baseline(&left, &right);
                }
                Enter_State(DIR_STATE_WAIT_DIRECTION_ACK);
            }
            break;
        }

        case DIR_STATE_WAIT_DIRECTION_ACK:
        {
            /*
             * Direction echo is observed, not used as a hard zero-RPM gate.
             * This fixes the no-command condition seen when command_dir remains
             * STOP until the controller actually receives a non-zero RPM.
             */
            transition_hold_rpm = Get_Transition_Hold_RPM();
            Set_Equal_Wheel_Command(transition_hold_rpm,
                                    s_applied_direction);

            if (requested_direction != s_applied_direction)
            {
                if (s_active_brake_handover != 0U)
                {
                    s_applied_direction = requested_direction;
                    Set_Equal_Wheel_Command(transition_hold_rpm,
                                            s_applied_direction);
                    Enter_State(DIR_STATE_APPLY_DIRECTION);
                }
                else
                {
                    Set_Equal_Wheel_Command(0U, DIR_STOP);
                    Enter_State(DIR_STATE_NEUTRAL);
                }
                break;
            }

            s_state_timer_ms =
                Saturating_Add_ms(s_state_timer_ms,
                                  MACHINE_DRIVE_TASK_PERIOD_MS,
                                  VC_DIR_ACK_OBSERVE_MS);
            VS_DirectionStateTimer_ms = s_state_timer_ms;

            if ((snapshots_available != 0U) && (msg2_pair_new != 0U))
            {
                if ((left.command_dir == s_applied_direction) &&
                    (right.command_dir == s_applied_direction))
                {
                    if (s_ack_confirm_pairs <
                        VC_DIR_ACK_CONFIRM_FRESH_PAIRS)
                    {
                        s_ack_confirm_pairs++;
                    }
                }
                else
                {
                    s_ack_confirm_pairs = 0U;
                }
            }

            if ((s_ack_confirm_pairs >= VC_DIR_ACK_CONFIRM_FRESH_PAIRS) ||
                (s_state_timer_ms >= VC_DIR_ACK_OBSERVE_MS))
            {
                Complete_Direction_Apply(base_rpm,
                                         &left,
                                         &right,
                                         snapshots_available);
            }
            break;
        }

        case DIR_STATE_SYNC_PICKUP:
        {
            if (requested_direction != s_applied_direction)
            {
                Set_Active_Brake_Handover(
                    Active_Brake_Handover_Required(base_rpm,
                                                   &left,
                                                   &right,
                                                   snapshots_available));
                Enter_State(DIR_STATE_RAMP_DOWN);
                break;
            }

            if (base_rpm == 0U)
            {
                s_pickup_common_rpm = 0U;
                Set_Equal_Wheel_Command(0U, s_applied_direction);
                Enter_State(DIR_STATE_IDLE);
                break;
            }

            s_state_timer_ms =
                Saturating_Add_ms(s_state_timer_ms,
                                  MACHINE_DRIVE_TASK_PERIOD_MS,
                                  VC_DIR_PICKUP_TOTAL_TIMEOUT_MS);
            VS_DirectionStateTimer_ms = s_state_timer_ms;

            if (s_pickup_retry_hold_active != 0U)
            {
                transition_hold_rpm = Get_Transition_Hold_RPM();
                Set_Equal_Wheel_Command(transition_hold_rpm,
                                        s_applied_direction);

                s_pickup_retry_hold_timer_ms =
                    Saturating_Add_ms(s_pickup_retry_hold_timer_ms,
                                      MACHINE_DRIVE_TASK_PERIOD_MS,
                                      VC_DIR_PICKUP_RETRY_HOLD_MS);

                if (s_pickup_retry_hold_timer_ms >=
                    VC_DIR_PICKUP_RETRY_HOLD_MS)
                {
                    retry_start_rpm =
                        (uint16_t)((uint16_t)VC_DIR_PICKUP_START_RPM +
                                   ((uint16_t)s_pickup_retry_count *
                                    (uint16_t)VC_DIR_PICKUP_RETRY_INCREMENT_RPM));

                    retry_start_rpm = Min_U16(retry_start_rpm,
                                              VC_DIR_PICKUP_MAX_RPM);
                    s_pickup_common_rpm = Min_U16(base_rpm,
                                                  retry_start_rpm);

                    if ((s_active_brake_handover != 0U) &&
                        (base_rpm >= VC_DIR_ACTIVE_BRAKE_HOLD_RPM) &&
                        (s_pickup_common_rpm <
                         VC_DIR_ACTIVE_BRAKE_HOLD_RPM))
                    {
                        s_pickup_common_rpm =
                            VC_DIR_ACTIVE_BRAKE_HOLD_RPM;
                    }
                    s_pickup_retry_hold_active = 0U;
                    s_pickup_retry_hold_timer_ms = 0U;
                    s_pickup_freeze = 0U;
                    s_pickup_feedback_wait_ms = 0U;
                    Set_Equal_Wheel_Command(s_pickup_common_rpm,
                                            s_applied_direction);
                }
                break;
            }

            pickup_target = Min_U16(base_rpm, VC_DIR_PICKUP_MAX_RPM);

            if (s_pickup_freeze == 0U)
            {
                s_pickup_common_rpm =
                    Apply_RPM_Slew_Local(pickup_target,
                                         s_pickup_common_rpm,
                                         VC_DIR_PICKUP_RAMP_STEP_RPM);
            }

            Set_Equal_Wheel_Command(s_pickup_common_rpm,
                                    s_applied_direction);

            s_pickup_feedback_wait_ms =
                Saturating_Add_ms(s_pickup_feedback_wait_ms,
                                  MACHINE_DRIVE_TASK_PERIOD_MS,
                                  VC_DIR_PICKUP_FEEDBACK_START_TIMEOUT_MS);

            /*
             * MSG1 and MSG2 are asynchronous. Process a new direction frame
             * independently so it cannot be consumed before the next RPM pair.
             */
            if ((snapshots_available != 0U) && (msg2_pair_new != 0U))
            {
                left_moving =
                    (uint8_t)((left.speed_rpm >=
                               VC_DIR_PICKUP_MOVING_RPM) ? 1U : 0U);
                right_moving =
                    (uint8_t)((right.speed_rpm >=
                               VC_DIR_PICKUP_MOVING_RPM) ? 1U : 0U);

                s_left_direction_ok =
                    Feedback_Direction_OK_Or_Unknown(left.feedback_dir,
                                                     s_applied_direction);
                s_right_direction_ok =
                    Feedback_Direction_OK_Or_Unknown(right.feedback_dir,
                                                     s_applied_direction);

                if (((left_moving != 0U) &&
                     (left.command_dir == s_applied_direction) &&
                     (s_left_direction_ok == 0U)) ||
                    ((right_moving != 0U) &&
                     (right.command_dir == s_applied_direction) &&
                     (s_right_direction_ok == 0U)))
                {
                    if (s_direction_mismatch_pairs < 255U)
                    {
                        s_direction_mismatch_pairs++;
                    }
                }
                else
                {
                    s_direction_mismatch_pairs = 0U;
                }

                if (s_direction_mismatch_pairs >=
                    VC_DIR_DIRECTION_MISMATCH_CONFIRM_PAIRS)
                {
                    Enter_Fault(WHEEL_SYNC_FAULT_DIRECTION_MISMATCH);
                    break;
                }
            }

            if ((snapshots_available != 0U) && (msg1_pair_new != 0U))
            {
                s_pickup_seen_fresh_pair = 1U;
                s_pickup_feedback_wait_ms = 0U;

                feedback_diff = AbsDiff_U16(left.speed_rpm,
                                            right.speed_rpm);
                VS_WheelFeedbackDiff_RPM = feedback_diff;

                left_moving =
                    (uint8_t)((left.speed_rpm >=
                               VC_DIR_PICKUP_MOVING_RPM) ? 1U : 0U);
                right_moving =
                    (uint8_t)((right.speed_rpm >=
                               VC_DIR_PICKUP_MOVING_RPM) ? 1U : 0U);

                left_direction_ok = s_left_direction_ok;
                right_direction_ok = s_right_direction_ok;

                if ((left_moving != 0U) ^ (right_moving != 0U))
                {
                    s_pickup_freeze = 1U;
                    s_single_wheel_condition = 1U;
                    s_pickup_stable_pairs = 0U;

                    if (Max_U16(left.speed_rpm, right.speed_rpm) >=
                        VC_DIR_PICKUP_SINGLE_WHEEL_ABORT_RPM)
                    {
                        Start_Pickup_Retry(&left,
                                           &right,
                                           snapshots_available);
                        break;
                    }
                }
                else if ((left_moving != 0U) && (right_moving != 0U))
                {
                    s_single_wheel_condition = 0U;
                    s_pickup_single_wheel_timer_ms = 0U;
                    s_pickup_freeze =
                        (uint8_t)((feedback_diff >
                                   VC_DIR_PICKUP_ALLOWED_DIFF_RPM) ? 1U : 0U);

                    s_filtered_left_rpm =
                        (uint16_t)((((uint32_t)s_filtered_left_rpm * 3U) +
                                    (uint32_t)left.speed_rpm) / 4U);
                    s_filtered_right_rpm =
                        (uint16_t)((((uint32_t)s_filtered_right_rpm * 3U) +
                                    (uint32_t)right.speed_rpm) / 4U);

                    feedback_diff = AbsDiff_U16(s_filtered_left_rpm,
                                                s_filtered_right_rpm);

                    if ((left_direction_ok != 0U) &&
                        (right_direction_ok != 0U) &&
                        (feedback_diff <= VC_DIR_PICKUP_ALLOWED_DIFF_RPM))
                    {
                        if (s_pickup_stable_pairs <
                            VC_DIR_PICKUP_STABLE_FRESH_PAIRS)
                        {
                            s_pickup_stable_pairs++;
                        }
                    }
                    else
                    {
                        s_pickup_stable_pairs = 0U;
                    }
                }
                else
                {
                    s_pickup_freeze = 0U;
                    s_single_wheel_condition = 0U;
                    s_pickup_single_wheel_timer_ms = 0U;
                    s_pickup_stable_pairs = 0U;
                }
            }

            if (s_single_wheel_condition != 0U)
            {
                s_pickup_single_wheel_timer_ms =
                    Saturating_Add_ms(s_pickup_single_wheel_timer_ms,
                                      MACHINE_DRIVE_TASK_PERIOD_MS,
                                      VC_DIR_PICKUP_SINGLE_WHEEL_TIMEOUT_MS);
            }
            else
            {
                s_pickup_single_wheel_timer_ms = 0U;
            }

            if (s_pickup_single_wheel_timer_ms >=
                VC_DIR_PICKUP_SINGLE_WHEEL_TIMEOUT_MS)
            {
                Start_Pickup_Retry(&left,
                                   &right,
                                   snapshots_available);
            }
            else if ((s_pickup_seen_fresh_pair == 0U) &&
                     (s_pickup_feedback_wait_ms >=
                      VC_DIR_PICKUP_FEEDBACK_START_TIMEOUT_MS))
            {
                Enter_Fault(WHEEL_SYNC_FAULT_FEEDBACK_MISSING);
            }
            else if (s_pickup_stable_pairs >=
                     VC_DIR_PICKUP_STABLE_FRESH_PAIRS)
            {
                VS_RPM_Wheel_Current = s_pickup_common_rpm;
                Enter_State(DIR_STATE_RAMP_UP);
            }
            else if (s_state_timer_ms >= VC_DIR_PICKUP_TOTAL_TIMEOUT_MS)
            {
                Enter_Fault(WHEEL_SYNC_FAULT_PICKUP_TIMEOUT);
            }
            break;
        }

        case DIR_STATE_RAMP_UP:
        {
            uint16_t supervised_target;

            if (requested_direction != VS_CurrentDirection)
            {
                Set_Active_Brake_Handover(
                    Active_Brake_Handover_Required(base_rpm,
                                                   &left,
                                                   &right,
                                                   snapshots_available));
                s_auto_resync_count = 0U;
                VS_WheelSyncRecoveryActive = 0U;
                VS_WheelSyncRecoveryCount = 0U;
                Enter_State(DIR_STATE_RAMP_DOWN);
                break;
            }

            /*
             * The synchronization supervisor is a pickup-only function. It
             * ramps equally only to a low release speed, then returns control
             * to the normal drive slew. The previous code supervised all the
             * way to the full throttle target; a normal transient difference
             * above 240 RPM could therefore latch a fault during acceleration.
             */
            supervised_target = Min_U16(base_rpm,
                                        VC_DIR_RAMPUP_SYNC_RELEASE_RPM);

            VS_RPM_Wheel_Current =
                Apply_RPM_Slew_Local(supervised_target,
                                     VS_RPM_Wheel_Current,
                                     VC_DIR_RAMP_UP_STEP_RPM);
            Set_Equal_Wheel_Command(VS_RPM_Wheel_Current,
                                    VS_CurrentDirection);

            if ((snapshots_available != 0U) && (msg1_pair_new != 0U))
            {
                feedback_diff = AbsDiff_U16(left.speed_rpm,
                                            right.speed_rpm);
                VS_WheelFeedbackDiff_RPM = feedback_diff;

                if (feedback_diff >= VC_DIR_RAMPUP_HARD_DIFF_RPM)
                {
                    if (s_rampup_hard_diff_pairs <
                        VC_DIR_RAMPUP_HARD_DIFF_CONFIRM_PAIRS)
                    {
                        s_rampup_hard_diff_pairs++;
                    }
                }
                else
                {
                    s_rampup_hard_diff_pairs = 0U;
                }
            }

            if (s_rampup_hard_diff_pairs >=
                VC_DIR_RAMPUP_HARD_DIFF_CONFIRM_PAIRS)
            {
                Start_Auto_Resync(WHEEL_SYNC_FAULT_SINGLE_WHEEL_PICKUP);
            }
            else if (VS_RPM_Wheel_Current == supervised_target)
            {
                /* Successful low-speed synchronized pickup. */
                s_auto_resync_count = 0U;
                Set_Active_Brake_Handover(0U);
                Set_Slope_Assist_Active(0U);
                VS_WheelSyncRecoveryActive = 0U;
                VS_WheelSyncRecoveryCount = 0U;
                Enter_State(DIR_STATE_IDLE);
            }
            break;
        }

        case DIR_STATE_FAULT:
        default:
        {
            VS_WheelSyncRecoveryActive = 0U;
            Set_Slope_Assist_Active(0U);
            Set_Equal_Wheel_Command(0U, DIR_STOP);

            if (base_rpm == 0U)
            {
                s_fault_clear_timer_ms =
                    Saturating_Add_ms(s_fault_clear_timer_ms,
                                      MACHINE_DRIVE_TASK_PERIOD_MS,
                                      VC_DIR_FAULT_CLEAR_NO_FEEDBACK_MS);

                if ((snapshots_available != 0U) && (msg1_pair_new != 0U))
                {
                    if ((left.speed_rpm <= VC_DIR_FAULT_CLEAR_FEEDBACK_RPM) &&
                        (right.speed_rpm <= VC_DIR_FAULT_CLEAR_FEEDBACK_RPM))
                    {
                        if (s_fault_clear_pairs <
                            VC_DIR_FAULT_CLEAR_CONFIRM_FRESH_PAIRS)
                        {
                            s_fault_clear_pairs++;
                        }
                    }
                    else
                    {
                        s_fault_clear_pairs = 0U;
                        s_fault_clear_timer_ms = 0U;
                    }
                }
            }
            else
            {
                s_fault_clear_pairs = 0U;
                s_fault_clear_timer_ms = 0U;
            }

            if ((s_fault_clear_pairs >=
                 VC_DIR_FAULT_CLEAR_CONFIRM_FRESH_PAIRS) ||
                ((base_rpm == 0U) &&
                 (s_fault_clear_timer_ms >=
                  VC_DIR_FAULT_CLEAR_NO_FEEDBACK_MS)))
            {
                VS_CurrentDirection = DIR_STOP;
                VS_Direction = DIR_STOP;
                VS_WheelCommandDirection = DIR_STOP;
                VS_WheelSyncFault = WHEEL_SYNC_FAULT_NONE;
                s_applied_direction = DIR_STOP;
                s_pickup_retry_count = 0U;
                s_auto_resync_count = 0U;
                Set_Active_Brake_Handover(0U);
                Set_Slope_Assist_Active(0U);
                VS_WheelSyncRecoveryActive = 0U;
                VS_WheelSyncRecoveryCount = 0U;
                Set_Equal_Wheel_Command(0U, DIR_STOP);
                Enter_State(DIR_STATE_IDLE);
            }
            break;
        }
    }
}
