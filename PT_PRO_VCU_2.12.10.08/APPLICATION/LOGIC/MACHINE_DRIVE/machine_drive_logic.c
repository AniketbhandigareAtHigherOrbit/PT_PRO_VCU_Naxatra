#include "machine_drive_logic.h"
#include "vehicle_state.h"
#include "vehicle_config.h"
#include "wheel_direction_sync.h"

#ifndef MACHINE_DRIVE_TASK_PERIOD_MS
#ifdef ONROAD_TASK_PERIOD_MS
#define MACHINE_DRIVE_TASK_PERIOD_MS      ONROAD_TASK_PERIOD_MS
#else
#define MACHINE_DRIVE_TASK_PERIOD_MS      20U
#endif
#endif

static uint16_t VS_TurnGradDiff_RPM   = 0U;
static uint8_t  VS_TurnPrev           = 0U;

/*
 * Turn/synchronizer ownership latch.
 *
 * 0: feedback direction synchronizer has not yet completed/confirmed a
 *    stable direction since reset or a direction request change.
 * 1: stable direction has been established; a single turn may temporarily
 *    bypass left/right feedback symmetry enforcement.
 */
static uint8_t  s_turn_override_armed = 0U;

/* ================= Battery Fuse Save runtime ================= */
#define BATT_DAMP_STATE_WAIT_ENTRY   (0U)
#define BATT_DAMP_STATE_ACTIVE       (1U)
#define BATT_DAMP_STATE_REENTRY_WAIT (2U)

//uint8_t  VS_BatteryCutoffRotorLock = 0U;
#define BATT_LOCK_REASON_NONE             (0U)
#define BATT_LOCK_REASON_CRITICAL_CURRENT (1U)
#define BATT_LOCK_REASON_CUTOFF           (2U)
#define BATT_LOCK_REASON_SLOPE            (3U)

/*
 * Public rotor-lock diagnostic bit mask.
 *
 * VS_RotorLockCause reports every currently active protection cause.  The
 * value is intentionally a bit mask so that an SOC lock and a battery-current
 * lock can be reported at the same time without losing information.
 *
 * IoT mapping:
 *   0x01 = Battery 1 low SOC
 *   0x02 = Battery 2 low SOC
 *   0x04 = Two-battery average SOC low
 *   0x08 = Critical battery current
 *   0x10 = Direct battery cut-off pattern
 *   0x20 = Battery-current slope/cut-off pattern
 *   0x40 = Startup waiting for first valid SOC data
 *
 * VS_RotorLockLastCause and the trip snapshot remain available after the
 * active lock clears, allowing a slower IoT broadcast task to report the
 * event reliably.  They are replaced only when a new lock event begins.
 */

#define ROTOR_LOCK_CAUSE_NONE             (0x00U)
#define ROTOR_LOCK_CAUSE_SOC_BATTERY1     (0x01U)
#define ROTOR_LOCK_CAUSE_SOC_BATTERY2     (0x02U)
#define ROTOR_LOCK_CAUSE_SOC_AVERAGE      (0x04U)
#define ROTOR_LOCK_CAUSE_CRITICAL_CURRENT (0x08U)
#define ROTOR_LOCK_CAUSE_BATTERY_CUTOFF   (0x10U)
#define ROTOR_LOCK_CAUSE_BATTERY_SLOPE    (0x20U)
#define ROTOR_LOCK_CAUSE_SOC_DATA_WAIT    (0x40U)

uint8_t  VS_RotorLockActive                  = 0U;
uint8_t  VS_RotorLockCause                   = ROTOR_LOCK_CAUSE_NONE;
uint8_t  VS_RotorLockLastCause               = ROTOR_LOCK_CAUSE_NONE;
uint8_t  VS_RotorLockTripSoc1                = 0xFFU;
uint8_t  VS_RotorLockTripSoc2                = 0xFFU;
uint16_t VS_RotorLockTripBattery1Current_raw = 0U;
uint16_t VS_RotorLockTripBattery2Current_raw = 0U;

static uint8_t  s_batt_fuse_latched = 0U;
static uint8_t  s_batt_fuse_reduction_percent_internal = 0U;

/*
 * 46% normal-damping hold:
 * Once the HIGH (46%) region is reached during an ACTIVE damping window,
 * keep the wheel reduction at 46% until that existing ACTIVE window ends.
 * This prevents the protection from immediately relaxing to 35/15/10%
 * simply because the 46% reduction itself lowered battery current.
 *
 * This flag affects only the normal 46% damping level.
 * It does not change 10%, 15%, 35%, critical-current, cutoff, slope,
 * direction-sync, turn, or rotor behavior.
 */
static uint8_t  s_batt_high_reduction_hold_active = 0U;

static uint16_t s_batt_fuse_confirm_ms = 0U;
static uint16_t s_batt_critical_current_ms = 0U;
static uint16_t s_batt_cutoff_confirm_ms = 0U;

static uint16_t s_batt_damp_entry_ms = 0U;
static uint16_t s_batt_damp_active_ms = 0U;
static uint16_t s_batt_damp_reentry_ms = 0U;

static uint8_t  s_batt_damp_state_internal = BATT_DAMP_STATE_WAIT_ENTRY;
static uint8_t  s_batt_lock_reason = BATT_LOCK_REASON_NONE;

uint8_t s_psm_soc_lock_cause = ROTOR_LOCK_CAUSE_NONE;
static uint8_t  s_psm_soc_lock_valid_mask = 0U;
static uint8_t  s_psm_soc_immediate_inhibit = 1U;
static uint8_t  s_psm_soc_immediate_cause = ROTOR_LOCK_CAUSE_SOC_DATA_WAIT;
static uint8_t  s_psm_soc_data_seen = 0U;
static uint8_t  s_rotor_lock_previous_cause = ROTOR_LOCK_CAUSE_NONE;

static uint16_t s_b1_prev_current_raw = 0U;
static uint16_t s_b2_prev_current_raw = 0U;
static uint8_t  s_batt_prev_pair_valid = 0U;

/* Battery fuse-save tuning parameters are defined centrally in
 * vehicle_config.h. Do not duplicate or override them in this file. */

static uint8_t Decode_Rotor_Mode_From_Analog(uint16_t adc_value, uint8_t *mode_out)
{
    if (mode_out == 0)
    {
        return 0U;
    }

    if ((adc_value >= VC_ROTOR_MODE0_ADC_MIN) && (adc_value <= VC_ROTOR_MODE0_ADC_MAX))
    {
        *mode_out = 0U;
        return 1U;
    }
    else if ((adc_value >= VC_ROTOR_MODE1_ADC_MIN) && (adc_value <= VC_ROTOR_MODE1_ADC_MAX))
    {
        *mode_out = 1U;
        return 1U;
    }
    else if ((adc_value >= VC_ROTOR_MODE2_ADC_MIN) && (adc_value <= VC_ROTOR_MODE2_ADC_MAX))
    {
        *mode_out = 2U;
        return 1U;
    }
    else if ((adc_value >= VC_ROTOR_MODE3_ADC_MIN) && (adc_value <= VC_ROTOR_MODE3_ADC_MAX))
    {
        *mode_out = 3U;
        return 1U;
    }

    *mode_out = 0xFFU;
    return 0U;
}

/* Start in user Mode 1 until a valid selector value is first observed.
 * After selector input is seen, invalid values safely produce zero RPM. */
static uint8_t Select_Rotor_Mode(uint16_t adc_value, uint8_t *mode_out)
{
    static uint8_t selector_seen = 0U;
    uint8_t valid = Decode_Rotor_Mode_From_Analog(adc_value, mode_out);

    if (valid != 0U)
    {
        selector_seen = 1U;
    }
    else if ((selector_seen == 0U) && (mode_out != 0))
    {
        *mode_out = 0U; /* User Mode 1, VC_ROTOR_MODE1_RPM (2000 RPM). */
        valid = 1U;
    }

    return valid;
}

/* ================= Helper: ADC → RPM ================= */
static uint16_t Throttle_To_RPM(uint16_t adc)
{
    return (uint16_t)(((uint32_t)adc * (uint32_t)VC_WHEEL_MAX_RPM) / (uint32_t)VC_ADC_MAX_VALUE);
}

/* ================= Helper: Slew limiter ================= */
static uint16_t Apply_RPM_Slew(uint16_t target,
                               uint16_t current,
                               uint16_t slew)
{
    if (target > (uint16_t)(current + slew))
    {
        return (uint16_t)(current + slew);
    }
    else if (current > (uint16_t)(target + slew))
    {
        return (uint16_t)(current - slew);
    }
    else
    {
        return target;
    }
}

static uint16_t Reduce_RPM_By_Percent(uint16_t rpm, uint8_t reduce_percent)
{
    uint32_t remaining_percent;

    if (reduce_percent >= 100U)
    {
        return 0U;
    }

    remaining_percent = (uint32_t)(100U - reduce_percent);

    return (uint16_t)(((uint32_t)rpm * remaining_percent) / 100U);
}

static uint16_t Get_Total_Valid_Battery_Current_Raw(uint8_t b1_valid,
                                                     uint8_t b2_valid)
{
    uint16_t total_current = 0U;

    if ((b1_valid != 0U) && (b2_valid != 0U))
    {
        if(VS_Battery1.BatteryCurrent_raw >= VS_Battery2.BatteryCurrent_raw)
        {
            total_current = VS_Battery1.BatteryCurrent_raw;
            return total_current;
        }
        else
        {
            total_current = VS_Battery2.BatteryCurrent_raw;
            return total_current;
        }
    }

    if (b1_valid != 0U)
    {
        total_current += VS_Battery1.BatteryCurrent_raw;
    }

    if (b2_valid != 0U)
    {
        total_current += VS_Battery2.BatteryCurrent_raw;
    }

    return total_current;
}

static uint8_t Get_Battery_Damping_Reduction(uint16_t max_current)
{
    if (max_current >= VC_BATT_FUSE_HIGH_CURRENT_RAW)
    {
        return VC_BATT_FUSE_REDUCTION_HIGH_PERCENT;
    }
    else if (max_current >= VC_BATT_FUSE_DERATE2_CURRENT_RAW)
    {
        return VC_BATT_FUSE_REDUCTION_DERATE2_PERCENT;
    }
    else if (max_current >= VC_BATT_FUSE_DERATE1_CURRENT_RAW)
    {
        return VC_BATT_FUSE_REDUCTION_DERATE1_PERCENT;
    }

    /* Once damping is active, keep at least the warning reduction
     * for the full configured active duration. */
    
    return VC_BATT_FUSE_REDUCTION_WARN_PERCENT;
}

static void MachineDrive_BatteryFuseSaveStep(void)
{
#if VC_BATT_FUSE_SAVE_ENABLE
    uint8_t b1_valid;
    uint8_t b2_valid;
    uint8_t both_valid;
    uint8_t b1_zero;
    uint8_t b2_zero;
    uint8_t b1_other_high;
    uint8_t b2_other_high;
    uint8_t high_current;
    uint8_t direct_cutoff;
    uint8_t slope_condition;
    uint8_t fuse_condition;
    uint8_t battery_invalid;
    uint8_t current_reduction;

    uint16_t max_current;
    uint16_t b1_rise;
    uint16_t b2_rise;
    uint16_t b1_drop;
    uint16_t b2_drop;

    /*
     * VS_BattDampMonitor:
     *
     * Bit 0     = High current / critical-current event
     * Bit 1     = Damping active
     * Bit 2     = Re-entry wait active
     * Bit 3     = Battery cut-off/fuse pattern
     * Bit 4     = Protection latched
     * Bit 5     = No valid battery data
     * Bit 6-7   = Damping state
     * Bit 8-15  = Reduction percentage
     */

    high_current = 0U;
    direct_cutoff = 0U;
    slope_condition = 0U;
    fuse_condition = 0U;
    battery_invalid = 0U;
    current_reduction = 0U;

    b1_rise = 0U;
    b2_rise = 0U;
    b1_drop = 0U;
    b2_drop = 0U;

    /* =====================================================
     * 1. EXISTING LATCH HAS HIGHEST PRIORITY
     * ===================================================== */
    if (s_batt_fuse_latched != 0U)
    {
        VS_BatteryCutoffRotorLock = 1U;

        s_batt_fuse_reduction_percent_internal =
            VC_BATT_FUSE_REDUCTION_LATCH_PERCENT;

        VS_BattDampMonitor = 0U;

        if (s_batt_lock_reason == BATT_LOCK_REASON_CRITICAL_CURRENT)
        {
            VS_BattDampMonitor |= (uint16_t)(1U << 0);
        }

        if ((s_batt_lock_reason == BATT_LOCK_REASON_CUTOFF) ||
            (s_batt_lock_reason == BATT_LOCK_REASON_SLOPE))
        {
            VS_BattDampMonitor |= (uint16_t)(1U << 3);
        }

        VS_BattDampMonitor |= (uint16_t)(1U << 4);

        VS_BattDampMonitor |=
            (uint16_t)(((uint16_t)s_batt_damp_state_internal &
                        0x03U) << 6);

        VS_BattDampMonitor |=
            (uint16_t)(((uint16_t)
                        s_batt_fuse_reduction_percent_internal &
                        0xFFU) << 8);

        return;
    }

    /* =====================================================
     * 2. BATTERY VALIDITY
     *
     * Current protection remains active at low SOC.
     * Safety validity uses Alive + valid voltage.
     * ===================================================== */
    b1_valid =
        (uint8_t)(((VS_Battery1.Alive != 0U) &&
                   (VS_Battery1.BatteryVoltage_raw >
                    VC_BATT_VALID_VOLTAGE_RAW)) ? 1U : 0U);

    b2_valid =
        (uint8_t)(((VS_Battery2.Alive != 0U) &&
                   (VS_Battery2.BatteryVoltage_raw >
                    VC_BATT_VALID_VOLTAGE_RAW)) ? 1U : 0U);

    both_valid =
        (uint8_t)(((b1_valid != 0U) &&
                   (b2_valid != 0U)) ? 1U : 0U);

    /*
     * Single-battery operation is supported.
     * Only stop processing if BOTH batteries are invalid.
     */
    if ((b1_valid == 0U) && (b2_valid == 0U))
    {
        battery_invalid = 1U;

        s_batt_critical_current_ms = 0U;
        s_batt_cutoff_confirm_ms = 0U;
        s_batt_fuse_confirm_ms = 0U;

        s_batt_damp_entry_ms = 0U;
        s_batt_damp_active_ms = 0U;
        s_batt_damp_reentry_ms = 0U;

        s_batt_damp_state_internal =
            BATT_DAMP_STATE_WAIT_ENTRY;

        s_batt_fuse_reduction_percent_internal = 0U;
        s_batt_high_reduction_hold_active = 0U;

        s_batt_prev_pair_valid = 0U;

        s_b1_prev_current_raw =
            VS_Battery1.BatteryCurrent_raw;

        s_b2_prev_current_raw =
            VS_Battery2.BatteryCurrent_raw;

        VS_BattDampMonitor = 0U;

        VS_BattDampMonitor |=
            (uint16_t)(1U << 5);

        VS_BattDampMonitor |=
            (uint16_t)(((uint16_t)s_batt_damp_state_internal &
                        0x03U) << 6);

        return;
    }

    /* =====================================================
     * 3. GET CURRENT FROM VALID BATTERY/BATTERIES
     * ===================================================== */
    max_current =
        Get_Total_Valid_Battery_Current_Raw(b1_valid,
                                            b2_valid);

    high_current =
        (uint8_t)((max_current >=
                   VC_BATT_FUSE_WARN_CURRENT_RAW) ? 1U : 0U);

    /* =====================================================
     * 4. CRITICAL CURRENT ROTOR LOCK
     *
     * ANY valid battery >= VC_BATT_ROTOR_LOCK_CURRENT_RAW
     * continuously for VC_BATT_ROTOR_LOCK_CONFIRM_MS:
     *   - latch protection
     *   - lock rotor
     *   - apply latch wheel reduction
     *
     * Works with one or two valid batteries.
     * ===================================================== */
    if (((b1_valid != 0U) &&
         (VS_Battery1.BatteryCurrent_raw >=
          VC_BATT_ROTOR_LOCK_CURRENT_RAW)) ||
        ((b2_valid != 0U) &&
         (VS_Battery2.BatteryCurrent_raw >=
          VC_BATT_ROTOR_LOCK_CURRENT_RAW)))
    {
        if (s_batt_critical_current_ms <
            VC_BATT_ROTOR_LOCK_CONFIRM_MS)
        {
            s_batt_critical_current_ms =
                (uint16_t)(s_batt_critical_current_ms +
                           MACHINE_DRIVE_TASK_PERIOD_MS);

            if (s_batt_critical_current_ms >
                VC_BATT_ROTOR_LOCK_CONFIRM_MS)
            {
                s_batt_critical_current_ms =
                    VC_BATT_ROTOR_LOCK_CONFIRM_MS;
            }
        }
    }
    else
    {
        s_batt_critical_current_ms = 0U;
    }

    if (s_batt_critical_current_ms >=
        VC_BATT_ROTOR_LOCK_CONFIRM_MS)
    {
        s_batt_fuse_latched = 1U;

        s_batt_lock_reason =
            BATT_LOCK_REASON_CRITICAL_CURRENT;

        VS_BatteryCutoffRotorLock = 1U;

        s_batt_fuse_reduction_percent_internal =
            VC_BATT_FUSE_REDUCTION_LATCH_PERCENT;

        VS_BattDampMonitor = 0U;

        VS_BattDampMonitor |=
            (uint16_t)(1U << 0);

        VS_BattDampMonitor |=
            (uint16_t)(1U << 4);

        VS_BattDampMonitor |=
            (uint16_t)(((uint16_t)s_batt_damp_state_internal &
                        0x03U) << 6);

        VS_BattDampMonitor |=
            (uint16_t)(((uint16_t)
                        s_batt_fuse_reduction_percent_internal &
                        0xFFU) << 8);

        s_b1_prev_current_raw =
            VS_Battery1.BatteryCurrent_raw;

        s_b2_prev_current_raw =
            VS_Battery2.BatteryCurrent_raw;

        s_batt_prev_pair_valid = both_valid;

        return;
    }

    /* =====================================================
     * 5. DIRECT BATTERY CUT-OFF DETECTION
     *
     * Requires TWO valid batteries so an intentionally absent
     * battery cannot be interpreted as a zero-current cut-off.
     * ===================================================== */
    b1_zero =
        (uint8_t)(((b1_valid != 0U) &&
                   (VS_Battery1.BatteryCurrent_raw <=
                    VC_BATT_FUSE_ZERO_CURRENT_RAW)) ? 1U : 0U);

    b2_zero =
        (uint8_t)(((b2_valid != 0U) &&
                   (VS_Battery2.BatteryCurrent_raw <=
                    VC_BATT_FUSE_ZERO_CURRENT_RAW)) ? 1U : 0U);

    b1_other_high =
        (uint8_t)(((b1_valid != 0U) &&
                   (VS_Battery1.BatteryCurrent_raw >=
                    VC_BATT_FUSE_OTHER_HIGH_RAW)) ? 1U : 0U);

    b2_other_high =
        (uint8_t)(((b2_valid != 0U) &&
                   (VS_Battery2.BatteryCurrent_raw >=
                    VC_BATT_FUSE_OTHER_HIGH_RAW)) ? 1U : 0U);

    if (both_valid != 0U)
    {
        if (((b1_zero != 0U) &&
             (b2_other_high != 0U)) ||
            ((b2_zero != 0U) &&
             (b1_other_high != 0U)))
        {
            direct_cutoff = 1U;
        }
    }

    if (direct_cutoff != 0U)
    {
        if (s_batt_cutoff_confirm_ms <
            VC_BATT_CUTOFF_CONFIRM_MS)
        {
            s_batt_cutoff_confirm_ms =
                (uint16_t)(s_batt_cutoff_confirm_ms +
                           MACHINE_DRIVE_TASK_PERIOD_MS);

            if (s_batt_cutoff_confirm_ms >
                VC_BATT_CUTOFF_CONFIRM_MS)
            {
                s_batt_cutoff_confirm_ms =
                    VC_BATT_CUTOFF_CONFIRM_MS;
            }
        }
    }
    else
    {
        s_batt_cutoff_confirm_ms = 0U;
    }

    /* =====================================================
     * 6. SECONDARY SLOPE DETECTOR
     *
     * Only compare slopes when both batteries are valid now
     * and were also valid during the previous sample.
     * ===================================================== */
    if ((both_valid != 0U) &&
        (s_batt_prev_pair_valid != 0U))
    {
        if (VS_Battery1.BatteryCurrent_raw >=
            s_b1_prev_current_raw)
        {
            b1_rise =
                (uint16_t)(VS_Battery1.BatteryCurrent_raw -
                           s_b1_prev_current_raw);
            b1_drop = 0U;
        }
        else
        {
            b1_rise = 0U;
            b1_drop =
                (uint16_t)(s_b1_prev_current_raw -
                           VS_Battery1.BatteryCurrent_raw);
        }

        if (VS_Battery2.BatteryCurrent_raw >=
            s_b2_prev_current_raw)
        {
            b2_rise =
                (uint16_t)(VS_Battery2.BatteryCurrent_raw -
                           s_b2_prev_current_raw);
            b2_drop = 0U;
        }
        else
        {
            b2_rise = 0U;
            b2_drop =
                (uint16_t)(s_b2_prev_current_raw -
                           VS_Battery2.BatteryCurrent_raw);
        }

        if (((b1_drop >= VC_BATT_FUSE_SLOPE_DROP_RAW) &&
             (b2_rise >= VC_BATT_FUSE_SLOPE_RISE_RAW) &&
             (b2_other_high != 0U)) ||
            ((b2_drop >= VC_BATT_FUSE_SLOPE_DROP_RAW) &&
             (b1_rise >= VC_BATT_FUSE_SLOPE_RISE_RAW) &&
             (b1_other_high != 0U)))
        {
            slope_condition = 1U;
        }
    }

    if (slope_condition != 0U)
    {
        if (s_batt_fuse_confirm_ms <
            VC_BATT_FUSE_CONFIRM_MS)
        {
            s_batt_fuse_confirm_ms =
                (uint16_t)(s_batt_fuse_confirm_ms +
                           MACHINE_DRIVE_TASK_PERIOD_MS);

            if (s_batt_fuse_confirm_ms >
                VC_BATT_FUSE_CONFIRM_MS)
            {
                s_batt_fuse_confirm_ms =
                    VC_BATT_FUSE_CONFIRM_MS;
            }
        }
    }
    else
    {
        s_batt_fuse_confirm_ms = 0U;
    }

    fuse_condition =
        (uint8_t)(((direct_cutoff != 0U) ||
                   (slope_condition != 0U)) ? 1U : 0U);

    /* =====================================================
     * 7. LATCH CUT-OFF / SLOPE FAULT
     * ===================================================== */
    if (s_batt_cutoff_confirm_ms >=
        VC_BATT_CUTOFF_CONFIRM_MS)
    {
        s_batt_fuse_latched = 1U;
        s_batt_lock_reason =
            BATT_LOCK_REASON_CUTOFF;
    }
    else if (s_batt_fuse_confirm_ms >=
             VC_BATT_FUSE_CONFIRM_MS)
    {
        s_batt_fuse_latched = 1U;
        s_batt_lock_reason =
            BATT_LOCK_REASON_SLOPE;
    }

    if (s_batt_fuse_latched != 0U)
    {
        VS_BatteryCutoffRotorLock = 1U;

        s_batt_fuse_reduction_percent_internal =
            VC_BATT_FUSE_REDUCTION_LATCH_PERCENT;

        VS_BattDampMonitor = 0U;

        if (high_current != 0U)
        {
            VS_BattDampMonitor |=
                (uint16_t)(1U << 0);
        }

        VS_BattDampMonitor |=
            (uint16_t)(1U << 3);

        VS_BattDampMonitor |=
            (uint16_t)(1U << 4);

        VS_BattDampMonitor |=
            (uint16_t)(((uint16_t)s_batt_damp_state_internal &
                        0x03U) << 6);

        VS_BattDampMonitor |=
            (uint16_t)(((uint16_t)
                        s_batt_fuse_reduction_percent_internal &
                        0xFFU) << 8);

        s_b1_prev_current_raw =
            VS_Battery1.BatteryCurrent_raw;

        s_b2_prev_current_raw =
            VS_Battery2.BatteryCurrent_raw;

        s_batt_prev_pair_valid = both_valid;

        return;
    }

    /* =====================================================
     * 8. EXISTING NORMAL DAMPING FSM
     * ===================================================== */
    switch (s_batt_damp_state_internal)
    {
        case BATT_DAMP_STATE_WAIT_ENTRY:
            s_batt_fuse_reduction_percent_internal = 0U;
            s_batt_high_reduction_hold_active = 0U;

            if (high_current != 0U)
            {
                if (s_batt_damp_entry_ms <
                    VC_BATT_DAMP_ENTRY_CONFIRM_MS)
                {
                    s_batt_damp_entry_ms =
                        (uint16_t)(s_batt_damp_entry_ms +
                                   MACHINE_DRIVE_TASK_PERIOD_MS);

                    if (s_batt_damp_entry_ms >
                        VC_BATT_DAMP_ENTRY_CONFIRM_MS)
                    {
                        s_batt_damp_entry_ms =
                            VC_BATT_DAMP_ENTRY_CONFIRM_MS;
                    }
                }

                if (s_batt_damp_entry_ms >=
                    VC_BATT_DAMP_ENTRY_CONFIRM_MS)
                {
                    s_batt_damp_entry_ms = 0U;
                    s_batt_damp_active_ms = 0U;
                    s_batt_damp_state_internal =
                        BATT_DAMP_STATE_ACTIVE;

                    s_batt_fuse_reduction_percent_internal =
                        Get_Battery_Damping_Reduction(max_current);

                    if (s_batt_fuse_reduction_percent_internal ==
                        VC_BATT_FUSE_REDUCTION_HIGH_PERCENT)
                    {
                        s_batt_high_reduction_hold_active = 1U;
                    }
                }
            }
            else
            {
                s_batt_damp_entry_ms = 0U;
            }
            break;

        case BATT_DAMP_STATE_ACTIVE:
            /*
             * Re-evaluate the normal damping region exactly as before.
             *
             * Special handling only for the HIGH region:
             * once 46% is reached during this ACTIVE window, retain 46%
             * until the existing VC_BATT_DAMP_ACTIVE_MS period finishes.
             * This prevents self-relaxation caused by the reduction itself
             * lowering battery current below the HIGH threshold.
             */
            current_reduction =
                Get_Battery_Damping_Reduction(max_current);

            if ((s_batt_high_reduction_hold_active != 0U) ||
                (current_reduction ==
                 VC_BATT_FUSE_REDUCTION_HIGH_PERCENT))
            {
                s_batt_high_reduction_hold_active = 1U;
                s_batt_fuse_reduction_percent_internal =
                    VC_BATT_FUSE_REDUCTION_HIGH_PERCENT;
            }
            else
            {
                /*
                 * 10%, 15% and 35% retain their original behavior.
                 */
                s_batt_fuse_reduction_percent_internal =
                    current_reduction;
            }

            if (s_batt_damp_active_ms <
                VC_BATT_DAMP_ACTIVE_MS)
            {
                s_batt_damp_active_ms =
                    (uint16_t)(s_batt_damp_active_ms +
                               MACHINE_DRIVE_TASK_PERIOD_MS);

                if (s_batt_damp_active_ms >
                    VC_BATT_DAMP_ACTIVE_MS)
                {
                    s_batt_damp_active_ms =
                        VC_BATT_DAMP_ACTIVE_MS;
                }
            }

            if (s_batt_damp_active_ms >=
                VC_BATT_DAMP_ACTIVE_MS)
            {
                s_batt_damp_active_ms = 0U;
                s_batt_damp_reentry_ms = 0U;
                s_batt_damp_state_internal =
                    BATT_DAMP_STATE_REENTRY_WAIT;
                s_batt_fuse_reduction_percent_internal = 0U;
                s_batt_high_reduction_hold_active = 0U;
            }
            break;

        case BATT_DAMP_STATE_REENTRY_WAIT:
        default:
            s_batt_fuse_reduction_percent_internal = 0U;
            s_batt_high_reduction_hold_active = 0U;

            if (s_batt_damp_reentry_ms <
                VC_BATT_DAMP_REENTRY_DELAY_MS)
            {
                s_batt_damp_reentry_ms =
                    (uint16_t)(s_batt_damp_reentry_ms +
                               MACHINE_DRIVE_TASK_PERIOD_MS);

                if (s_batt_damp_reentry_ms >
                    VC_BATT_DAMP_REENTRY_DELAY_MS)
                {
                    s_batt_damp_reentry_ms =
                        VC_BATT_DAMP_REENTRY_DELAY_MS;
                }
            }

            if (s_batt_damp_reentry_ms >=
                VC_BATT_DAMP_REENTRY_DELAY_MS)
            {
                s_batt_damp_reentry_ms = 0U;

                if (high_current != 0U)
                {
                    s_batt_damp_active_ms = 0U;
                    s_batt_damp_state_internal =
                        BATT_DAMP_STATE_ACTIVE;

                    s_batt_fuse_reduction_percent_internal =
                        Get_Battery_Damping_Reduction(max_current);

                    if (s_batt_fuse_reduction_percent_internal ==
                        VC_BATT_FUSE_REDUCTION_HIGH_PERCENT)
                    {
                        s_batt_high_reduction_hold_active = 1U;
                    }
                }
                else
                {
                    s_batt_damp_entry_ms = 0U;
                    s_batt_damp_state_internal =
                        BATT_DAMP_STATE_WAIT_ENTRY;
                }
            }
            break;
    }

    /* =====================================================
     * 9. MONITOR
     * ===================================================== */
    VS_BattDampMonitor = 0U;

    if (high_current != 0U)
    {
        VS_BattDampMonitor |=
            (uint16_t)(1U << 0);
    }

    if (s_batt_damp_state_internal ==
        BATT_DAMP_STATE_ACTIVE)
    {
        VS_BattDampMonitor |=
            (uint16_t)(1U << 1);
    }

    if (s_batt_damp_state_internal ==
        BATT_DAMP_STATE_REENTRY_WAIT)
    {
        VS_BattDampMonitor |=
            (uint16_t)(1U << 2);
    }

    if (fuse_condition != 0U)
    {
        VS_BattDampMonitor |=
            (uint16_t)(1U << 3);
    }

    if (s_batt_fuse_latched != 0U)
    {
        VS_BattDampMonitor |=
            (uint16_t)(1U << 4);
    }

    if (battery_invalid != 0U)
    {
        VS_BattDampMonitor |=
            (uint16_t)(1U << 5);
    }

    VS_BattDampMonitor |=
        (uint16_t)(((uint16_t)s_batt_damp_state_internal &
                    0x03U) << 6);

    VS_BattDampMonitor |=
        (uint16_t)(((uint16_t)
                    s_batt_fuse_reduction_percent_internal &
                    0xFFU) << 8);

    /* =====================================================
     * 10. SAVE CURRENT FOR NEXT SLOPE SAMPLE
     * ===================================================== */
    s_b1_prev_current_raw =
        VS_Battery1.BatteryCurrent_raw;

    s_b2_prev_current_raw =
        VS_Battery2.BatteryCurrent_raw;

    s_batt_prev_pair_valid = both_valid;

#endif
}

/* ===================================================== */
/*           LOW-SOC ROTOR LOCK + DIAGNOSTICS            */
/* ===================================================== */
static void MachineDrive_UpdateSocRotorLock(void)
{
#if VC_PSM_SOC_ENABLE
    uint8_t soc1 = (uint8_t)VS_Battery1.SOC;
    uint8_t soc2 = (uint8_t)VS_Battery2.SOC;
    uint8_t b1_soc_valid;
    uint8_t b2_soc_valid;
    uint8_t valid_mask = 0U;
    uint8_t low_soc_cause = ROTOR_LOCK_CAUSE_NONE;
    uint8_t clear_soc_condition = 0U;
    uint16_t average_soc = 0U;

    b1_soc_valid =
        (uint8_t)(((VS_Battery1.Alive != 0U) &&
                   (VS_Battery1.BatteryVoltage_raw >
                    VC_BATT_VALID_VOLTAGE_RAW) &&
                   (soc1 <= 100U)) ? 1U : 0U);

    b2_soc_valid =
        (uint8_t)(((VS_Battery2.Alive != 0U) &&
                   (VS_Battery2.BatteryVoltage_raw >
                    VC_BATT_VALID_VOLTAGE_RAW) &&
                   (soc2 <= 100U)) ? 1U : 0U);

    if (b1_soc_valid != 0U)
    {
        valid_mask |= 0x01U;

        if (soc1 <= VC_PSM_SOC_TRIP_PERCENT)
        {
            low_soc_cause |= ROTOR_LOCK_CAUSE_SOC_BATTERY1;
        }
    }

    if (b2_soc_valid != 0U)
    {
        valid_mask |= 0x02U;

        if (soc2 <= VC_PSM_SOC_TRIP_PERCENT)
        {
            low_soc_cause |= ROTOR_LOCK_CAUSE_SOC_BATTERY2;
        }
    }

    if (valid_mask == 0x03U )
    {
        if(soc1 > soc2)
        {
            average_soc = (uint16_t)(soc1-soc2);
        }
        else
        {
            average_soc = (uint16_t)(soc2-soc1);
        }

        if (average_soc >= VC_PSM_AVG_SOC_DIFF)
        {
            low_soc_cause |= ROTOR_LOCK_CAUSE_SOC_AVERAGE;
        }
    }

    if (valid_mask != 0U)
    {
        s_psm_soc_data_seen = 1U;
    }

    if (VS_PsmRotorLock == 0U)
    {
        if ((valid_mask != 0U) &&
            (low_soc_cause != ROTOR_LOCK_CAUSE_NONE))
        {
            /* Saturating addition avoids a uint16_t timer wraparound. */
            if (VS_PsmSocConfirmTimer_ms <
                VC_PSM_SOC_CONFIRM_TIME_MS)
            {
                if ((uint32_t)VS_PsmSocConfirmTimer_ms +
                    (uint32_t)MACHINE_DRIVE_TASK_PERIOD_MS >=
                    (uint32_t)VC_PSM_SOC_CONFIRM_TIME_MS)
                {
                    VS_PsmSocConfirmTimer_ms =
                        VC_PSM_SOC_CONFIRM_TIME_MS;
                }
                else
                {
                    VS_PsmSocConfirmTimer_ms =
                        (uint16_t)(VS_PsmSocConfirmTimer_ms +
                                   MACHINE_DRIVE_TASK_PERIOD_MS);
                }
            }

            if (VS_PsmSocConfirmTimer_ms >=
                VC_PSM_SOC_CONFIRM_TIME_MS)
            {
                VS_PsmRotorLock = 1U;
                s_psm_soc_lock_cause = low_soc_cause;
                s_psm_soc_lock_valid_mask = valid_mask;
                VS_PsmSocConfirmTimer_ms = 0U;
            }
        }
        else
        {
            VS_PsmSocConfirmTimer_ms = 0U;
            s_psm_soc_lock_cause = ROTOR_LOCK_CAUSE_NONE;
            s_psm_soc_lock_valid_mask = 0U;
        }
    }
    else
    {
        /*
         * A locked source must remain valid and reach the clear threshold.
         * This keeps the lock sticky through missing/invalid CAN data and
         * prevents the healthy battery from clearing a lock caused by the
         * other battery disappearing from the bus.
         */
        clear_soc_condition = 1U;

        if ((s_psm_soc_lock_valid_mask & 0x01U) != 0U)
        {
            if ((b1_soc_valid == 0U) ||
                (soc1 < VC_PSM_SOC_CLEAR_PERCENT))
            {
                clear_soc_condition = 0U;
            }
        }

        if ((s_psm_soc_lock_valid_mask & 0x02U) != 0U)
        {
            if ((b2_soc_valid == 0U) ||
                (soc2 < VC_PSM_SOC_CLEAR_PERCENT))
            {
                clear_soc_condition = 0U;
            }
        }

        if (s_psm_soc_lock_valid_mask == 0U)
        {
            clear_soc_condition = 0U;
        }

        if (clear_soc_condition != 0U)
        {
            if (VS_PsmSocConfirmTimer_ms <
                VC_PSM_SOC_CONFIRM_TIME_MS)
            {
                if ((uint32_t)VS_PsmSocConfirmTimer_ms +
                    (uint32_t)MACHINE_DRIVE_TASK_PERIOD_MS >=
                    (uint32_t)VC_PSM_SOC_CONFIRM_TIME_MS)
                {
                    VS_PsmSocConfirmTimer_ms =
                        VC_PSM_SOC_CONFIRM_TIME_MS;
                }
                else
                {
                    VS_PsmSocConfirmTimer_ms =
                        (uint16_t)(VS_PsmSocConfirmTimer_ms +
                                   MACHINE_DRIVE_TASK_PERIOD_MS);
                }
            }

            if (VS_PsmSocConfirmTimer_ms >=
                VC_PSM_SOC_CONFIRM_TIME_MS)
            {
                VS_PsmRotorLock = 0U;
                VS_PsmSocConfirmTimer_ms = 0U;
                s_psm_soc_lock_cause = ROTOR_LOCK_CAUSE_NONE;
                s_psm_soc_lock_valid_mask = 0U;
            }
        }
        else
        {
            VS_PsmSocConfirmTimer_ms = 0U;
        }
    }

    if (VS_PsmRotorLock != 0U)
    {
        s_psm_soc_immediate_inhibit = 1U;
        s_psm_soc_immediate_cause = s_psm_soc_lock_cause;
    }
    else if (low_soc_cause != ROTOR_LOCK_CAUSE_NONE)
    {
        s_psm_soc_immediate_inhibit = 1U;
        s_psm_soc_immediate_cause = low_soc_cause;
    }
    else if (s_psm_soc_data_seen == 0U)
    {
        s_psm_soc_immediate_inhibit = 1U;
        s_psm_soc_immediate_cause = ROTOR_LOCK_CAUSE_SOC_DATA_WAIT;
    }
    else
    {
        s_psm_soc_immediate_inhibit = 0U;
        s_psm_soc_immediate_cause = ROTOR_LOCK_CAUSE_NONE;
    }
#else
    /* Feature-disabled builds must never report a stale SOC lock cause. */
    s_psm_soc_lock_cause = ROTOR_LOCK_CAUSE_NONE;
    s_psm_soc_lock_valid_mask = 0U;
    s_psm_soc_immediate_inhibit = 0U;
    s_psm_soc_immediate_cause = ROTOR_LOCK_CAUSE_NONE;
    s_psm_soc_data_seen = 0U;
#endif
}

static void MachineDrive_UpdateRotorLockDiagnostic(void)
{
    uint8_t cause = ROTOR_LOCK_CAUSE_NONE;

    if (s_batt_fuse_latched != 0U)
    {
        if (s_batt_lock_reason == BATT_LOCK_REASON_CRITICAL_CURRENT)
        {
            cause |= ROTOR_LOCK_CAUSE_CRITICAL_CURRENT;
        }
        else if (s_batt_lock_reason == BATT_LOCK_REASON_CUTOFF)
        {
            cause |= ROTOR_LOCK_CAUSE_BATTERY_CUTOFF;
        }
        else if (s_batt_lock_reason == BATT_LOCK_REASON_SLOPE)
        {
            cause |= ROTOR_LOCK_CAUSE_BATTERY_SLOPE;
        }
        else
        {
            /* A latched protection with no decoded reason is still active. */
            cause |= ROTOR_LOCK_CAUSE_BATTERY_CUTOFF;
        }
    }

#if VC_PSM_SOC_ENABLE
    if ((VC_ROTOR_BATTERY_STOP_BYPASS == 0U) &&
        (s_psm_soc_immediate_inhibit != 0U))
    {
        cause |= s_psm_soc_immediate_cause;
    }
#endif

    VS_RotorLockCause = cause;
    VS_RotorLockActive =
        (uint8_t)((cause != ROTOR_LOCK_CAUSE_NONE) ? 1U : 0U);

    /* Capture a fresh snapshot only at the start/change of a lock event. */
    if ((cause != ROTOR_LOCK_CAUSE_NONE) &&
        (cause != s_rotor_lock_previous_cause))
    {
        VS_RotorLockLastCause = cause;
        VS_RotorLockTripSoc1 = (uint8_t)VS_Battery1.SOC;
        VS_RotorLockTripSoc2 = (uint8_t)VS_Battery2.SOC;
        VS_RotorLockTripBattery1Current_raw =
            VS_Battery1.BatteryCurrent_raw;
        VS_RotorLockTripBattery2Current_raw =
            VS_Battery2.BatteryCurrent_raw;
    }

    s_rotor_lock_previous_cause = cause;
}

/* --- Rotor direction FSM state --- */
#define ROT_DIR_STATE_IDLE      (0U)
#define ROT_DIR_STATE_RAMP_DOWN (1U)
#define ROT_DIR_STATE_NEUTRAL   (2U)
#define ROT_DIR_STATE_RAMP_UP   (3U)

/* ===================================================== */
/*                  MACHINE DRIVE MODE STEP              */
/* ===================================================== */
void MachineDrive_Mode_Step(void)
{
    uint16_t base_rpm;
    uint16_t rotor_rpm;
    uint8_t  requested_dir;
    uint16_t target_left_rpm;
    uint16_t target_right_rpm;
    uint16_t diff;
    uint16_t base;
    uint16_t slew;
    uint8_t  turn_now;
    uint8_t  direction_transition_active;
    uint8_t  direction_matches_request;

    /* =====================================================
     * 1. Battery Fuse Save Check
    * ===================================================== */
    base_rpm = Throttle_To_RPM(VS_Throttle_Wheel);
    MachineDrive_BatteryFuseSaveStep();
    MachineDrive_UpdateSocRotorLock();
    MachineDrive_UpdateRotorLockDiagnostic();

#if VC_PSM_SOC_ENABLE
    if ((VC_ROTOR_BATTERY_STOP_BYPASS == 0U) &&
        (s_psm_soc_immediate_inhibit != 0U))
    {
        VS_RPM_Rotor = 0U;
    }
#endif

    /* Detect KillSwitch rising edge */
    if ((VS_KillSwitchPrev == 0U) && (VS_KillSwitch == 1U))
    {
        VS_KillRecoveryActive = 1U;
    }
    VS_KillSwitchPrev = VS_KillSwitch;

    /* ---------- KillSwitch ACTIVE (OFF = emergency stop) ---------- */
    if (VS_KillSwitch == 0U)
    {

        target_left_rpm  = 0U;
        target_right_rpm = 0U;

        VS_RPM_Left  = Apply_RPM_Slew(0U, VS_RPM_Left,  VC_RPM_SLEW_RATE_EXIT);
        VS_RPM_Right = Apply_RPM_Slew(0U, VS_RPM_Right, VC_RPM_SLEW_RATE_EXIT);
        VS_DualButtonDriveActive = 0U;
        /* Stop all actuators */
        base_rpm = 0U;
        VS_RPM_Left  = 0U;
        VS_RPM_Right = 0U;
#if !VC_ROTOR_LEVER_BYPASS
        VS_RPM_Rotor = 0U;
#endif

        VS_RPM_Wheel_Current = 0U;
  
        /* Reset the high-end direction/pickup synchronizer. */
        WheelDirectionSync_Reset();
        VS_DirHoldCounter = 0U;

        /* ---------- HARD RESET TURN MEMORY ---------- */
        VS_TurnIntensity    = 0U;
        VS_TurnHoldTime_ms  = 0U;
        VS_TurnGradDiff_RPM = 0U;
        VS_TurnEntryTime_ms = 0U;
        VS_TurnPrev         = 0U;
        s_turn_override_armed = 0U;
        
        /* ---------- Prevent aggressive recovery ---------- */
        VS_KillRecoveryActive = 0U;
#if VC_ROTOR_LEVER_BYPASS
        /* Keep the existing dual-button stop even during the lever bypass. */
        if ((VS_TurnLeft != 0U) && (VS_TurnRight != 0U))
        {
            VS_RPM_Rotor = 0U;
            return;
        }
        goto ROTOR_CONTROL;
#else
        return;
#endif
    }

    VS_DualButtonDriveActive = 0U;
    //VS_DualButtonActive_300RPM = 0U;

    /* =====================================================
     * 2. TURN-PRIORITY GATE + HIGH-END DIRECTION SYNC
     *
     * IMPORTANT OWNERSHIP RULE:
     *
     *   A normal LEFT/RIGHT turn intentionally creates different wheel
     *   commands and therefore different wheel feedback.  The feedback-
     *   based direction synchronizer must NOT interpret that intentional
     *   mismatch as a synchronization fault and overwrite steering.
     *
     * Priority:
     *   1. Kill switch / hard safety        (handled above)
     *   2. Dual-turn stop                   (immediate stop)
     *   3. Real direction transition/pickup (feedback sync owns wheels)
     *   4. Stable-direction single turn     (turn logic owns wheels)
     *   5. Straight drive                   (feedback sync owns base drive)
     *
     * When a single turn is requested while the commanded direction is
     * already stable and the sync FSM is IDLE, we intentionally DO NOT
     * call WheelDirectionSync_Step().  This prevents the synchronizer from
     * seeing the turn-induced L/R RPM difference on the next cycle and
     * cancelling the turn.
     *
     * If a real direction change is requested, or a transition/pickup is
     * already active, the synchronizer keeps higher priority.
     * ===================================================== */

    turn_now =
        (uint8_t)(((VS_TurnLeft != 0U) ||
                   (VS_TurnRight != 0U)) ? 1U : 0U);

    /* -----------------------------------------------------
     * DUAL TURN STOP: handle BEFORE feedback synchronization.
     * This makes both-button stop deterministic and prevents a sync step
     * from rewriting the wheel command before the stop is applied.
     * ----------------------------------------------------- */
    if ((VS_TurnLeft != 0U) && (VS_TurnRight != 0U))
    {
        VS_RPM_Left  = Apply_RPM_Slew(0U,
                                      VS_RPM_Left,
                                      VC_RPM_SLEW_RATE_EXIT);
        VS_RPM_Right = Apply_RPM_Slew(0U,
                                      VS_RPM_Right,
                                      VC_RPM_SLEW_RATE_EXIT);

        VS_RPM_Rotor = 0U;

        VS_TurnIntensity    = 0U;
        VS_TurnHoldTime_ms  = 0U;
        VS_TurnGradDiff_RPM = 0U;
        VS_TurnEntryTime_ms = 0U;
        VS_TurnPrev         = 0U;

        return;
    }

    requested_dir =
        (VS_Direction_Request == 0U) ? DIR_FORWARD : DIR_REVERSE;

    base_rpm = Throttle_To_RPM(VS_Throttle_Wheel);

    if (base_rpm < VC_THROTTLE_DEAD_ZONE)
    {
        base_rpm = 0U;
    }

    direction_matches_request =
        (uint8_t)((VS_Direction == requested_dir) ? 1U : 0U);

    /*
     * A genuine direction request change always disarms turn override.
     * The feedback synchronizer must complete the direction transition before
     * steering is allowed to bypass symmetry enforcement again.
     */
    if (direction_matches_request == 0U)
    {
        s_turn_override_armed = 0U;
    }

    if ((turn_now != 0U) &&
        (direction_matches_request != 0U) &&
        (s_turn_override_armed != 0U))
    {
        /*
         * STABLE-DIRECTION TURN OWNERSHIP
         *
         * Do not call WheelDirectionSync_Step() while the turn is active.
         * The L/R RPM difference is intentional steering, not a sync fault.
         * Keep only the common throttle/base slew alive; the turn block below
         * owns the individual left/right wheel commands.
         */
        VS_RPM_Wheel_Current =
            Apply_RPM_Slew(base_rpm,
                           VS_RPM_Wheel_Current,
                           VC_RPM_SLEW_RATE);

        direction_transition_active = 0U;
    }
    else
    {
        /*
         * Startup, straight driving, safety pickup, or a genuine direction
         * change: retain the complete feedback-based jerk-removal logic.
         */
        WheelDirectionSync_Step(base_rpm, requested_dir);

        direction_transition_active =
            WheelDirectionSync_IsTransitionActive();

        /*
         * Arm steering override only after the synchronizer has genuinely
         * reached IDLE with the requested direction applied.  This prevents
         * a turn from bypassing the zero/neutral/direction-ack pickup sequence
         * immediately after kill-switch recovery or a direction change.
         */
        direction_matches_request =
            (uint8_t)((VS_Direction == requested_dir) ? 1U : 0U);

        if ((direction_transition_active == 0U) &&
            (direction_matches_request != 0U))
        {
            s_turn_override_armed = 1U;
        }
    }

    /* The synchronizer handles safety-lever pickup smoothly. */
    VS_KillRecoveryActive = 0U;


{
    /* =====================================================
     * 6. TURN LOGIC (LEFT / RIGHT) — JERK-FREE
     *
     * Sharp-turn behavior intentionally follows the previously
     * proven implementation:
     *   - below VC_TURN_GRADIENT_RPM_LIMIT (600 RPM in config),
     *     VS_TurnHoldTime_ms accumulates while one turn button
     *     remains continuously pressed;
     *   - once VS_TurnHoldTime_ms reaches
     *     VC_TURN_INNER_ZERO_HOLD_MS (2000 ms in config),
     *     the inner wheel target is forced to 0 RPM;
     *   - the outer wheel remains at the existing common wheel
     *     command `base`;
     *   - direction-transition synchronization keeps higher
     *     priority and clears turn memory until transition IDLE.
     * ===================================================== */
    if ((turn_now == 1U) && (VS_TurnPrev == 0U))
    {
        VS_TurnEntryTime_ms = 0U;
    }

    VS_TurnPrev = turn_now;

    /* Keep the existing normal-turn base exactly as before. */
    base = VS_RPM_Wheel_Current;

    if (direction_transition_active != 0U)
    {
        /*
         * Direction transition/pickup must remain perfectly symmetric.
         * Ignore turn requests and clear all turn memory until IDLE.
         */
        VS_TurnHoldTime_ms   = 0U;
        VS_TurnGradDiff_RPM  = 0U;
        VS_TurnEntryTime_ms  = 0U;
        VS_TurnIntensity     = 0U;
        VS_TurnPrev          = 0U;

        target_left_rpm  = base;
        target_right_rpm = base;
    }
    else if (!VS_TurnLeft && !VS_TurnRight)
    {
        VS_TurnHoldTime_ms   = 0U;
        VS_TurnGradDiff_RPM  = 0U;
        VS_TurnEntryTime_ms  = 0U;
        VS_TurnIntensity     = 0U;

        target_left_rpm  = base;
        target_right_rpm = base;
    }
    else
    {
        diff =
            (uint16_t)(((uint32_t)base *
                        (uint32_t)VC_TURN_DIFF_RATIO_NUM) /
                       (uint32_t)VC_TURN_DIFF_RATIO_DEN);

        if (base < VC_TURN_LOW_RPM_LIMIT)
        {
            if (diff < VC_TURN_SOFT_DIFF_MIN)
            {
                diff = VC_TURN_SOFT_DIFF_MIN;
            }
        }
        else
        {
            if (diff <= VC_TURN_DIFF_MIN_RPM)
            {
                diff = VC_TURN_DIFF_MIN_RPM;
            }
        }

        if (diff > VC_TURN_DIFF_MAX_RPM)
        {
            diff = VC_TURN_DIFF_MAX_RPM;
        }

#if VC_TURN_GRADIENT_ENABLE
        /*
         * IMPORTANT:
         * This is the same hold timer used by the previously working
         * sharp-turn implementation.  Below 600 RPM it accumulates
         * continuously for as long as one turn button is held.
         */
        if (base < VC_TURN_GRADIENT_RPM_LIMIT)
        {
            if (VS_TurnHoldTime_ms < 40000U)
            {
                VS_TurnHoldTime_ms += MACHINE_DRIVE_TASK_PERIOD_MS;
            }

            if (VS_TurnHoldTime_ms >= VC_TURN_GRADIENT_DELAY_MS)
            {
                if ((VS_TurnHoldTime_ms %
                     VC_TURN_GRADIENT_STEP_MS) == 0U)
                {
                    if (VS_TurnGradDiff_RPM <
                        VC_TURN_GRADIENT_MAX_DIFF)
                    {
                        VS_TurnGradDiff_RPM +=
                            VC_TURN_GRADIENT_STEP_RPM;

                        if (VS_TurnGradDiff_RPM >
                            VC_TURN_GRADIENT_MAX_DIFF)
                        {
                            VS_TurnGradDiff_RPM =
                                VC_TURN_GRADIENT_MAX_DIFF;
                        }
                    }
                }
            }
        }
        else
        {
            /* At/above the low-speed limit, sharp-turn timing resets. */
            VS_TurnHoldTime_ms  = 0U;
            VS_TurnGradDiff_RPM = 0U;

        }
#else
        VS_TurnHoldTime_ms  = 0U;
        VS_TurnGradDiff_RPM = 0U;
#endif

        diff = (uint16_t)(diff + VS_TurnGradDiff_RPM);

        if (diff > VC_TURN_DIFF_MAX_RPM)
        {
            diff = VC_TURN_DIFF_MAX_RPM;
        }

#if VC_TURN_ENTRY_SOFTEN_ENABLE
        if ((base >= VC_TURN_ENTRY_RPM_START) &&
            (VS_TurnEntryTime_ms < VC_TURN_ENTRY_TIME_MS))
        {
            uint16_t entry_limit;

            VS_TurnEntryTime_ms += MACHINE_DRIVE_TASK_PERIOD_MS;

            entry_limit =
                (uint16_t)(((uint32_t)diff *
                            (uint32_t)VC_TURN_ENTRY_MAX_RATIO_NUM) /
                           (uint32_t)VC_TURN_ENTRY_MAX_RATIO_DEN);

            if (diff > entry_limit)
            {
                diff = entry_limit;
            }
        }
#endif

        VS_TurnIntensity =
            Apply_RPM_Slew(diff,
                           VS_TurnIntensity,
                           VC_TURN_INTENSITY_SLEW);

        /* ---------------- LEFT TURN ---------------- */
        if (VS_TurnLeft)
        {
            if (VS_TurnHoldTime_ms >= VC_TURN_INNER_ZERO_HOLD_MS)
            {
                /*
                 * Proven sharp-turn behavior:
                 * after 2 s below 600 RPM, LEFT inner wheel -> 0 RPM.
                 */
                target_left_rpm = 0U;
            }
            else
            {
                target_left_rpm =
                    (base > VS_TurnIntensity) ?
                    (uint16_t)(base - VS_TurnIntensity) :
                    VC_TURN_INNER_MIN_RPM;
            }

            /* Outer wheel remains at the existing throttle/common RPM. */
            target_right_rpm = base;
        }
        /* ---------------- RIGHT TURN ---------------- */
        else
        {
            if (VS_TurnHoldTime_ms >= VC_TURN_INNER_ZERO_HOLD_MS)
            {
                /*
                 * Proven sharp-turn behavior:
                 * after 2 s below 600 RPM, RIGHT inner wheel -> 0 RPM.
                 */
                target_right_rpm = 0U;
            }
            else
            {
                target_right_rpm =
                    (base > VS_TurnIntensity) ?
                    (uint16_t)(base - VS_TurnIntensity) :
                    VC_TURN_INNER_MIN_RPM;
            }

            /* Outer wheel remains at the existing throttle/common RPM. */
            target_left_rpm = base;
        }
    }

    if (target_left_rpm > VC_WHEEL_MAX_RPM)
    {
        target_left_rpm = VC_WHEEL_MAX_RPM;
    }

    if (target_right_rpm > VC_WHEEL_MAX_RPM)
    {
        target_right_rpm = VC_WHEEL_MAX_RPM;
    }
}


#if VC_BATT_FUSE_SAVE_ENABLE
    /*
     * Apply battery fuse-save reduction only to final wheel targets.
     * Do not change base, VS_RPM_Wheel_Current, diff, or VS_TurnIntensity.
     * This keeps the previous steering/handling sequence intact.
     *
     * Important for turning:
     * Use each wheel's own original target for minimum-RPM clamp.
     * Do not use common VS_RPM_Wheel_Current here, otherwise the inner
     * wheel during a turn can be increased back to VC_BATT_FUSE_MIN_WHEEL_RPM
     * and turning will become weak.
     */
    if (s_batt_fuse_reduction_percent_internal != 0U)
    {
        target_left_rpm =
            Reduce_RPM_By_Percent(target_left_rpm,
                                  s_batt_fuse_reduction_percent_internal);

        target_right_rpm =
            Reduce_RPM_By_Percent(target_right_rpm,
                                  s_batt_fuse_reduction_percent_internal);

        /*
         * Do not apply a minimum wheel-RPM floor while damping.  A floor
         * changes the requested percentage whenever the calculated target
         * is below that floor; for example, a 46% reduction of 800 RPM is
         * 432 RPM, not 450 RPM.  Every configured damping level must retain
         * its exact percentage.
         */
    }
#endif

    slew =
        (base < VC_TURN_LOW_RPM_LIMIT) ?
        VC_RPM_SLEW_RATE_TURN_OUTER :
        VC_RPM_SLEW_RATE_TURN_INNER;

    /* =====================================================
     * 6A. FINAL WHEEL RPM OUTPUT
     * ===================================================== */
    if ((direction_transition_active != 0U) ||
        (s_batt_fuse_reduction_percent_internal != 0U))
    {
        /*
         * Never allow independent slew to reintroduce a previous left/right
         * mismatch during zero confirmation, direction apply, or pickup.
         *
         * An active battery damping level is a protection command.  Send
         * its calculated target directly; passing it through the normal
         * slew would leave the output at an intermediate RPM and therefore
         * apply less than the selected 46/50/55/60/80 percent reduction.
         */
        VS_RPM_Left  = target_left_rpm;
        VS_RPM_Right = target_right_rpm;
    }
    else
    {
        VS_RPM_Right =
            Apply_RPM_Slew(target_right_rpm,
                           VS_RPM_Right,
                           slew);
        VS_RPM_Left =
            Apply_RPM_Slew(target_left_rpm,
                           VS_RPM_Left,
                           slew);
    }

#if VC_ROTOR_LEVER_BYPASS
ROTOR_CONTROL:
#endif
    /* =====================================================
    * 7. ROTOR LOGIC (UPDATED: preset RPM + analog mode + 2-GPIO direction + SOC PSM + dir-change gradient)
    * ===================================================== */
   
    {
        uint8_t mode = 0xFFU;
        uint8_t mode_valid =
            Select_Rotor_Mode((uint16_t)VS_Throttle_Rotor, &mode);

        /* Direction inputs (keep same as your current wiring) */
        uint8_t gpio_fwd = (uint8_t)(VS_GPIO8 ? 1U : 0U);        /* GPIO8 high -> FWD d3 */
        uint8_t gpio_rev = (uint8_t)(VS_RotorEnable ? 1U : 0U);  /* VS_RotorEnable high -> REV f2 */

        /* =====================================================
         * Rotor direction FSM (local to this block)
         * - FWD<->REV: ramp down -> neutral hold -> ramp up
         * - STOP (both low OR both high): IMMEDIATE STOP
         * ===================================================== */
        static uint8_t  VS_RotDirState         = ROT_DIR_STATE_IDLE;
        static uint8_t  VS_RotCurrentDirection = DIR_STOP;
        static uint8_t  VS_RotRequestedDir     = DIR_STOP;
        static uint16_t VS_RotNeutralTimer_ms  = 0U;

        /*
         * Fuse/cutoff/critical-current latch has highest rotor priority.
         * The existing 80% latch still stops the rotor completely.
         */
        if ((VC_ROTOR_BATTERY_STOP_BYPASS == 0U) &&
            (VS_BatteryCutoffRotorLock != 0U))
        {
            VS_RPM_Rotor            = 0U;
            VS_RotDirState          = ROT_DIR_STATE_IDLE;
            VS_RotNeutralTimer_ms   = 0U;
            VS_RotCurrentDirection  = DIR_STOP;
            VS_RotorDirection       = DIR_STOP;
            goto ROTOR_DONE;
        }

        /*
         * Keep the existing shared-state meaning: VS_Rotormodestate
         * contains the user's analog-selected mode (0..3).  Normal current
         * damping does not modify this rotor selection.
         */
        VS_Rotormodestate = mode;

        /* Decode requested direction from TWO GPIOs */
        if ((gpio_fwd == 1U) && (gpio_rev == 0U))
        {
            VS_RotRequestedDir = DIR_FORWARD;
        }
        else if ((gpio_fwd == 0U) && (gpio_rev == 1U))
        {
            VS_RotRequestedDir = DIR_REVERSE;
        }
        else
        {
            /* both low OR both high -> STOP (safe) */
            VS_RotRequestedDir = DIR_STOP;
        }

        /* IMMEDIATE STOP requirement (no gradient) */
        if (VS_RotRequestedDir == DIR_STOP)
        {
            VS_RPM_Rotor            = 0U;
            VS_RotDirState          = ROT_DIR_STATE_IDLE;
            VS_RotNeutralTimer_ms   = 0U;
            VS_RotCurrentDirection  = DIR_STOP;
            VS_RotorDirection       = DIR_STOP;
            goto ROTOR_DONE;
        }

        #if VC_PSM_SOC_ENABLE
                if ((VC_ROTOR_BATTERY_STOP_BYPASS == 0U) &&
                    ((s_psm_soc_immediate_inhibit != 0U) ||
                     (VS_PsmRotorLock != 0U)))
                {
                    VS_RPM_Rotor            = 0U;
                    VS_RotDirState          = ROT_DIR_STATE_IDLE;
                    VS_RotNeutralTimer_ms   = 0U;
                    VS_RotCurrentDirection  = DIR_STOP;
                    VS_RotorDirection       = DIR_STOP;
                    goto ROTOR_DONE;
                }
        #endif

        if (mode_valid != 0U)
        {
            switch (mode)
            {
                case 0U: rotor_rpm = VC_ROTOR_MODE1_RPM; break;
                case 1U: rotor_rpm = VC_ROTOR_MODE2_RPM; break;
                case 2U: rotor_rpm = VC_ROTOR_MODE3_RPM; break;
                case 3U: rotor_rpm = VC_ROTOR_MODE4_RPM; break;
                default: rotor_rpm = 0U;                  break;
            }
        }
        else
        {
            /* Invalid analog value -> safe RPM 0 */
            rotor_rpm = 0U;
        }

        /* Start direction change only when FSM is IDLE */
        if ((VS_RotDirState == ROT_DIR_STATE_IDLE) &&
            (VS_RotRequestedDir != VS_RotCurrentDirection))
        {
            VS_RotDirState = ROT_DIR_STATE_RAMP_DOWN;
        }

        /* Direction FSM (exclusive owner of VS_RPM_Rotor while active) */
        if (VS_RotDirState != ROT_DIR_STATE_IDLE)
        {
            switch (VS_RotDirState)
            {
                case ROT_DIR_STATE_RAMP_DOWN:
                    if (VS_RPM_Rotor > VC_ROTOR_DIR_RAMP_STEP_RPM)
                    {
                        VS_RPM_Rotor = (uint16_t)(VS_RPM_Rotor - VC_ROTOR_DIR_RAMP_STEP_RPM);
                    }
                    else
                    {
                        VS_RPM_Rotor          = 0U;
                        VS_RotNeutralTimer_ms = 0U;
                        VS_RotDirState        = ROT_DIR_STATE_NEUTRAL;
                    }
                    break;

                case ROT_DIR_STATE_NEUTRAL:
                    VS_RPM_Rotor      = 0U;
                    VS_RotorDirection = DIR_STOP;

                    VS_RotNeutralTimer_ms =
                        (uint16_t)(VS_RotNeutralTimer_ms + MACHINE_DRIVE_TASK_PERIOD_MS);

                    if (VS_RotNeutralTimer_ms >= VC_DIR_NEUTRAL_ROT_TIME_MS)
                    {
                        VS_RotNeutralTimer_ms  = 0U;
                        VS_RotCurrentDirection = VS_RotRequestedDir;
                        VS_RotorDirection      = VS_RotCurrentDirection;
                        VS_RotDirState         = ROT_DIR_STATE_RAMP_UP;
                    }
                    break;

                case ROT_DIR_STATE_RAMP_UP:
                {
                    uint16_t target_rpm = (rotor_rpm < VC_ROTOR_DEADZONE_RPM) ? 0U : rotor_rpm;

                    VS_RotorDirection = VS_RotCurrentDirection;

                    if ((uint32_t)VS_RPM_Rotor + (uint32_t)VC_ROTOR_DIR_RAMP_STEP_RPM < (uint32_t)target_rpm)
                    {
                        VS_RPM_Rotor = (uint16_t)(VS_RPM_Rotor + VC_ROTOR_DIR_RAMP_STEP_RPM);
                    }
                    else
                    {
                        VS_RPM_Rotor   = target_rpm;
                        VS_RotDirState = ROT_DIR_STATE_IDLE;
                    }
                    break;
                }

                default:
                    VS_RotDirState = ROT_DIR_STATE_IDLE;
                    break;
            }
        }
        else
        {
            /* FSM idle: follow mode changes smoothly (gradient) */
            uint16_t target_rpm = (rotor_rpm < VC_ROTOR_DEADZONE_RPM) ? 0U : rotor_rpm;

            VS_RotorDirection = VS_RotCurrentDirection;

            VS_RPM_Rotor = Apply_RPM_Slew(target_rpm,
                                          VS_RPM_Rotor,
                                          VC_ROTOR_RPM_SLEW_RATE);
        }

    ROTOR_DONE:
        ;
    }

}
