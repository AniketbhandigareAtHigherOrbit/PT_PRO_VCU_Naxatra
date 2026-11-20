#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

/* Rotor protections — set to 0U for normal/production use.
 * Stale bypass retains the last received command if drive updates stop. */
#ifndef VC_ROTOR_BATTERY_STOP_BYPASS
#define VC_ROTOR_BATTERY_STOP_BYPASS 0U
#endif
#ifndef VC_ROTOR_STALE_STOP_BYPASS
#define VC_ROTOR_STALE_STOP_BYPASS 1U
#endif

/* Rotor lever interlock — 1U = bypass (no lever required to run rotor).
 * Set to 0U to restore lever interlock for production. */
#ifndef VC_ROTOR_LEVER_BYPASS
#define VC_ROTOR_LEVER_BYPASS 0U
#endif

#include "app_typedef.h"
#define VC_RPM_SLEW_RATE        60U   /* RPM per cycle */
#define ONROAD_TASK_PERIOD_MS   20U    
//#define VC_RPM_SLEW_RATE_TURN   80U
#define VC_RPM_TARGETRPM_TURN   0U //300
#define VC_THROTTLE_DEAD_ZONE  180U
#define VC_LONG_PRESS_MS        10U   /* long press threshold */
#define VC_SINGLE_WHEEL_LOW_RPM 100U   /* slower wheel RPM */
#define VC_MAX_RPM_ROTOR        3000U
#define VC_ROTOR_DEADZONE_RPM   180U
#define VC_DIR_RAMP_STEP_RPM     50U //////    80
#define VC_DIR_NEUTRAL_CYCLES    5U    /* 5 × 10ms = 50ms */
#define VC_DIR_NEUTRAL_TIME_MS   900U   /* same as safety_pause = 0.3s */
#define VC_DIR_NEUTRAL_ROT_TIME_MS   30U   /* same as safety_pause = 0.3s */
#define VC_DUAL_TURN_STOP_ENABLE   1U
//#define VC_DIR_APPLY_SETTLE_TIME_MS    300U
//#define VC_DIR_RAMP_UP_FIRST_STEP_RPM   30U
//#define DIR_STATE_PRE_RAMP_ZERO    5U
/* ================= TURN LOGIC CONFIG ================= */
/* Differential scaling (percentage of base RPM) */
#define VC_TURN_DIFF_RATIO_NUM     1U
#define VC_TURN_DIFF_RATIO_DEN     2U      /* 50% of base RPM */
/* Differential RPM limits */
#define VC_TURN_DIFF_MIN_RPM       0U    /* sharper low-speed turn */
#define VC_TURN_DIFF_MAX_RPM       700U    /* aggressive but safe high-speed */
/* Mid-speed boost */
#define VC_TURN_MID_RPM_START      400U
#define VC_TURN_MID_RPM_END        1000U
#define VC_TURN_MID_BOOST_RPM      250U
/* Inner wheel floor (never stall) */
#define VC_TURN_INNER_MIN_RPM      0U //150
/* Slew rates during turn */
#define VC_RPM_SLEW_RATE_TURN_INNER  150U
#define VC_RPM_SLEW_RATE_TURN_OUTER  100U
#define VC_TURN_INTENSITY_SLEW     100U   /* how fast turn builds */
#define VC_TURN_LOW_RPM_LIMIT     550U  /* below this, soften turn */
#define VC_TURN_SOFT_DIFF_MIN     100U   /* small diff at low RPM --- 50*/
#define VC_SAFETY_ENTRY_TIME_MS     500U
#define VC_SAFETY_ENTRY_SLEW_RPM    50U

/* ---------- Turn intensity slew ---------- */
#define VC_TURN_INTENSITY_SLEW_LOW    80U
#define VC_TURN_INTENSITY_SLEW_HIGH   40U   /* slows entry at high speed */
/* ---------- Turn RPM limits ---------- */
#define VC_TURN_OUTER_BOOST_MAX       250U  /* prevents handle pull */
/* ---------- Turn exit ---------- */
#define VC_RPM_SLEW_RATE_EXIT         500U  /* faster straightening */
#define VC_RPM_SLEW_RATE_TURN         90U
#define VC_TURN_HIGH_RPM_START        900U
#define VC_KILL_RECOVERY_RPM_LIMIT   300U

/* ================= LOW RPM TURN GRADIENT ================= */
/* Enable gradient turn at low RPM */
#define VC_TURN_GRADIENT_ENABLE        1U
/* Time before sharpening starts */
#define VC_TURN_GRADIENT_DELAY_MS      2500U   /* 3 seconds */
/* Gradient ramp */
#define VC_TURN_GRADIENT_STEP_MS       100U    /* update every 100ms */
#define VC_TURN_GRADIENT_STEP_RPM      55U     /* increase diff by this */
/* Limits */
#define VC_TURN_GRADIENT_MAX_DIFF      400U    /* cap sharpening */
/* Active only below this RPM */
#define VC_TURN_GRADIENT_RPM_LIMIT     600U
/* ===== High RPM Turn Entry Softening ===== */
#define VC_TURN_ENTRY_SOFTEN_ENABLE        1U

#define VC_TURN_ENTRY_TIME_MS              200U   /* very short */
#define VC_TURN_ENTRY_MAX_RATIO_NUM        1U
#define VC_TURN_ENTRY_MAX_RATIO_DEN        3U     /* 33% of diff initially */
#define VC_TURN_ENTRY_RPM_START            600U   /* apply above this */
      
/* ================= SHARP LOW-SPEED TURN ================= */
#define VC_TURN_SHARP_ENABLE                 (1U)
#define VC_TURN_SHARP_RPM_LIMIT              (600U)
#define VC_TURN_SHARP_HOLD_MS                (2000U)

/* Legacy setting retained so any existing references remain compatible. */
#define VC_TURN_INNER_ZERO_HOLD_MS         2000U   /* after this hold, inner wheel becomes 0 */
#define VC_ACTIVE_BRK_MIN_RPM              02U
/* ================= ROTOR FIXED SPEED MODES =================
 * Selected by 2 GPIO (2-bit mode)
 * 00 -> Mode1, 01 -> Mode2, 10 -> Mode3, 11 -> Mode4
 * Units: RPM
 */
#define VC_ROTOR_MODE1_RPM               2000U
#define VC_ROTOR_MODE2_RPM               2350U
#define VC_ROTOR_MODE3_RPM               2660U
#define VC_ROTOR_MODE4_RPM               2999U

#define VC_ROTOR_DIR_RAMP_STEP_RPM       900U
#define VC_ROTOR_DIR_NEUTRAL_TIME_MS     100U
#define VC_ROTOR_RPM_SLEW_RATE           900U

/* =========================================================
 *  ROTOR MODE ADC CONFIG
 
#define VC_ROTOR_MODE0_ADC_MIN   1U
#define VC_ROTOR_MODE0_ADC_MAX   800U

#define VC_ROTOR_MODE1_ADC_MIN   900U
#define VC_ROTOR_MODE1_ADC_MAX   1400U

#define VC_ROTOR_MODE2_ADC_MIN   1500U
#define VC_ROTOR_MODE2_ADC_MAX   2500U

#define VC_ROTOR_MODE3_ADC_MIN   3100U
#define VC_ROTOR_MODE3_ADC_MAX   4150U
* ========================================================= */

#define VC_ROTOR_MODE0_ADC_MIN   1U
#define VC_ROTOR_MODE0_ADC_MAX   800U

#define VC_ROTOR_MODE1_ADC_MIN   900U
#define VC_ROTOR_MODE1_ADC_MAX   1650U

#define VC_ROTOR_MODE2_ADC_MIN   1700U
#define VC_ROTOR_MODE2_ADC_MAX   2800U

#define VC_ROTOR_MODE3_ADC_MIN   3000U
#define VC_ROTOR_MODE3_ADC_MAX   4150U

//#define VC_S1_BTRY1_ID_DEFAULT  0x0746F41FUL

#define VC_S1_BTRY1_ID_DEFAULT  0x0746CD62UL
#define VC_S1_BTRY2_ID_DEFAULT  0x0746D608UL
 
#define VC_S2_BTRY1_ID_DEFAULT  0x0746CE3EUL
#define VC_S2_BTRY2_ID_DEFAULT  0x0746CD04UL

#define VC_S3_BTRY1_ID_DEFAULT  0x0746F41FUL
#define VC_S3_BTRY2_ID_DEFAULT  0x0746F418UL

#define VC_S4_BTRY1_ID_DEFAULT  0x0746F423UL
#define VC_S4_BTRY2_ID_DEFAULT  0x0746F413UL


#define VC_BATTERY_PREFIX_FILTER_ENABLE  (1U)
#define VC_BATTERY_PREFIX_ID_BASE1       (0x0046F000UL)
#define VC_BATTERY_PREFIX_ID_BASE2       (0x0046C000UL)
#define VC_BATTERY_PREFIX_ID_BASE3       (0x0046D000UL)
#define VC_BATTERY_PREFIX_ID_BASE4       (0x0046E000UL)
#define VC_BATTERY_PREFIX_ID_MASK        (0x10FFF000UL)

/* =========================================================
 *  ADC CONFIG
 * ========================================================= */
extern const uint16_t VC_ADC_MAX_VALUE;

/* =========================================================
 *  RPM CONFIG
 * ========================================================= */
extern const uint16_t VC_WHEEL_MAX_RPM;
extern const uint16_t VC_ROTOR_MAX_RPM;

/* =========================================================
 *  CONTROLLER DEVICE IDs
 * ========================================================= */
extern const uint8_t VC_WHEEL_LEFT_ID;
extern const uint8_t VC_WHEEL_RIGHT_ID;
extern const uint8_t VC_ROTOR_ID;

/* =========================================================
 *  CAN CONFIG
 * ========================================================= */
extern const uint8_t VC_CAN_ENABLE_DEFAULT;
extern const uint8_t VC_DIRECTION_DEFAULT;

/* =========================================================
 *  CAN BASE ID (Message 3)
 * ========================================================= */
extern const uint32_t VC_BASE_CAN_ID;
extern const uint16_t VC_TURN_REDUCTION_RPM;
extern const uint16_t VC_TURN_LOW_RPM;

/*==========================================================
 * COUNTER FOR STARTUP FRAMES
 *========================================================*/
extern uint16_t STARTUP_FRAME_COUNTER ;
extern uint8_t VC_OvercurrentProtectionFaultFlag;


//extern const uint32_t VC_S1_BTRY1_ID;
//extern const uint32_t VC_S1_BTRY2_ID;
//extern const uint32_t VC_S2_BTRY1_ID;
//extern const uint32_t VC_S2_BTRY2_ID;

/* =========================================================
 *  BATTERY FUSE-SAVING / CURRENT DERATING CONFIG
 *  These values were previously local defaults in the
 *  machine-drive logic. Keep tuning centralized here.
 * ========================================================= */
#define VC_BATT_FUSE_SAVE_ENABLE                  (1U)
#define VC_BATT_FUSE_RX_TIMEOUT_MS                (600U)

/* Battery damping cycle timing */
#define VC_BATT_DAMP_ENTRY_CONFIRM_MS             (2000U) //2000
#define VC_BATT_DAMP_ACTIVE_MS                    (8000U) //6000
#define VC_BATT_DAMP_REENTRY_DELAY_MS             (1700U) //1700

/* BatteryCurrent_raw thresholds */
#define VC_BATT_FUSE_WARN_CURRENT_RAW             (4200U)
#define VC_BATT_FUSE_DERATE1_CURRENT_RAW          (5000U)
#define VC_BATT_FUSE_DERATE2_CURRENT_RAW          (5800U)
#define VC_BATT_FUSE_HIGH_CURRENT_RAW             (6500U)

#define VC_BATT_ROTOR_LOCK_CURRENT_RAW            (8100U)   /* 80.00 A */
#define VC_BATT_ROTOR_LOCK_CONFIRM_MS             (240U)     /* 3 x 20 ms */
/* Battery cut-off detection */
#define VC_BATT_FUSE_ZERO_CURRENT_RAW             (300U)
#define VC_BATT_FUSE_OTHER_HIGH_RAW               (5800U)
#define VC_BATT_CUTOFF_CONFIRM_MS                 (60U)     /* 3 x 20 ms */

#define VC_BATT_FUSE_CONFIRM_MS                   (200U)
#define VC_BATT_FUSE_SLOPE_DROP_RAW               (2000U)
#define VC_BATT_FUSE_SLOPE_RISE_RAW               (2000U)

/* Wheel-RPM reduction by current region */
#define VC_BATT_FUSE_REDUCTION_WARN_PERCENT       (10U)
#define VC_BATT_FUSE_REDUCTION_DERATE1_PERCENT    (15U)
#define VC_BATT_FUSE_REDUCTION_DERATE2_PERCENT    (35U)
#define VC_BATT_FUSE_REDUCTION_HIGH_PERCENT       (46U)
#define VC_BATT_FUSE_REDUCTION_LATCH_PERCENT      (80U)
#define VC_BATT_FUSE_MIN_WHEEL_RPM                (450U)

/* =========================================================
 * ROTOR CONTROL DURING NORMAL BATTERY-CURRENT DAMPING
 *
 * 10% / 15% : keep user-selected rotor mode
 * 35%       : run one mode below user selection (Mode 1 minimum)
 * 46%       : cap rotor at Mode 2; never increase user selection
 *              and keep the cap until the hold timer expires
 * Critical current / cutoff / fuse latch / SOC / MOSFET locks
 * remain higher priority and stop the rotor completely.
 * ========================================================= */

//#define VC_BATT_ROTOR_MODE_DERATE_ENABLE           (0U)
//#define VC_BATT_ROTOR_HIGH_DAMP_MODE_INDEX         (0U)      /* Mode 2: internal index 1 */
//#define VC_BATT_ROTOR_MODE_HOLD_MS                 (1U)  /* 40 seconds */

//#define VC_BATT_FUSE_MIN_WHEEL_RPM                (400U)
/* Battery-data validity checks */

#define VC_BATT_VALID_VOLTAGE_RAW                 (2500U)
//#define VC_BATT_VALID_SOC_PERCENT                 (5U)

/* ================= POWER SAVING (SOC BASED) ================= */
/* Set to 0U to disable — rotor runs regardless of SOC, battery alive, or MOSFET temp. */
#define VC_PSM_SOC_ENABLE              (1U)
/* Trip threshold (enter power-saving) */
#define VC_PSM_SOC_TRIP_PERCENT        (5U)
/* Clear threshold (exit power-saving) - hysteresis to avoid chatter */
#define VC_PSM_SOC_CLEAR_PERCENT       (6U)
/* Rotor is immediately locked if either MOSFET temperature exceeds 55°C. */
#define VC_PSM_MOSFET_TRIP_TEMP_C      (55)
/*Both battery MOSFET temperatures must fall to 50°C or below. */
#define VC_PSM_MOSFET_CLEAR_TEMP_C     (50)
/* Rotor is  locked if SOC differnce this higher . */
#define VC_PSM_AVG_SOC_DIFF            (20U)
/* SOC must be continuously at/below trip threshold for this long */
#define VC_PSM_SOC_CONFIRM_TIME_MS     (2000U)   /* 3 seconds */

//#define VC_CURRENT_PROTECTION           (1U)   /* Enable overcurrent protection logic */
//#define VC_OVERCURRENT_THRESHOLD        (5500U)  /* Raw ADC value threshold for overcurrent */
//#define VC_OVERCURRENT_PROTECTION_RPM   (400U)   /* Wheel RPM cap during overcurrent */
//#define VC_OVERCURRENT_CONFIRM_TIME_MS  (500)
//#define VC_Current_threshold           (6500U) /* 100A */

#include "vehicle_config_direction_sync_block.h"
#endif 
