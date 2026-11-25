#ifndef ERROR_CODE_H
#define ERROR_CODE_H

#include <stdint.h>
#include "vehicle_state.h"
#include "pal_can_conf.h"
/* ================= Config ================= */

#define EC_TASK_PERIOD_DEFAULT_MS              (10U)

/* CAN settings */
#define EC_CAN_USE_EXTENDED_ID                (0U)
#define EV_CAN_STD_ID_BASE                    (0x760U)
#define EC_LEFT_CAN_ID                        (0x791U)
#define EC_RIGHT_CAN_ID                       (0x792U)
#define EC_ROTOR_CAN_ID                       (0x793U)

#define EB1_ALARM1_CAN_ID                     (0x7A0U)
#define EB1_ALARM2_CAN_ID                     (0x7A1U)

#define EB1_PROT1_CAN_ID                      (0x7A2U)
#define EB1_PROT2_CAN_ID                      (0x7A3U)

#define EB1_FAULT1_CAN_ID                     (0x7A4U)

#define EB2_ALARM1_CAN_ID                     (0x7A5U)
#define EB2_ALARM2_CAN_ID                     (0x7A6U)

#define EB2_PROT1_CAN_ID                      (0x7A7U)
#define EB2_PROT2_CAN_ID                      (0x7A8U)

#define EB2_FAULT1_CAN_ID                     (0x7A9U)
#define EC_CAN_EXT_ID_BASE                    (0x18FF7600UL)

/* Battery thresholds */
#define EC_BATT_UV_THRESHOLD_DV               (420U)    /* example: 42.0V */
#define EC_BATT_OT_THRESHOLD_C                (65)
#define EC_BATT_MISMATCH_SOC_DIFF_PERCENT     (10U)
#define EC_BATT_MISMATCH_VOLT_DIFF_DV         (30U)

/* Controller thresholds */
#define EC_CTRL_OT_THRESHOLD_C                (90U)
#define EC_MOTOR_OT_THRESHOLD_C               (110U)

/* Current thresholds */
#define EC_OVERCURRENT_THRESHOLD_A            (110.0f)
#define EC_OVERCURRENT_DEBOUNCE_MS            (20U)

/* Button thresholds */
#define EC_BUTTON_STUCK_MS                    (10000U)

/* RPM response thresholds */
#define EC_WHEEL_CMD_MIN_RPM                  (0U)
#define EC_WHEEL_ACTUAL_MIN_RPM               (50U)
#define EC_WHEEL_NO_RESPONSE_MS               (1500U)

#define EC_ROTOR_ACTUAL_MIN_RPM               (100U)
#define EC_ROTOR_NO_RESPONSE_MS               (1500U)

#define EC_WHEEL_IMBALANCE_RPM                (600U)

/* Direction transition thresholds */
#define EC_DIR_TRANSITION_TIMEOUT_MS          (3000U)

/* Throttle threshold */
#define EC_WHEEL_THROTTLE_ACTIVE_THRESHOLD    (10U)

/* ================= Types ================= */


typedef enum
{
    EV_NONE = 0,

    EV001_KILL_SWITCH_OFF = 1,
    EV002_KILL_RECOVERY_ACTIVE,
   /* EV003_MOTOR_DIRECTION_DATA,*/
    /* EV003_ROTOR_CMD_ENABLE_AMBIGUITY, */
    EV003_INVALID_DUAL_TURN_CMD,
    /* EV006_INVALID_INTERNAL_DIRECTION_STATE, */
    EV004_INVALID_ROTOR_DIRECTION_STATE,
    EV005_INVALID_ROTOR_MODE_STATE,
    /* EV008_ROTOR_DIR_INPUT_CONFLICT, */
    EV006_LEFT_CTRL_CAN_LOST,
    EV007_RIGHT_CTRL_CAN_LOST,
    EV008_ROTOR_CTRL_CAN_LOST,
    EV009_BATTERY1_CAN_LOST,
    EV010_BATTERY2_CAN_LOST,
    /* EV014_REQUIRED_NODE_OFFLINE, */
    EV011_LOW_SOC_ROTOR_LOCK_ACTIVE,
    EV012_LOW_SOC_CONFIRMED,
    EV013_BATTERY1_UNDERVOLTAGE,
    EV014_BATTERY2_UNDERVOLTAGE,
    EV015_BATTERY1_OVERTEMP,
    EV016_BATTERY2_OVERTEMP,
    EV017_BATTERY1_BMS_ALARM_ACTIVE,
    EV018_BATTERY2_BMS_ALARM_ACTIVE,
    EV019_BATTERY_PACK_MISMATCH,
    EV020_LEFT_CTRL_FAULT_ACTIVE,
    EV021_RIGHT_CTRL_FAULT_ACTIVE,
    EV022_ROTOR_CTRL_FAULT_ACTIVE,
    EV023_CONTROLLER_OVERTEMP,
    EV024_MOTOR_OVERTEMP,
    EV025_CTRL_FEEDBACK_DIR_MISMATCH,
    EV026_WHEEL_RPM_NOT_RESPONDING,
    EV027_LEFT_RIGHT_WHEEL_RPM_IMBALANCE,
    /* EV032_ROTOR_RPM_NOT_RESPONDING, */
    EV028_ONLY_ONE_BATTERY_ALIVE,
    /* EV034_THROTTLE_IGNORED_DURING_DIR_TRANSITION, */
    /* EV035_INVALID_DRIVE_MODE_GPIO_COMBINATION, */
    EV029_SOC_DIFF_PSM_ROTORLOCK,
    /* EV038_OVERCURRENT_EVENT, */
    EV030_BATTERY_FUSE_LATCHED,
   /* EV040_DRIVE_INHIBITED, */
   /* EV041_ROTOR_INHIBITED, */

    EV_MAX
} EV_Code_t;

typedef struct
{
    uint8_t active[EV_MAX];
} EV_State_t;

typedef struct
{
    uint32_t id;
    uint8_t  dlc;
    uint8_t  is_extended;
    uint8_t  data[8];
} EV_CanFrame_t;

/* ================= Globals ================= */
extern EV_State_t g_ev_state;

/* ================= API ================= */
void EC_Init(void);
void EC_Update(uint16_t period_ms);
void EC_QueueAllActiveErrors(void);
uint8_t EC_IsActive(EV_Code_t code);
void EC_ClearAll(void);

/* New-error CAN event API */
uint8_t  EV_HasPendingCanEvent(void);
uint8_t  EV_GetNextPendingCanFrame(EV_CanFrame_t *frame);
uint32_t EV_GetCanIdForError(EV_Code_t code);
uint8_t  EC_GetNextPendingCanFrame(EV_CanFrame_t *frame);
uint8_t  EB_GetNextPendingCanFrame(EV_CanFrame_t *frame);

#endif /* ERROR_CODE_H */
