#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H

#include "app_typedef.h"

/* ================= Inputs ================= */
extern uint16_t VS_RPM_Wheel_Target;
extern uint16_t VS_RPM_Wheel_Current;

extern uint16_t VS_Throttle_Wheel;
extern uint16_t VS_Throttle_Rotor;

extern uint8_t  VS_KillSwitch;
extern uint8_t  VS_TurnLeft;
extern uint8_t  VS_TurnRight;

/* Raw/debounced operator direction request: 0=FWD request, 1=REV request. */
extern uint8_t  VS_Direction_Request;

/* Accepted vehicle direction used for status/broadcasting. */
extern uint8_t  VS_Direction;

extern uint8_t  VS_RotorEnable;
extern uint8_t  VS_RotorDirection;
extern uint8_t  VS_Rotormodestate;

/* ================= Outputs ================= */
extern uint16_t VS_RPM_Left;
extern uint16_t VS_RPM_Right;
extern uint16_t VS_RPM_Rotor;
extern uint16_t VS_RPM_Wheel;
extern uint8_t  VS_MotorEnable;
extern uint16_t VS_TurnEntryTime_ms;

/* ================= Status ================= */
extern uint8_t VS_FaultFlag;

/* ================= GPIO ================= */
extern uint8_t VS_GPIO5;
extern uint8_t VS_GPIO6;
extern uint8_t VS_GPIO7;
extern uint8_t VS_GPIO8;

/* ================= Drive mode ================= */
typedef enum
{
    MODE_IDLE = 0,
    MODE_SINGLE_LEFT,
    MODE_SINGLE_RIGHT
} DriveMode_En_t;

extern DriveMode_En_t VS_DriveMode;

/* ================= Buttons ================= */
extern uint8_t  VS_LeftBtn_Last;
extern uint8_t  VS_RightBtn_Last;
extern uint16_t VS_LeftBtn_HoldMs;
extern uint16_t VS_RightBtn_HoldMs;

/* ================= Direction FSM ================= */
typedef enum
{
    DIR_STATE_IDLE = 0,

    /* Old direction, equal commands, ramp to active-brake floor (or zero at rest). */
    DIR_STATE_RAMP_DOWN,

    /* Hold electronic braking while zero/rollback/safe-handover feedback is checked. */
    DIR_STATE_WAIT_ZERO,

    /* Static/startup fallback only: true neutral, 0 RPM + DIR_STOP. */
    DIR_STATE_NEUTRAL,

    /* Apply latest requested direction at the retained braking floor (or zero at rest). */
    DIR_STATE_APPLY_DIRECTION,

    /* Wait until both controllers report command-direction acknowledgement. */
    DIR_STATE_WAIT_DIRECTION_ACK,

    /* Equal low-RPM launch; feedback is used only for gating, never hunting. */
    DIR_STATE_SYNC_PICKUP,

    /* Equal common ramp after synchronized pickup. */
    DIR_STATE_RAMP_UP,

    /* Safe zero-output lock until throttle is released and wheels stop. */
    DIR_STATE_FAULT
} DirectionState_En_t;

typedef enum
{
    DIR_STOP    = 0U,
    DIR_FORWARD = 1U,
    DIR_REVERSE = 2U
} Direction_t;

/* Direction-FSM fault codes for diagnostics/Grafana. */
typedef enum
{
    WHEEL_SYNC_FAULT_NONE = 0U,
    WHEEL_SYNC_FAULT_FEEDBACK_MISSING,
    WHEEL_SYNC_FAULT_ZERO_TIMEOUT,
    WHEEL_SYNC_FAULT_DIRECTION_ACK_TIMEOUT,
    WHEEL_SYNC_FAULT_SINGLE_WHEEL_PICKUP,
    WHEEL_SYNC_FAULT_PICKUP_TIMEOUT,
    WHEEL_SYNC_FAULT_DIRECTION_MISMATCH
} WheelSyncFault_t;

extern DirectionState_En_t VS_DirState;
extern uint8_t  VS_CurrentDirection;
extern uint8_t  VS_RequestedDirection;

/* Actual direction byte sent to both wheel controllers over CAN. */
extern uint8_t  VS_WheelCommandDirection;

extern uint16_t VS_DirHoldCounter;
extern uint8_t  VS_KillSwitchPrev;
extern uint8_t  VS_KillRecoveryActive;
extern uint8_t  VS_DualButtonDriveActive;
extern uint16_t VS_TurnIntensity;
extern uint16_t VS_TurnHoldTime_ms;
extern uint16_t VS_TurnGradientDiff;
extern uint8_t  wheel_moving;

/* High-end direction/pickup diagnostics. */
extern uint8_t  VS_WheelSyncFault;
extern uint16_t VS_WheelFeedbackDiff_RPM;
extern uint16_t VS_WheelPickupCommonRPM;
extern uint16_t VS_DirectionStateTimer_ms;

/* Transient automatic re-synchronization / hill-hold diagnostics. */
extern uint8_t  VS_WheelSyncRecoveryActive;
extern uint8_t  VS_WheelSyncRecoveryCount;
extern uint8_t  VS_WheelActiveBrakeHandover; /* 1 = min-RPM braking floor active */
extern uint8_t  VS_WheelSlopeAssistActive;   /* 1 = slope/load evidence selected no-coast handover */

/* ================= Controller ================= */
typedef enum
{
    CTRL_WHEEL_LEFT = 0,
    CTRL_WHEEL_RIGHT,
    CTRL_ROTOR,
    CTRL_COUNT
} ControllerIndex_t;

typedef struct
{
    uint16_t speed_rpm;
    int16_t  motor_current_A;
    uint16_t battery_voltage_dV;

    int16_t  controller_temp_C;
    int16_t  motor_temp_C;

    uint16_t error_code;

    uint8_t  feedback_dir;
    uint8_t  command_dir;
    uint8_t  switch_status;

    uint8_t  alive;
    int32_t  bus_current_dA;
    uint16_t phase_current_RMS_A;
    uint8_t gear_status;
    uint8_t status_feedback_1;
    uint8_t status_feedback_2;
    uint8_t status_feedback_3;
    uint8_t power_mode;
    uint8_t handle_opening;
} ControllerState_t;

extern ControllerState_t VS_Controller[CTRL_COUNT];

/* ================= Currents ================= */
extern float VS_Current_1_A;
extern float VS_Current_2_A;
extern float VS_Current_3_A;

extern uint16_t VS_RWCurrent_1_A;
extern uint16_t VS_RWCurrent_2_A;
extern uint16_t VS_RWCurrent_3_A;

/* ================= Battery ================= */
typedef struct
{
    uint16_t Voltage_dV;
    int16_t  Current_dA;
    uint8_t  SOC;
    uint8_t  SOH;
    uint16_t Cycles;
    uint16_t Capacity;
    uint8_t  Switch_Status;
    int8_t   Ambient_Temp_C;
    int8_t   MOSFET_Temp_C;
    uint8_t  Alive;
    uint16_t RxTimeoutMs;

    uint8_t  BmsUniqueIdRaw[8];

    uint16_t BatteryVoltage_raw;
    uint16_t BatteryCurrent_raw;
    int8_t   BatteryTemp_C;
    uint8_t  LoadStatus;
    uint8_t  Status;

    uint16_t BatteryCapacity_raw;
    uint8_t  Charge;

    int8_t   CellThermistor_C[10];

    char     ProtocolVersion[8];
    char     BatterySerialNumber[8];

    uint8_t  AlarmBits_1;
    uint8_t  AlarmBits_2;
    uint8_t  ProtBits_1;
    uint8_t  ProtBits_2;
    uint8_t  FaultBits_1;

    uint16_t CellVoltage_raw_mV[16];

} BatteryState_t;

extern BatteryState_t VS_Battery1;
extern BatteryState_t VS_Battery2;

extern uint8_t VS_BattSOC_Percent;
extern uint8_t  VS_PsmRotorLock;
extern uint8_t  VS_PsmRotorLockDueToSocDiff;
extern uint16_t VS_PsmSocConfirmTimer_ms;
extern uint8_t s_batt_fuse_reduction_percent;
extern uint8_t VS_BatteryCutoffRotorLock;
extern uint8_t s_batt_damp_state;
extern uint16_t VS_BattDampMonitor;
extern uint8_t s_psm_soc_lock_cause;

#endif /* VEHICLE_STATE_H */
