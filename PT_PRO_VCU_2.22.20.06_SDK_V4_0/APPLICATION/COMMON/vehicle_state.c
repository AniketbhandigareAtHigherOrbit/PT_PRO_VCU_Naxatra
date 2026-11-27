#include "vehicle_state.h"

/* ================= Inputs ================= */
uint16_t VS_RPM_Wheel_Target  = 0U;
uint16_t VS_RPM_Wheel_Current = 0U;

uint16_t VS_Throttle_Wheel = 0U;
uint16_t VS_Throttle_Rotor = 0U;

uint8_t VS_KillSwitch = 0U;
uint8_t VS_KillSwitchPrev = 0U;
uint8_t VS_KillRecoveryActive = 0U;
uint8_t VS_DualButtonDriveActive = 0U;
uint8_t VS_DualButtonActive_300RPM = 0U;

uint8_t VS_TurnLeft   = 0U;
uint8_t VS_TurnRight  = 0U;
uint16_t VS_TurnIntensity = 0U;   /* 0 → diff */
uint16_t VS_TurnHoldTime_ms = 0U;
uint16_t VS_TurnGradientDiff = 0U;

uint8_t VS_Direction_Request = 0U;
uint8_t VS_Direction         = DIR_FORWARD;
uint8_t VS_RotorEnable       = 0U;
//uint16_t VS_TurnHoldTime_ms = 0U;
uint16_t VS_TurnEntryTime_ms = 0U;

/* ================= Outputs ================= */
uint16_t VS_RPM_Left  = 0U;
uint16_t VS_RPM_Right = 0U;
uint16_t VS_RPM_Rotor = 0U;
uint16_t VS_RPM_Wheel = 0U;

//uint16_t base_rpm = 0U;
//uint16_t rotor_rpm = 0U;
uint8_t VS_MotorEnable = 0U;

/* ================= Status ================= */
uint8_t VS_FaultFlag = 0U;

/* ================= GPIO ================= */
uint8_t VS_GPIO5 = 0U;
uint8_t VS_GPIO6 = 0U;
uint8_t VS_GPIO7 = 0U;
uint8_t VS_GPIO8 = 0U;

/* ================= Drive mode ================= */
DriveMode_En_t VS_DriveMode = MODE_IDLE;

/* ================= Buttons ================= */
uint8_t  VS_LeftBtn_Last  = 0U;
uint8_t  VS_RightBtn_Last = 0U;

uint16_t VS_LeftBtn_HoldMs  = 0U;
uint16_t VS_RightBtn_HoldMs = 0U;

/* ================= Direction FSM ================= */
DirectionState_En_t VS_DirState = DIR_STATE_IDLE;

uint8_t  VS_CurrentDirection   = DIR_FORWARD;
uint8_t  VS_RequestedDirection = DIR_FORWARD;
uint16_t VS_DirHoldCounter     = 0U;
uint8_t  VS_RotorDirection = 0U;
uint8_t  VS_Rotormodestate = 0U;
uint8_t wheel_moving = 0U;

/* ================= Wheel direction synchronization ================= */
uint8_t  VS_WheelCommandDirection = DIR_STOP;
uint8_t  VS_WheelSyncFault = WHEEL_SYNC_FAULT_NONE;
uint16_t VS_WheelFeedbackDiff_RPM = 0U;
uint16_t VS_WheelPickupCommonRPM = 0U;
uint16_t VS_DirectionStateTimer_ms = 0U;
uint8_t  VS_WheelSyncRecoveryActive = 0U;
uint8_t  VS_WheelSyncRecoveryCount = 0U;
uint8_t  VS_WheelActiveBrakeHandover = 0U;
uint8_t  VS_WheelSlopeAssistActive = 0U;
/* ================= Controller ================= */
ControllerState_t VS_Controller[CTRL_COUNT] =
{
    {0}, {0}, {0}
};

/* ================= Currents ================= */
float VS_Current_1_A = 0.0f;
float VS_Current_2_A = 0.0f;
float VS_Current_3_A = 0.0f;

uint16_t VS_RWCurrent_1_A = 0U;
uint16_t VS_RWCurrent_2_A = 0U;
uint16_t VS_RWCurrent_3_A = 0U;


/* ================= Battery ================= */
/* Order MUST match BatteryState_t exactly */

BatteryState_t VS_Battery1 =
{
    /* legacy/basic */
    0U,     /* Voltage_dV */
    0,      /* Current_dA */
    0U,     /* SOC */
    0U,     /* SOH */
    0U,     /* Cycles */
    0U,     /* Capacity */
    0U,     /* Switch_Status */
    0,      /* Ambient_Temp_C */
    0,      /* MOSFET_Temp_C */
    0U,     /* Alive */
    0U,     /* RxTimeoutMs */
    /* new fields */
    {0U,0U,0U,0U,0U,0U,0U,0U},   /* BmsUniqueIdRaw[8] */
    0U,     /* BatteryVoltage_raw */
    0U,     /* BatteryCurrent_raw */
    0,      /* BatteryTemp_C */
    0U,     /* LoadStatus */
    0U,     /* Status */
    0U,     /* BatteryCapacity_raw */
    0U,     /* Charge */
    {0,0,0,0,0,0,0,0,0,0},       /* CellThermistor_C[10] */
    {0,0,0,0,0,0,0,0},           /* ProtocolVersion[8] */
    {0,0,0,0,0,0,0,0},           /* BatterySerialNumber[8] */
    0U,     /* AlarmBits_1 */
    0U,     /* AlarmBits_2 */
    0U,     /* ProtBits_1 */
    0U,     /* ProtBits_2 */
    0U,     /* FaultBits_1 */
    {0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U} /* CellVoltage_raw_mV[16] */
};

BatteryState_t VS_Battery2 =
{
    /* legacy/basic */
    0U,     /* Voltage_dV */
    0,      /* Current_dA */
    0U,     /* SOC */
    0U,     /* SOH */
    0U,     /* Cycles */
    0U,     /* Capacity */
    0U,     /* Switch_Status */
    0,      /* Ambient_Temp_C */
    0,      /* MOSFET_Temp_C */
    0U,     /* Alive */
    0U,     /* RxTimeoutMs */
    /* new fields */
    {0U,0U,0U,0U,0U,0U,0U,0U},   /* BmsUniqueIdRaw[8] */
    0U,     /* BatteryVoltage_raw */
    0U,     /* BatteryCurrent_raw */
    0,      /* BatteryTemp_C */
    0U,     /* LoadStatus */
    0U,     /* Status */
    0U,     /* BatteryCapacity_raw */
    0U,     /* Charge */
    {0,0,0,0,0,0,0,0,0,0},       /* CellThermistor_C[10] */
    {0,0,0,0,0,0,0,0},           /* ProtocolVersion[8] */
    {0,0,0,0,0,0,0,0},           /* BatterySerialNumber[8] */
    0U,     /* AlarmBits_1 */
    0U,     /* AlarmBits_2 */
    0U,     /* ProtBits_1 */
    0U,     /* ProtBits_2 */
    0U,     /* FaultBits_1 */
    {0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U} /* CellVoltage_raw_mV[16] */
};

uint8_t  VS_BattSOC_Percent = 100U;
uint8_t  VS_PsmRotorLock = 0U;
uint8_t  VS_PsmRotorLockDueToSocDiff = 0U;
uint16_t VS_PsmSocConfirmTimer_ms = 0U;
uint8_t s_batt_fuse_reduction_percent = 0U;
uint8_t VS_BatteryCutoffRotorLock = 0U;
uint8_t s_batt_damp_state = 0U;
uint16_t VS_BattDampMonitor = 0U;
