#include "Can_Broadcast.h"
#include "AppTest.h"
#include "vehicle_state.h"
#include "error_code.h"
#include "vehicle_config.h"
#include "battery_can_rx.h"
#include "dashboard_can_rx.h"
#include "controller_can_rx.h"
#include <string.h>

#define BROADCAST_CAN_CHANNEL    CAN_0

/* ================= CAN IDs ================= */
#define CANID_VCU_CMD_FB_1       0x700U
#define CANID_VCU_CMD_FB_2       0x701U
#define CANID_VCU_CUR_DIR        0x702U
#define CANID_VCU_SWITCH_DIR     0x703U
#define CANID_CTRL_ERRORS        0x704U
#define CANID_BMS1_FAST          0x705U
#define CANID_BMS2_FAST          0x706U
#define CANID_BMS1_WVC           0x707U
#define CANID_BMS2_WVC           0x708U
#define CANID_WHEEL_SYNC_DIAG     0x709U

#define CANID_ROTOR_DC_BUS       0x750U
#define CANID_ROTOR_DQ_STATUS    0x751U
#define CANID_CTRL_TEMPS         0x720U
#define CANID_BMS1_TEMPS         0x721U
#define CANID_BMS1_CELL_1_4      0x722U
#define CANID_BMS1_CELL_5_8      0x723U
#define CANID_BMS1_CELL_9_12     0x724U
#define CANID_BMS1_CELL_13_16    0x725U
#define CANID_BMS2_TEMPS         0x726U
#define CANID_BMS2_CELL_1_4      0x727U
#define CANID_BMS2_CELL_5_8      0x728U
#define CANID_BMS2_CELL_9_12     0x729U
#define CANID_BMS2_CELL_13_16    0x72AU
#define CODE_VERSION             0x600U

#define CANID_STARTUP_BMS1       0x740U
#define CANID_STARTUP_BMS2       0x741U

#define DISPLASY_DATA1       0x300U
#define DISPLASY_DATA2       0x301U
#define DISPLASY_DATA3       0x302U
#define DISPLAY_VERSION      0x601U

#define CODE_VERSION_MAJOR       2U
#define CODE_VERSION_MINOR       1U
#define CODE_VERSION_PATCH1      1U
#define CODE_VERSION_PATCH2      0U
#define CODE_VERSION_BUILD1      0U
#define CODE_VERSION_BUILD2      6U

/* ================= Timing ================= */
#define BROADCAST_1S_TICKS       10U 
#define BROADCAST_60S_TICKS      600U 

/* 0x233 has no cycle-time attribute in the supplier's R0 DBC. Mark its
 * latest values fresh for 600 ms after reception; always send a validity
 * bit so IoT can reject stale or never-received Id/Iq measurements. */
#define ROTOR_DQ_FRESHNESS_MS    600U

/* ================= Timing for 100ms and 1sec =================
#define BROADCAST_1S_TICKS       1U 
#define BROADCAST_60S_TICKS      10U
 =================                             ================= */
#define STARTUP_FRAME_PERIOD_TICKS  BROADCAST_60S_TICKS
#define STARTUP_FRAME_TX_LIMIT      60U


uint8_t reset_counter = 0U;
static uint16_t BROADCAST_STARTUP_COUNTER = 0U;

/* ================= Helpers ================= */
#define CTRL_IS_ALIVE(idx) (VS_Controller[(idx)].alive)

#define U16_TO_CAN_BE(buf, pos, val)                                  \
    do                                                                \
    {                                                                 \
        (buf)[(pos)]      = (uint8_t)(((uint16_t)(val) >> 8U) & 0xFFU); \
        (buf)[(pos) + 1U] = (uint8_t)((uint16_t)(val) & 0xFFU);        \
    } while (0)


#define U16_TO_CAN_LE(buf, idx, val)                                  \
    do {                                                              \
        (buf)[(idx)]     = (uint8_t)((uint16_t)(val) & 0xFFU);        \
        (buf)[(idx) + 1] = (uint8_t)(((uint16_t)(val) >> 8) & 0xFFU); \
    } while (0)

static uint16_t Broadcast_1sCounter = 0U;
static uint16_t Broadcast_60sCounter = 0U;
static uint8_t  Broadcast_StartupDone = 0U;
static uint8_t  Broadcast_60sPending = 0U;
static uint8_t  Broadcast_StartupFrameTxCount = 0U;

typedef struct
{
    uint8_t  bms1_soc;
    uint8_t  bms2_soc;
    uint16_t throttle_wheel;
    uint16_t throttle_rotor;
    uint16_t rpm_wheel_current;
    uint16_t rpm_rotor;
    uint8_t  rotor_mode_state;
    uint8_t  bms1_alive;
    uint8_t  bms2_alive;
} DisplaySnapshot_t;

typedef struct
{
    uint8_t left;
    uint8_t right;
    uint8_t killswitch;
    uint8_t direction_request;
    uint8_t rotor_enable;

} GpioLeftRight_t;

static DisplaySnapshot_t g_display_snapshot;

static GpioLeftRight_t g_gpio_left_right;


static uint8_t Clamp_U8_From_S16(int16_t val)
{
    if (val < 0)
    {
        return 0U;
    }

    if (val > 255)
    {
        return 255U;
    }

    return (uint8_t)val;
}

static uint16_t Current_To_CAN(float current_A)
{
    float scaled;

    if (current_A <= 0.0f)
    {
        return 0U;
    }

    scaled = current_A * 10.0f;

    if (scaled >= 65535.0f)
    {
        return 65535U;
    }

    return (uint16_t)scaled;
}

static void Broadcast_Tx(uint16_t can_id, uint8_t *buf)
{
    CanMsgTransmit(BROADCAST_CAN_CHANNEL, buf, 8U, 0U, 0U, can_id);
}

/* CAN0 0x750: signed current (0.1 A), unsigned voltage (0.1 V), big-endian.
 * Bytes 4-5 carry rotor motor/controller fault flags; bytes 6-7 carry
 * unsigned phase RMS current (0.1 A/count), big-endian.
 * Values are the latest decoded rotor feedback data. */
static void Broadcast_SendRotorDcBus(void)
{
    uint8_t buf[8];
    int32_t current_dA;
    ControllerFeedbackSnapshot_t rotor;

    if (Controller_CAN_GetSnapshot(CTRL_ROTOR, &rotor) == 0U)
    {
        return;
    }

    current_dA = rotor.bus_current_dA;
    if (current_dA > 32767L) { current_dA = 32767L; }
    if (current_dA < -32768L) { current_dA = -32768L; }
    memset(buf, 0, sizeof(buf));
    U16_TO_CAN_BE(buf, 0U, (uint16_t)(int16_t)current_dA);
    U16_TO_CAN_BE(buf, 2U, rotor.battery_voltage_dV);
    /* Keep Naxatra_CAN0_Telemetry.dbc unchanged: motor-status low nibble
     * and controller-status bit 0 are reserved in CAN0 0x750. R0 adds
     * CANRevLost/LostPhaseFault internally and in the existing 0x704 error
     * word, but they must not leak into these reserved 0x750 positions. */
    buf[4] = (uint8_t)((rotor.error_code >> 8U) & 0xF0U);
    buf[5] = (uint8_t)(rotor.error_code & 0xFEU);
    U16_TO_CAN_BE(buf, 6U, Current_To_CAN((float)rotor.phase_current_RMS_A));
    Broadcast_Tx(CANID_ROTOR_DC_BUS, buf);
}

/* CAN0 0x751: signed Iq and Id currents, 1 A/count, Motorola big-endian.
 * Byte 4 flags: bit0 LostPhaseFault, bit1 CANRevLost, bit2 DQ received,
 * bit3 DQ fresh, bit4 status 0x411 fresh, bit5 status 0x412 fresh.
 * Bytes 5-6: age of the most recent 0x233 frame, in milliseconds.
 * Byte 7 is reserved zero. All fields are defined in the IoT R1 DBC. */
static void Broadcast_SendRotorDqStatus(void)
{
    uint8_t buf[8];
    ControllerFeedbackSnapshot_t rotor;

    if (Controller_CAN_GetSnapshot(CTRL_ROTOR, &rotor) == 0U)
    {
        return;
    }

    memset(buf, 0, sizeof(buf));

    if (rotor.msg3_received != 0U)
    {
        U16_TO_CAN_BE(buf, 0U, (uint16_t)rotor.iq_current_A);
        U16_TO_CAN_BE(buf, 2U, (uint16_t)rotor.id_current_A);
        buf[4] |= 0x04U;

        if (rotor.msg3_age_ms <= ROTOR_DQ_FRESHNESS_MS)
        {
            buf[4] |= 0x08U;
        }
    }

    /* The R0 DBC calls these byte-4 bit0 and byte-5 bit0 respectively.
     * They are retained in the internal rotor error_code, but excluded
     * from reserved bits of the existing CAN0 0x750 frame. */
    if ((rotor.error_code & 0x0001U) != 0U)
    {
        buf[4] |= 0x01U;
    }
    if ((rotor.error_code & 0x0100U) != 0U)
    {
        buf[4] |= 0x02U;
    }

    if ((rotor.msg1_received != 0U) &&
        (rotor.msg1_age_ms <= VC_CONTROLLER_MSG1_TIMEOUT_MS))
    {
        buf[4] |= 0x10U;
    }
    if ((rotor.msg2_received != 0U) &&
        (rotor.msg2_age_ms <= VC_CONTROLLER_MSG2_TIMEOUT_MS))
    {
        buf[4] |= 0x20U;
    }

    U16_TO_CAN_BE(buf, 5U, rotor.msg3_age_ms);
    Broadcast_Tx(CANID_ROTOR_DQ_STATUS, buf);
}

static void Update_Display_Snapshot(void)
{
    g_display_snapshot.bms1_soc          = VS_Battery1.SOC;
    g_display_snapshot.bms2_soc          = VS_Battery2.SOC;
    g_display_snapshot.throttle_wheel    = VS_Throttle_Wheel;
    g_display_snapshot.throttle_rotor    = VS_Throttle_Rotor;
    g_display_snapshot.rpm_wheel_current = VS_RPM_Wheel_Current;
    g_display_snapshot.rpm_rotor         = VS_Controller[CTRL_ROTOR].speed_rpm;
    g_display_snapshot.rotor_mode_state  = VS_Rotormodestate;
    g_display_snapshot.bms1_alive        = VS_Battery1.Alive;
    g_display_snapshot.bms2_alive        = VS_Battery2.Alive;
}

static void reset_left_right(void)
{
    if (reset_counter > 0U)
    {
        g_gpio_left_right.left = 0U;
        g_gpio_left_right.right = 0U;
        g_gpio_left_right.killswitch = 0U;
        g_gpio_left_right.direction_request = 0U;
        g_gpio_left_right.rotor_enable = 0U;
        reset_counter = 0U;
    }
}

static void Update_Gpio_Snapshot(void)
{
    g_gpio_left_right.left |= VS_TurnLeft;
    g_gpio_left_right.right |= VS_TurnRight;
    g_gpio_left_right.killswitch |= VS_KillSwitch;
    g_gpio_left_right.direction_request |= VS_Direction_Request;
    g_gpio_left_right.rotor_enable |= VS_RotorEnable;
}

/* ================= Startup Frames ================= */

static void Broadcast_SendStartupFrames(void)
{
    uint8_t buf[8];

    memset(buf, 0, sizeof(buf));
    buf[0] = VS_Battery1.BatterySerialNumber[0];
    buf[1] = VS_Battery1.BatterySerialNumber[1];
    buf[2] = VS_Battery1.BatterySerialNumber[2];
    buf[3] = VS_Battery1.BatterySerialNumber[3];
    buf[4] = VS_Battery1.BatterySerialNumber[4];
    buf[5] = VS_Battery1.BatterySerialNumber[5];
    buf[6] = VS_Battery1.BatterySerialNumber[6];
    buf[7] = VS_Battery1.Switch_Status;
    Broadcast_Tx(CANID_STARTUP_BMS1, buf);

    memset(buf, 0, sizeof(buf));
    buf[0] = VS_Battery2.BatterySerialNumber[0];
    buf[1] = VS_Battery2.BatterySerialNumber[1];
    buf[2] = VS_Battery2.BatterySerialNumber[2];
    buf[3] = VS_Battery2.BatterySerialNumber[3];
    buf[4] = VS_Battery2.BatterySerialNumber[4];
    buf[5] = VS_Battery2.BatterySerialNumber[5];
    buf[6] = VS_Battery2.BatterySerialNumber[6];
    buf[7] = VS_Battery2.Switch_Status;
    Broadcast_Tx(CANID_STARTUP_BMS2, buf);

    memset(buf, 0, sizeof(buf));
    buf[0] = CODE_VERSION_MAJOR;
    buf[1] = CODE_VERSION_MINOR;
    buf[2] = CODE_VERSION_PATCH1;
    buf[3] = CODE_VERSION_PATCH2;
    buf[4] = CODE_VERSION_BUILD1;    
    buf[5] = CODE_VERSION_BUILD2;
    Broadcast_Tx(CODE_VERSION, buf);

    memset(buf, 0, sizeof(buf));
    buf[0] = VS_DisplayVersion.major;
    buf[1] = VS_DisplayVersion.minor;
    buf[2] = VS_DisplayVersion.patch1;
    buf[3] = VS_DisplayVersion.patch2;
    buf[4] = VS_DisplayVersion.build1;
    buf[5] = VS_DisplayVersion.build2;
    Broadcast_Tx(DISPLAY_VERSION, buf);
}

/* ================= 1 Second Frames ================= */

static void Broadcast_Send1sFrames(void)
{
    uint8_t buf[8];

    memset(buf, 0, sizeof(buf));
    U16_TO_CAN_BE(buf, 0U, VS_RPM_Left);
    U16_TO_CAN_BE(buf, 2U, VS_RPM_Right);
    U16_TO_CAN_BE(buf, 4U, VS_RPM_Rotor);
    U16_TO_CAN_BE(buf, 6U, VS_Throttle_Wheel);
    Broadcast_Tx(CANID_VCU_CMD_FB_1, buf);

    memset(buf, 0, sizeof(buf));
    U16_TO_CAN_BE(buf, 0U, VS_Controller[CTRL_WHEEL_LEFT].speed_rpm);
    U16_TO_CAN_BE(buf, 2U, VS_Controller[CTRL_WHEEL_RIGHT].speed_rpm);
    U16_TO_CAN_BE(buf, 4U, VS_Controller[CTRL_ROTOR].speed_rpm);
    U16_TO_CAN_BE(buf, 6U, VS_Throttle_Rotor);
    Broadcast_Tx(CANID_VCU_CMD_FB_2, buf);

    memset(buf, 0, sizeof(buf));
    U16_TO_CAN_BE(buf, 0U, Current_To_CAN(VS_Controller[CTRL_WHEEL_LEFT].motor_current_A));
    U16_TO_CAN_BE(buf, 2U, Current_To_CAN(VS_Controller[CTRL_WHEEL_RIGHT].motor_current_A));
    U16_TO_CAN_BE(buf, 4U, Current_To_CAN(VS_Controller[CTRL_ROTOR].motor_current_A));
    buf[6] = VS_CurrentDirection;
    buf[7] = VS_Direction;
    Broadcast_Tx(CANID_VCU_CUR_DIR, buf);

    memset(buf, 0, sizeof(buf));
    buf[0] = g_gpio_left_right.killswitch;
    buf[1] = g_gpio_left_right.left;
    buf[2] = g_gpio_left_right.right;
    buf[3] = VS_Controller[CTRL_WHEEL_LEFT].feedback_dir;
    buf[4] = VS_Controller[CTRL_WHEEL_RIGHT].feedback_dir;
    buf[5] = VS_Controller[CTRL_ROTOR].feedback_dir;
    buf[6] = g_gpio_left_right.direction_request;
    buf[7] = VS_RequestedDirection;
    Broadcast_Tx(CANID_VCU_SWITCH_DIR, buf);
    reset_counter++;

    memset(buf, 0, sizeof(buf));
    U16_TO_CAN_BE(buf, 0U, VS_Controller[CTRL_WHEEL_LEFT].error_code);
    U16_TO_CAN_BE(buf, 2U, VS_Controller[CTRL_WHEEL_RIGHT].error_code);
    U16_TO_CAN_BE(buf, 4U, VS_Controller[CTRL_ROTOR].error_code);
    buf[6] = VS_Rotormodestate;
    buf[7] = VS_DualButtonDriveActive;
    Broadcast_Tx(CANID_CTRL_ERRORS, buf);

    memset(buf, 0, sizeof(buf));
    U16_TO_CAN_BE(buf, 0U, VS_Battery1.BatteryCurrent_raw);
    U16_TO_CAN_BE(buf, 2U, VS_Battery1.BatteryVoltage_raw);
    buf[4] = VS_Battery1.LoadStatus;
    buf[5] = VS_Battery1.Alive;
    //buf[6] = s_batt_damp_state;
    buf[6] = VS_BatteryCutoffRotorLock;
    buf[7] = wheel_moving;
    Broadcast_Tx(CANID_BMS1_FAST, buf);

    memset(buf, 0, sizeof(buf));
    U16_TO_CAN_BE(buf, 0U, VS_Battery2.BatteryCurrent_raw);
    U16_TO_CAN_BE(buf, 2U, VS_Battery2.BatteryVoltage_raw);
    buf[4] = VS_Battery2.LoadStatus;
    buf[5] = VS_Battery2.Alive;
    U16_TO_CAN_BE(buf, 6U, VS_BattDampMonitor);
    Broadcast_Tx(CANID_BMS2_FAST, buf);

    /* PT_PRO_VCU_2.11.10.04 wheel synchronization diagnostics. 
    memset(buf, 0, sizeof(buf));
    buf[0] = (uint8_t)VS_DirState;
    buf[1] = VS_WheelSyncFault;
    buf[2] = VS_WheelCommandDirection;
    buf[3] = (uint8_t)(((VS_Controller[CTRL_WHEEL_LEFT].alive != 0U) ? 1U : 0U) |
                       ((VS_Controller[CTRL_WHEEL_RIGHT].alive != 0U) ? 2U : 0U) |
                       ((VS_WheelSyncRecoveryActive != 0U) ? 4U : 0U) |
                       ((VS_WheelActiveBrakeHandover != 0U) ? 8U : 0U) |
                       ((VS_WheelSlopeAssistActive != 0U) ? 16U : 0U) |
                       (uint8_t)((VS_WheelSyncRecoveryCount & 0x07U) << 5U));
    U16_TO_CAN_BE(buf, 4U, VS_WheelPickupCommonRPM);
    U16_TO_CAN_BE(buf, 6U, VS_WheelFeedbackDiff_RPM);
    Broadcast_Tx(CANID_WHEEL_SYNC_DIAG, buf);
    */
}

/* ================= 60 Second Frames ================= */

static void Broadcast_Send60sFrames(void)
{
    uint8_t buf[8];
    memset(buf, 0, sizeof(buf));
    buf[0] = Clamp_U8_From_S16(VS_Controller[CTRL_WHEEL_LEFT].controller_temp_C);
    buf[1] = Clamp_U8_From_S16(VS_Controller[CTRL_WHEEL_RIGHT].controller_temp_C);
    buf[2] = Clamp_U8_From_S16(VS_Controller[CTRL_ROTOR].controller_temp_C);
    buf[3] = Clamp_U8_From_S16(VS_Controller[CTRL_WHEEL_LEFT].motor_temp_C);
    buf[4] = Clamp_U8_From_S16(VS_Controller[CTRL_WHEEL_RIGHT].motor_temp_C);
    buf[5] = Clamp_U8_From_S16(VS_Controller[CTRL_ROTOR].motor_temp_C);

    Broadcast_Tx(CANID_CTRL_TEMPS, buf);

    memset(buf, 0, sizeof(buf));
    buf[0] = VS_Battery1.SOC;
    buf[1] = Clamp_U8_From_S16(VS_Battery1.MOSFET_Temp_C);
    buf[2] = Clamp_U8_From_S16(VS_Battery1.BatteryTemp_C);
    buf[3] = Clamp_U8_From_S16(VS_Battery1.CellThermistor_C[0]);
    buf[4] = Clamp_U8_From_S16(VS_Battery1.CellThermistor_C[1]);
    buf[5] = Clamp_U8_From_S16(VS_Battery1.CellThermistor_C[2]);
    buf[6] = Clamp_U8_From_S16(VS_Battery1.CellThermistor_C[3]);
    Broadcast_Tx(CANID_BMS1_TEMPS, buf);
    
    memset(buf, 0, sizeof(buf));
    U16_TO_CAN_BE(buf, 0U, VS_Battery1.CellVoltage_raw_mV[0]);
    U16_TO_CAN_BE(buf, 2U, VS_Battery1.CellVoltage_raw_mV[1]);
    U16_TO_CAN_BE(buf, 4U, VS_Battery1.CellVoltage_raw_mV[2]);
    U16_TO_CAN_BE(buf, 6U, VS_Battery1.CellVoltage_raw_mV[3]);
    Broadcast_Tx(CANID_BMS1_CELL_1_4, buf);

    memset(buf, 0, sizeof(buf));
    U16_TO_CAN_BE(buf, 0U, VS_Battery1.CellVoltage_raw_mV[4]);
    U16_TO_CAN_BE(buf, 2U, VS_Battery1.CellVoltage_raw_mV[5]);
    U16_TO_CAN_BE(buf, 4U, VS_Battery1.CellVoltage_raw_mV[6]);
    U16_TO_CAN_BE(buf, 6U, VS_Battery1.CellVoltage_raw_mV[7]);
    Broadcast_Tx(CANID_BMS1_CELL_5_8, buf);

    memset(buf, 0, sizeof(buf));
    U16_TO_CAN_BE(buf, 0U, VS_Battery1.CellVoltage_raw_mV[8]);
    U16_TO_CAN_BE(buf, 2U, VS_Battery1.CellVoltage_raw_mV[9]);
    U16_TO_CAN_BE(buf, 4U, VS_Battery1.CellVoltage_raw_mV[10]);
    U16_TO_CAN_BE(buf, 6U, VS_Battery1.CellVoltage_raw_mV[11]);
    Broadcast_Tx(CANID_BMS1_CELL_9_12, buf);

    memset(buf, 0, sizeof(buf));
    U16_TO_CAN_BE(buf, 0U, VS_Battery1.CellVoltage_raw_mV[12]);
    U16_TO_CAN_BE(buf, 2U, VS_Battery1.CellVoltage_raw_mV[13]);
    U16_TO_CAN_BE(buf, 4U, VS_Battery1.CellVoltage_raw_mV[14]);
    U16_TO_CAN_BE(buf, 6U, VS_Battery1.CellVoltage_raw_mV[15]);
    Broadcast_Tx(CANID_BMS1_CELL_13_16, buf);

    memset(buf, 0, sizeof(buf));
    buf[0] = VS_Battery2.SOC;
    buf[1] = Clamp_U8_From_S16(VS_Battery2.MOSFET_Temp_C);
    buf[2] = Clamp_U8_From_S16(VS_Battery2.BatteryTemp_C);
    buf[3] = Clamp_U8_From_S16(VS_Battery2.CellThermistor_C[0]);
    buf[4] = Clamp_U8_From_S16(VS_Battery2.CellThermistor_C[1]);
    buf[5] = Clamp_U8_From_S16(VS_Battery2.CellThermistor_C[2]);
    buf[6] = Clamp_U8_From_S16(VS_Battery2.CellThermistor_C[3]);
    Broadcast_Tx(CANID_BMS2_TEMPS, buf);

    //Broadcast_SendCodeVersion();

    memset(buf, 0, sizeof(buf));
    U16_TO_CAN_BE(buf, 0U, VS_Battery2.CellVoltage_raw_mV[0]);
    U16_TO_CAN_BE(buf, 2U, VS_Battery2.CellVoltage_raw_mV[1]);
    U16_TO_CAN_BE(buf, 4U, VS_Battery2.CellVoltage_raw_mV[2]);
    U16_TO_CAN_BE(buf, 6U, VS_Battery2.CellVoltage_raw_mV[3]);
    Broadcast_Tx(CANID_BMS2_CELL_1_4, buf);

    memset(buf, 0, sizeof(buf));
    U16_TO_CAN_BE(buf, 0U, VS_Battery2.CellVoltage_raw_mV[4]);
    U16_TO_CAN_BE(buf, 2U, VS_Battery2.CellVoltage_raw_mV[5]);
    U16_TO_CAN_BE(buf, 4U, VS_Battery2.CellVoltage_raw_mV[6]);
    U16_TO_CAN_BE(buf, 6U, VS_Battery2.CellVoltage_raw_mV[7]);
    Broadcast_Tx(CANID_BMS2_CELL_5_8, buf);

    memset(buf, 0, sizeof(buf));
    U16_TO_CAN_BE(buf, 0U, VS_Battery2.CellVoltage_raw_mV[8]);
    U16_TO_CAN_BE(buf, 2U, VS_Battery2.CellVoltage_raw_mV[9]);
    U16_TO_CAN_BE(buf, 4U, VS_Battery2.CellVoltage_raw_mV[10]);
    U16_TO_CAN_BE(buf, 6U, VS_Battery2.CellVoltage_raw_mV[11]);
    Broadcast_Tx(CANID_BMS2_CELL_9_12, buf);

    memset(buf, 0, sizeof(buf));
    U16_TO_CAN_BE(buf, 0U, VS_Battery2.CellVoltage_raw_mV[12]);
    U16_TO_CAN_BE(buf, 2U, VS_Battery2.CellVoltage_raw_mV[13]);
    U16_TO_CAN_BE(buf, 4U, VS_Battery2.CellVoltage_raw_mV[14]);
    U16_TO_CAN_BE(buf, 6U, VS_Battery2.CellVoltage_raw_mV[15]);
    Broadcast_Tx(CANID_BMS2_CELL_13_16, buf);

    memset(buf, 0, sizeof(buf));
    buf[0] = VS_Battery1.SOH;
    U16_TO_CAN_BE(buf, 1U, VS_Battery1.Cycles);
    U16_TO_CAN_BE(buf, 3U, VS_Battery1.BatteryCapacity_raw);
    buf[5] = s_psm_soc_lock_cause;
    Broadcast_Tx(CANID_BMS1_WVC, buf);
 
    memset(buf, 0, sizeof(buf));
    buf[0] = VS_Battery2.SOH;
    U16_TO_CAN_BE(buf, 1U, VS_Battery2.Cycles);
    U16_TO_CAN_BE(buf, 3U, VS_Battery2.BatteryCapacity_raw);
    Broadcast_Tx(CANID_BMS2_WVC, buf);

}

static void Send_Display_(void)
{
    uint8_t buf[8];
    static uint8_t display_100ms_counter = 0U;

    display_100ms_counter++;

    if (display_100ms_counter >= 5U)
    {
        display_100ms_counter = 0U;
        Update_Display_Snapshot();
    }

    memset(buf, 0, sizeof(buf));
    buf[0] = g_display_snapshot.bms1_soc;
    buf[1] = g_display_snapshot.bms2_soc;
    U16_TO_CAN_LE(buf, 2U, g_display_snapshot.throttle_wheel);
    U16_TO_CAN_LE(buf, 4U, g_display_snapshot.throttle_rotor);
    U16_TO_CAN_LE(buf, 6U, g_display_snapshot.rpm_wheel_current);
    Broadcast_Tx(DISPLASY_DATA1, buf);

    memset(buf, 0, sizeof(buf));
    U16_TO_CAN_LE(buf, 0U, g_display_snapshot.rpm_rotor);
    buf[2] = g_display_snapshot.rotor_mode_state;
    buf[3] = g_display_snapshot.bms1_alive;
    buf[4] = g_display_snapshot.bms2_alive;
    
    buf[7] = s_psm_soc_lock_cause;
    Broadcast_Tx(DISPLASY_DATA2, buf);    
}

static void Gpio_left_right(void)
{
    static uint8_t gpio_100ms_counter = 0U;
    
    gpio_100ms_counter++;

    if (gpio_100ms_counter >= 10U)
    {   
        reset_left_right();
        Update_Gpio_Snapshot();
        gpio_100ms_counter = 0U;
        
    }
   
}

static void Send_Pending_Error_Frames(void)
{
    EV_CanFrame_t frame;

    /* ================= EV ================= */

    if(EV_GetNextPendingCanFrame(&frame) != 0U)
    {
        CanMsgTransmit(
            BROADCAST_CAN_CHANNEL,
            frame.data,
            frame.dlc,
            0U,
            frame.is_extended,
            frame.id
        );
    }

    /* ================= EC ================= */

    
    if(EC_GetNextPendingCanFrame(&frame) != 0U)
    {
        CanMsgTransmit(
            BROADCAST_CAN_CHANNEL,
            frame.data,
            frame.dlc,
            0U,
            frame.is_extended,
            frame.id
        );
    }

    /* ================= EB ================= */

    if(EB_GetNextPendingCanFrame(&frame) != 0U)
    {
        CanMsgTransmit(
            BROADCAST_CAN_CHANNEL,
            frame.data,
            frame.dlc,
            0U,
            frame.is_extended,
            frame.id
        );
    }
}

 static void Send_Repeat_Error_Frames(void)
{
    EV_CanFrame_t frame;
    uint8_t i;

    /* ================= EV ================= */

    while(EV_GetNextPendingCanFrame(&frame) != 0U)
    {
        CanMsgTransmit(
            BROADCAST_CAN_CHANNEL,
            frame.data,
            frame.dlc,
            0U,
            frame.is_extended,
            frame.id
        );
    }

    /* ================= EC ================= */

    for(i = 0U; i < CTRL_COUNT; i++)
    {
        if(EC_GetNextPendingCanFrame(&frame) == 0U)
        {
            break;
        }

        CanMsgTransmit(
            BROADCAST_CAN_CHANNEL,
            frame.data,
            frame.dlc,
            0U,
            frame.is_extended,
            frame.id
        );
    }

     /*================= EB ================= */

    for(i = 0U; i < 10U; i++)
    {
        if(EB_GetNextPendingCanFrame(&frame) == 0U)
        {
            break;
        }

        CanMsgTransmit(
           BROADCAST_CAN_CHANNEL,
            frame.data,
            frame.dlc,
            0U,
            frame.is_extended,
            frame.id
        );
    }
} 

/* ================= Controller Telemetry TX ================= */

void Controller_Data_Tx(void)
{
    uint8_t buf[8];

    /* LEFT WHEEL (0x800) */
    memset(buf, 0, sizeof(buf));
    if (CTRL_IS_ALIVE(CTRL_WHEEL_LEFT))
    {
        U16_TO_CAN_LE(buf, 0, VS_Controller[CTRL_WHEEL_LEFT].speed_rpm);
        U16_TO_CAN_LE(buf, 2, (uint16_t)VS_Controller[CTRL_WHEEL_LEFT].motor_current_A);
        buf[4] = Clamp_U8_From_S16(VS_Controller[CTRL_WHEEL_LEFT].controller_temp_C);
        buf[5] = VS_Controller[CTRL_WHEEL_LEFT].feedback_dir;
        U16_TO_CAN_LE(buf, 6, VS_Controller[CTRL_WHEEL_LEFT].error_code);
    }
    CanMsgTransmit(CAN_0, buf, 8, 0, 0, 0x200);

    /* RIGHT WHEEL (0x201) */
    memset(buf, 0, sizeof(buf));
    if (CTRL_IS_ALIVE(CTRL_WHEEL_RIGHT))
    {
        U16_TO_CAN_LE(buf, 0, VS_Controller[CTRL_WHEEL_RIGHT].speed_rpm);
        U16_TO_CAN_LE(buf, 2, (uint16_t)VS_Controller[CTRL_WHEEL_RIGHT].motor_current_A);
        buf[4] = Clamp_U8_From_S16(VS_Controller[CTRL_WHEEL_RIGHT].controller_temp_C);
        buf[5] = VS_Controller[CTRL_WHEEL_RIGHT].feedback_dir;
        U16_TO_CAN_LE(buf, 6, VS_Controller[CTRL_WHEEL_RIGHT].error_code);
    }
    CanMsgTransmit(CAN_0, buf, 8, 0, 0, 0x201);

    /* ROTOR (0x202) */
    memset(buf, 0, sizeof(buf));
    if (CTRL_IS_ALIVE(CTRL_ROTOR))
    {
        U16_TO_CAN_LE(buf, 0, VS_Controller[CTRL_ROTOR].speed_rpm);
        U16_TO_CAN_LE(buf, 2, (uint16_t)VS_Controller[CTRL_ROTOR].motor_current_A);
        buf[4] = Clamp_U8_From_S16(VS_Controller[CTRL_ROTOR].controller_temp_C);
        buf[5] = VS_Controller[CTRL_ROTOR].feedback_dir;
        U16_TO_CAN_LE(buf, 6, VS_Controller[CTRL_ROTOR].error_code);
    }
    CanMsgTransmit(CAN_0, buf, 8, 0, 0, 0x202);
}

/* ================= Public Functions ================= */

void Broadcast_Init(void)
{
    Broadcast_1sCounter = 0U;
    Broadcast_60sCounter = 0U;
    Broadcast_StartupDone = 0U;
    Broadcast_60sPending = 0U;
    Broadcast_StartupFrameTxCount = 0U;
    BROADCAST_STARTUP_COUNTER = STARTUP_FRAME_PERIOD_TICKS;
    EC_Init();
    Update_Display_Snapshot();
}

void Broadcast_Task_100ms(void)

{
   static uint16_t error_repeat_counter = 0U; 
    EC_Update(100U);
    Broadcast_SendRotorDcBus();
    Broadcast_SendRotorDqStatus();

   error_repeat_counter++;
    if (error_repeat_counter >= 1200U)    //120 sec 
    {
        error_repeat_counter = 0U;
        Send_Repeat_Error_Frames();
    }

    if (Broadcast_StartupDone == 0U)
    {
        if (BROADCAST_STARTUP_COUNTER > 0U)
        {
            BROADCAST_STARTUP_COUNTER--;
        }

        if (BROADCAST_STARTUP_COUNTER == 0U)
        {
            Broadcast_SendStartupFrames();
            Broadcast_StartupFrameTxCount++;

            if (Broadcast_StartupFrameTxCount >= STARTUP_FRAME_TX_LIMIT)
            {
                Broadcast_StartupDone = 1U;
            }
            else
            {
                BROADCAST_STARTUP_COUNTER = STARTUP_FRAME_PERIOD_TICKS;
            }
            
            return;
        }
    }

    if (Broadcast_60sPending != 0U)
    {
        Broadcast_Send60sFrames();
        Broadcast_60sPending = 0U;
        return;
    }

    Broadcast_1sCounter++;
    Broadcast_60sCounter++;

    if (Broadcast_1sCounter >= BROADCAST_1S_TICKS)
    {
        Broadcast_1sCounter = 0U;
        
        Broadcast_Send1sFrames();
        //Controller_Data_Tx();
    }

    if (Broadcast_60sCounter >= BROADCAST_60S_TICKS)
    {
        Broadcast_60sCounter = 0U;
        Broadcast_60sPending = 1U;
    }
        /* normal cycle also send newly generated error frames */
    Send_Display_();
    Send_Pending_Error_Frames();
    Gpio_left_right();
} 
