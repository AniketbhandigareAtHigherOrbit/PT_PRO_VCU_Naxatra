#include "AppTest.h"
#include "databank.h"
/* #include "eeprom.h" */
/* #include "nvm_userinterface.h" */
#include "vehicle_state.h"
#include "vehicle_config.h"
#include "machine_drive_logic.h"
#include "controller_can_rx.h"
#include "naxatra_can.h"

/* ================= Helpers / Macros ================= */
#define VC_CAN_TASK_PERIOD_MS              20U

/* The former wheel-feedback/low-RPM active-braking state machine is disabled.
 * A kill-switch release now commands Naxatra Park at 0 RPM on all motors. */
static void Send_RPM_To_LeftController(uint8_t device_id,
                                       uint16_t rpm,
                                       uint8_t enable,
                                       uint8_t direction,
                                       uint8_t brake_enable)
{
    (void)device_id;
    Naxatra_CAN_SendCommand(CTRL_WHEEL_LEFT, rpm, enable, direction,
                            brake_enable);
}

static void Send_RPM_To_RightController(uint8_t device_id,
                                        uint16_t rpm,
                                        uint8_t enable,
                                        uint8_t direction,
                                        uint8_t brake_enable)
{
    (void)device_id;
    Naxatra_CAN_SendCommand(CTRL_WHEEL_RIGHT, rpm, enable, direction,
                            brake_enable);
}

static void Send_RPM_To_RotController(uint8_t device_id, uint16_t rpm, uint8_t enable)
{
    (void)device_id;
    Naxatra_CAN_SendCommand(CTRL_ROTOR, rpm, enable, VS_RotorDirection, 0U);
}

/* ================= Bluetooth / Test CAN Function ================= */

void CAN_functionality_BT(void)
{
    uint8_t en_left;
    uint8_t en_right;

    /* Kill switch ALWAYS wins */
    if (VS_KillSwitch == 0U)
    {
        Send_RPM_To_LeftController(VC_WHEEL_LEFT_ID,
                                   0U,
                                   1U,
                                   DIR_STOP,
                                   1U);

        Send_RPM_To_RightController(VC_WHEEL_RIGHT_ID,
                                    0U,
                                    1U,
                                    DIR_STOP,
                                    1U);

        return;
    }

    en_left  = (VS_RPM_Left  > 0U) ? 1U : 0U;
    en_right = (VS_RPM_Right > 0U) ? 1U : 0U;

    (void)en_left;
    (void)en_right;

    /* Rotor motor */
    if (VS_RotorEnable != 0U)
    {
        /* Reserved for BT rotor control */
    }
    else
    {
        /* Reserved for BT rotor stop */
    }
}

/* ================= Input Reading ================= */

void Vehicle_ReadInputs(void)
{
    /* ADC */
    VS_Throttle_Wheel = Get_ADC_Result(G3_ADC1_E);
    VS_Throttle_Rotor = Get_ADC_Result(G4_ADC2_E);

    /* GPIO */
    VS_KillSwitch = Get_GPInput_State(B2_GPINUPUT1_E);
    VS_TurnLeft   = Get_GPInput_State(C2_GPINUPUT2_E);
    VS_TurnRight  = Get_GPInput_State(D2_GPINUPUT3_E);
    VS_Direction_Request = Get_GPInput_State(E2_GPINUPUT4_E);
}

/* ================= Battery Timeout ================= */

void Battery_Timeout_Task(uint16_t task_period_ms)
{
    /* Battery 1 */
    if (VS_Battery1.RxTimeoutMs > task_period_ms)
    {
        VS_Battery1.RxTimeoutMs =
            (uint16_t)(VS_Battery1.RxTimeoutMs - task_period_ms);
    }
    else
    {
        VS_Battery1.RxTimeoutMs = 0U;
        VS_Battery1.Alive = 0U;

        VS_Battery1.Voltage_dV = 0U;
        VS_Battery1.Current_dA = 0;
        VS_Battery1.SOC        = 0U;
        VS_Battery1.SOH        = 0U;
    }

    /* Battery 2 */
    if (VS_Battery2.RxTimeoutMs > task_period_ms)
    {
        VS_Battery2.RxTimeoutMs =
            (uint16_t)(VS_Battery2.RxTimeoutMs - task_period_ms);
    }
    else
    {
        VS_Battery2.RxTimeoutMs = 0U;
        VS_Battery2.Alive = 0U;

        VS_Battery2.Voltage_dV = 0U;
        VS_Battery2.Current_dA = 0;
        VS_Battery2.SOC        = 0U;
        VS_Battery2.SOH        = 0U;
    }
}

/* ================= Main CAN Function ================= */

void CAN_functionality(void)
{
    uint8_t wheel_cmd_direction;
    uint16_t left_rpm_command;
    uint16_t right_rpm_command;

    /* Kill switch off: command zero RPM + Park on all three Naxatra
     * controllers. No low-RPM active-braking or feedback timer loop. */
    if (VS_KillSwitch == 0U)
    {
        Send_RPM_To_RotController(VC_ROTOR_ID, 0U, 0U);
        Send_RPM_To_LeftController(VC_WHEEL_LEFT_ID,
                                   0U, 1U, DIR_STOP, 1U);
        Send_RPM_To_RightController(VC_WHEEL_RIGHT_ID,
                                    0U, 1U, DIR_STOP, 1U);

        return;
    }

    /* Use one command snapshot in this 20 ms iteration for both wheels. */
    wheel_cmd_direction = VS_WheelCommandDirection;
    left_rpm_command = VS_RPM_Left;
    right_rpm_command = VS_RPM_Right;

    Send_RPM_To_LeftController(VC_WHEEL_LEFT_ID,
                               left_rpm_command,
                               1U,
                               wheel_cmd_direction,
                               0U);

    Send_RPM_To_RightController(VC_WHEEL_RIGHT_ID,
                                right_rpm_command,
                                1U,
                                wheel_cmd_direction,
                                0U);

    Send_RPM_To_RotController(VC_ROTOR_ID, VS_RPM_Rotor, 1U);
}

/* 100 ms task */

void Battery_Timeout_Task_100ms(void)
{
    Battery_Timeout_Task(100U);
}

/* ================= 10 ms Vehicle Task ================= */

void Vehicle_Task_10ms(void)
{
    /* Actual scheduler period is 20 ms in task_scheduler_conf.c. */
    Controller_CAN_TimeoutTask(VC_CAN_TASK_PERIOD_MS);
    MachineDrive_Mode_Step();
    CAN_functionality();
}
