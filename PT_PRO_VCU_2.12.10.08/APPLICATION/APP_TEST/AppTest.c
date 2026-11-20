#include "AppTest.h"
#include "databank.h"
#include "pal_can_if.h"
/* #include "eeprom.h" */
/* #include "nvm_userinterface.h" */
#include "vehicle_state.h"
#include "vehicle_config.h"
#include "machine_drive_logic.h"
#include "controller_can_rx.h"
#include "naxatra_can.h"
#include <string.h>

/* ================= Helpers / Macros ================= */
#define VC_CAN_TASK_PERIOD_MS              20U
#define VC_ACTIVE_BRK_LOW_RPM_TIME_MS      7000U
#define VC_ACTIVE_BRK_FEEDBACK_RPM_TH      10U

/* If both wheels show feedback, confirm faster */
#define VC_ACTIVE_BRK_FEEDBACK_BOTH_CONFIRM_MS   300U

/* If only one wheel shows feedback, confirm slower */
#define VC_ACTIVE_BRK_FEEDBACK_SINGLE_CONFIRM_MS 500U

/* Once movement is confirmed, apply low RPM braking for this time */
#define VC_ACTIVE_BRK_FEEDBACK_HOLD_MS           800U

uint8_t brake_status = 0U;
static uint16_t VS_BrakeFeedbackSinglePresentMs = 0U;
static uint16_t VS_BrakeFeedbackBothPresentMs = 0U;
static uint16_t VS_BrakeFeedbackHoldMs = 0U;

#define MAKE_CAN_ID(dev_id) \
    ((VC_BASE_CAN_ID & 0xFFFF00FFUL) | ((uint32_t)(dev_id) << 8))

#define CTRL_IS_ALIVE(idx) (VS_Controller[(idx)].alive)

#define U16_TO_CAN_LE(buf, idx, val)                                  \
    do {                                                              \
        (buf)[(idx)]     = (uint8_t)((uint16_t)(val) & 0xFFU);        \
        (buf)[(idx) + 1] = (uint8_t)(((uint16_t)(val) >> 8) & 0xFFU); \
    } while (0)

/* =====================================================
 * Status Byte
 *
 * direction:
 * 0 = STOP
 * 1 = FORWARD
 * 2 = REVERSE
 *
 * Bit 0-1 = direction
 * Bit 2   = brake enable
 *
 * Result:
 * STOP + brake OFF     = 0x00
 * FWD  + brake OFF     = 0x01
 * REV  + brake OFF     = 0x02
 * STOP + brake ON      = 0x04
 * FWD  + brake ON      = 0x05
 * REV  + brake ON      = 0x06
 * ===================================================== */

 static uint8_t Build_Status_Byte(uint8_t direction, uint8_t brake_enable)
{
    uint8_t status = 0U;

    /*
     * Bit 0-1 = direction
     * 0 = STOP
     * 1 = FORWARD
     * 2 = REVERSE
     *
     * Bit 2 = brake enable
     *
     * DIR_STOP + brake ON = 0x04
     */
    status |= (uint8_t)(direction & 0x03U);

    if (brake_enable != 0U)
    {
        status |= (uint8_t)(1U << 2);
    }

    return status;
}

static uint8_t Wheel_Feedback_Is_Moving(void)
{
    int32_t left_fb;
    int32_t right_fb;

    uint8_t left_moving;
    uint8_t right_moving;

    left_fb  = (int32_t)VS_Controller[CTRL_WHEEL_LEFT].speed_rpm;
    right_fb = (int32_t)VS_Controller[CTRL_WHEEL_RIGHT].speed_rpm;

    if (left_fb < 0)
    {
        left_fb = -left_fb;
    }

    if (right_fb < 0)
    {
        right_fb = -right_fb;
    }

    left_moving = 0U;
    right_moving = 0U;

    if ((CTRL_IS_ALIVE(CTRL_WHEEL_LEFT) != 0U) &&
        ((uint32_t)left_fb > VC_ACTIVE_BRK_FEEDBACK_RPM_TH))
    {
        left_moving = 1U;
    }

    if ((CTRL_IS_ALIVE(CTRL_WHEEL_RIGHT) != 0U) &&
        ((uint32_t)right_fb > VC_ACTIVE_BRK_FEEDBACK_RPM_TH))
    {
        right_moving = 1U;
    }

    /*
     * Both wheels moving:
     * Confirm faster.
     */
    if ((left_moving != 0U) && (right_moving != 0U))
    {
        VS_BrakeFeedbackSinglePresentMs = 0U;

        if (VS_BrakeFeedbackBothPresentMs < VC_ACTIVE_BRK_FEEDBACK_BOTH_CONFIRM_MS)
        {
            VS_BrakeFeedbackBothPresentMs =
                (uint16_t)(VS_BrakeFeedbackBothPresentMs + VC_CAN_TASK_PERIOD_MS);
        }

        if (VS_BrakeFeedbackBothPresentMs >= VC_ACTIVE_BRK_FEEDBACK_BOTH_CONFIRM_MS)
        {
            VS_BrakeFeedbackHoldMs = VC_ACTIVE_BRK_FEEDBACK_HOLD_MS;
        }
    }
    /*
     * Only one wheel moving:
     * Confirm slower.
     */
    else if ((left_moving != 0U) || (right_moving != 0U))
    {
        VS_BrakeFeedbackBothPresentMs = 0U;

        if (VS_BrakeFeedbackSinglePresentMs < VC_ACTIVE_BRK_FEEDBACK_SINGLE_CONFIRM_MS)
        {
            VS_BrakeFeedbackSinglePresentMs =
                (uint16_t)(VS_BrakeFeedbackSinglePresentMs + VC_CAN_TASK_PERIOD_MS);
        }

        if (VS_BrakeFeedbackSinglePresentMs >= VC_ACTIVE_BRK_FEEDBACK_SINGLE_CONFIRM_MS)
        {
            VS_BrakeFeedbackHoldMs = VC_ACTIVE_BRK_FEEDBACK_HOLD_MS;
        }
    }
    /*
     * No wheel feedback:
     * Reset confirmation timers.
     */
    else
    {
        VS_BrakeFeedbackSinglePresentMs = 0U;
        VS_BrakeFeedbackBothPresentMs = 0U;
    }

    /*
     * Hold low-RPM braking for fixed time once movement is confirmed.
     */
    if (VS_BrakeFeedbackHoldMs > 0U)
    {
        if (VS_BrakeFeedbackHoldMs > VC_CAN_TASK_PERIOD_MS)
        {
            VS_BrakeFeedbackHoldMs =
                (uint16_t)(VS_BrakeFeedbackHoldMs - VC_CAN_TASK_PERIOD_MS);
        }
        else
        {
            VS_BrakeFeedbackHoldMs = 0U;
        }

        return 1U;
    }

    return 0U;
}

static void Wheel_Feedback_Moving_Reset(void)
{
    VS_BrakeFeedbackSinglePresentMs = 0U;
    VS_BrakeFeedbackBothPresentMs = 0U;
    VS_BrakeFeedbackHoldMs = 0U;
}
/* =====================================================
 * Opposite-direction braking
 *
 * Machine running FORWARD -> brake in REVERSE direction
 * Machine running REVERSE -> brake in FORWARD direction
 * ===================================================== 

 static uint8_t Get_Opposite_Brake_Direction(uint8_t running_direction)
{
    if (running_direction == DIR_FORWARD)
    {
        return DIR_REVERSE;
    }
    else if (running_direction == DIR_REVERSE)
    {
        return DIR_FORWARD;
    }
    else
    {
        return DIR_STOP;
    }
}
*/
static void Send_RPM_To_LeftController(uint8_t device_id,
                                       uint16_t rpm,
                                       uint8_t enable,
                                       uint8_t direction,
                                       uint8_t brake_enable)
{
    uint8_t CanData[8];

    memset(CanData, 0, sizeof(CanData));

    CanData[0] = (uint8_t)(rpm & 0xFFU);
    CanData[1] = (uint8_t)((rpm >> 8) & 0xFFU);
    CanData[2] = (enable != 0U) ? 1U : 0U;
    //CanData[3] = Build_Status_Byte(direction, brake_enable);
    CanData[3] = Build_Status_Byte(direction, brake_enable);
    CanMsgTransmit(
        CAN_2,
        CanData,
        8,
        0,
        1,
        MAKE_CAN_ID(device_id)
    );
}

static void Send_RPM_To_RightController(uint8_t device_id,
                                        uint16_t rpm,
                                        uint8_t enable,
                                        uint8_t direction,
                                        uint8_t brake_enable)
{
    uint8_t CanData[8];

    memset(CanData, 0, sizeof(CanData));

    CanData[0] = (uint8_t)(rpm & 0xFFU);
    CanData[1] = (uint8_t)((rpm >> 8) & 0xFFU);
    CanData[2] = (enable != 0U) ? 1U : 0U;
    //CanData[3] = Build_Status_Byte(direction, brake_enable);
    CanData[3] = Build_Status_Byte(direction, brake_enable);

    CanMsgTransmit(
        CAN_0,
        CanData,
        8,
        0,
        1,
        MAKE_CAN_ID(device_id)
    );
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

    static uint16_t kill_brake_timer_ms = 0U;
    
    static uint16_t feedback_brake_timer_ms = 0U;
    static uint8_t latched_drive_direction = DIR_STOP;
    static uint8_t counter_1200ms = 0U;
    // wheel_moving;
    uint8_t brake_cmd_direction;
    uint8_t wheel_cmd_direction;
    uint16_t left_rpm_command;
    uint16_t right_rpm_command;
   // uint8_t  requested_dir = (VS_Direction_Request == 0U) ? DIR_FORWARD : DIR_REVERSE;

    /*
     * =====================================================
     * KILL SWITCH RELEASED / SAFETY LEVER OFF
     * =====================================================
     */

    /*
     * Remember the last valid direction while the lever is pressed.
     * Direction-switch changes while the lever is released must not
     * change the low-RPM braking direction.
     */
    if (VS_KillSwitch != 0U)
    {   
        
        if (counter_1200ms >= 12U) /* 500 ms */
        {
             if ((VS_CurrentDirection == DIR_FORWARD) ||
            (VS_CurrentDirection == DIR_REVERSE))
            {
                counter_1200ms = 0U;
            latched_drive_direction = VS_CurrentDirection;
            }
        }
       
    }
    counter_1200ms++;
     
    if (VS_KillSwitch == 0U)
    {
        
        Send_RPM_To_RotController(VC_ROTOR_ID, 0U, 0U);
        brake_cmd_direction = latched_drive_direction;
        /*
         * First braking phase:
         * Send VC_ACTIVE_BRK_MIN_RPM continuously.
         * Do not allow feedback spikes to build latch during this phase.
         */
        if (kill_brake_timer_ms < VC_ACTIVE_BRK_LOW_RPM_TIME_MS)
        {
            Wheel_Feedback_Moving_Reset();

            Send_RPM_To_LeftController(VC_WHEEL_LEFT_ID,
                                       VC_ACTIVE_BRK_MIN_RPM,
                                       1U,
                                       brake_cmd_direction,
                                       0U);

            Send_RPM_To_RightController(VC_WHEEL_RIGHT_ID,
                                        VC_ACTIVE_BRK_MIN_RPM,
                                        1U,
                                        brake_cmd_direction,
                                        0U);

            kill_brake_timer_ms =
                (uint16_t)(kill_brake_timer_ms + VC_CAN_TASK_PERIOD_MS);

            return;
        }
        

        /*
         * After initial braking time:
         * Check feedback RPM with debounce.
         */
        wheel_moving = Wheel_Feedback_Is_Moving();

        if (wheel_moving != 0U)
        {
            
             if (feedback_brake_timer_ms < VC_ACTIVE_BRK_LOW_RPM_TIME_MS)
            {
                        Send_RPM_To_LeftController(VC_WHEEL_LEFT_ID,
                                       VC_ACTIVE_BRK_MIN_RPM,
                                       1U,
                                       brake_cmd_direction,
                                       0U);

                        Send_RPM_To_RightController(VC_WHEEL_RIGHT_ID,
                                                    VC_ACTIVE_BRK_MIN_RPM,
                                                    1U,
                                                    brake_cmd_direction,
                                                    0U);
                        feedback_brake_timer_ms =
                            (uint16_t)(feedback_brake_timer_ms + VC_CAN_TASK_PERIOD_MS);
            }
                 
  
        }
       
        else
        {
            /*
             * Wheels stopped:
             * Send zero RPM with brake byte.
             *
             * CanData[3] = 0x04
             */
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
        }

        return;
    }

    /*
     * =====================================================
     * SAFETY LEVER PRESSED / NORMAL RUNNING
     * =====================================================
     */
    kill_brake_timer_ms = 0U;
    
    feedback_brake_timer_ms = 0U;
    Wheel_Feedback_Moving_Reset();

    /*
     * Take one coherent command snapshot before transmitting on the two CAN
     * peripherals. Both controllers therefore receive values from the same
     * 20 ms control iteration.
     */
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

    Send_RPM_To_RotController(VC_ROTOR_ID,VS_RPM_Rotor,1U);

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
