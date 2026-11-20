#include "naxatra_can.h"
#include "vehicle_config.h"
#include "pal_can_if.h"

void Naxatra_CAN_BuildCommand(uint8_t *data, uint16_t rpm,
                             uint8_t enable, uint8_t direction, uint8_t brake)
{
    uint8_t i;
    if (data == 0) { return; }
    for (i = 0U; i < 8U; i++) { data[i] = 0U; }
    data[0] = 0U;    /* VcuThrottlePercent: unused in speed mode. */
    data[1] = 0x08U; /* VcuSoftCmdEn: CAN-controlled, other bits clear. */
    data[2] = 0x01U; /* Soft response, speed mode; cruise/regen off. */
    data[5] = NAXATRA_ACCEL_TIME_DS;
    data[6] = NAXATRA_DECEL_TIME_DS;
    data[7] = NAXATRA_DC_FDB_CURRENT_RAW;
    if ((enable == 0U) || (brake != 0U) ||
        ((direction != DIR_FORWARD) && (direction != DIR_REVERSE)))
    {
        data[1] |= 0x04U; /* Park, zero speed, brake command. */
        return;
    }
    data[1] |= 0x20U; /* First gear. */
    if (direction == DIR_REVERSE) { data[1] |= 0x10U; }
    data[3] = (uint8_t)(rpm >> 8U);
    data[4] = (uint8_t)(rpm & 0xFFU);
}

/* Called directly once per 20 ms drive iteration; no buffered command. */
void Naxatra_CAN_SendCommand(ControllerIndex_t index, uint16_t rpm,
                            uint8_t enable, uint8_t direction, uint8_t brake)
{
    static uint16_t startup_ms[CTRL_COUNT] = {0U};
    uint8_t data[8];
    uint8_t stop;
    uint8_t channel;
    uint32_t command_id;
    uint16_t max_rpm;

    switch (index)
    {
        case CTRL_WHEEL_LEFT:
            channel = NAXATRA_LEFT_CHANNEL;
            command_id = NAXATRA_COMMAND_ID_LFT;
            max_rpm = VC_WHEEL_MAX_RPM;
            break;
        case CTRL_WHEEL_RIGHT:
            channel = NAXATRA_RIGHT_CHANNEL;
            command_id = NAXATRA_COMMAND_ID_RIG;
            max_rpm = VC_WHEEL_MAX_RPM;
            break;
        case CTRL_ROTOR:
            channel = NAXATRA_ROTOR_CHANNEL;
            command_id = NAXATRA_COMMAND_ID_ROT;
            max_rpm = VC_ROTOR_MAX_RPM;
            break;
        default:
            return;
    }

    stop = (uint8_t)((VS_KillSwitch == 0U) || (enable == 0U) ||
                    (brake != 0U) ||
                    ((direction != DIR_FORWARD) && (direction != DIR_REVERSE)));
    if (stop != 0U) { startup_ms[index] = 0U; }
    if (rpm > max_rpm) { rpm = max_rpm; }

    Naxatra_CAN_BuildCommand(data,
        (startup_ms[index] < NAXATRA_STARTUP_ZERO_MS) ? 0U : rpm,
        (stop != 0U) ? 0U : enable, direction, brake);
    CanMsgTransmit(channel, data, 8U, 0U, 0U, command_id);

    /* Track the requested direction per controller. A zero-RPM startup
     * command still selects its requested gear. */
    VS_Controller[index].command_dir =
        (stop != 0U) ? DIR_STOP : direction;

    if ((stop == 0U) && (startup_ms[index] < NAXATRA_STARTUP_ZERO_MS))
    {
        startup_ms[index] += NAXATRA_TX_PERIOD_MS;
    }
}
