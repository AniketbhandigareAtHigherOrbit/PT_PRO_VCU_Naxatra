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
    static uint16_t startup_ms = 0U;
    uint8_t data[8];
    uint8_t stop;

    if (index != CTRL_ROTOR) { return; }
    stop = (uint8_t)((VS_KillSwitch == 0U) || (enable == 0U) ||
                    (brake != 0U) ||
                    ((direction != DIR_FORWARD) && (direction != DIR_REVERSE)));
    if (stop != 0U) { startup_ms = 0U; }
    if (rpm > VC_ROTOR_MAX_RPM) { rpm = VC_ROTOR_MAX_RPM; }

    Naxatra_CAN_BuildCommand(data,
        (startup_ms < NAXATRA_STARTUP_ZERO_MS) ? 0U : rpm,
        (stop != 0U) ? 0U : enable, direction, brake);
    CanMsgTransmit(NAXATRA_ROTOR_CHANNEL, data, 8U, 0U, 0U,
                   NAXATRA_COMMAND_ID);

    /* Track the direction actually requested on 0x415, not the last RX
     * status bit. A zero-RPM startup command still selects its gear. */
    VS_Controller[CTRL_ROTOR].command_dir =
        (stop != 0U) ? DIR_STOP : direction;

    if ((stop == 0U) && (startup_ms < NAXATRA_STARTUP_ZERO_MS))
    {
        startup_ms += NAXATRA_TX_PERIOD_MS;
    }
}
