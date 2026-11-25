#ifndef DASHBOARD_CAN_RX_H
#define DASHBOARD_CAN_RX_H

#include <stdint.h>

#include "vehicle_state.h"

#define DASHBOARD_CANID_DISPLAY_VERSION 0x500U

typedef struct
{
    uint8_t major;
    uint8_t minor;
    uint8_t patch1;
    uint8_t patch2;
    uint8_t build1;
    uint8_t build2;
    uint8_t received;
} DashboardDisplayVersion_t;

extern volatile DashboardDisplayVersion_t VS_DisplayVersion;

void Dashboard_CAN_RxCallback(uint8_t CanNum_u8,
                              uint8_t Channel_u8,
                              uint8_t *DataBuff_pu8,
                              uint8_t Dlc_u8,
                              uint8_t CanMode_u8,
                              uint8_t Ide_u8,
                              uint32_t CanId_u32);

uint8_t Dashboard_CAN_GetDisplayVersion(
    DashboardDisplayVersion_t *version);

void Dashboard_CAN_ClearDisplayVersion(void);

#endif /* DASHBOARD_CAN_RX_H */