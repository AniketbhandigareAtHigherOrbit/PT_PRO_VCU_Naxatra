#include "dashboard_can_rx.h"

volatile DashboardDisplayVersion_t VS_DisplayVersion;

void Dashboard_CAN_RxCallback(uint8_t CanNum_u8,
                              uint8_t Channel_u8,
                              uint8_t *DataBuff_pu8,
                              uint8_t Dlc_u8,
                              uint8_t CanMode_u8,
                              uint8_t Ide_u8,
                              uint32_t CanId_u32)
{
    (void)CanNum_u8;
    (void)Channel_u8;
    (void)CanMode_u8;
    (void)Ide_u8;

    if ((DataBuff_pu8 == 0) ||
        (CanId_u32 != DASHBOARD_CANID_DISPLAY_VERSION) ||
        (Dlc_u8 < 6U))
    {
        return;
    }

    VS_DisplayVersion.major = DataBuff_pu8[0];
    VS_DisplayVersion.minor = DataBuff_pu8[1];
    VS_DisplayVersion.patch1 = DataBuff_pu8[2];
    VS_DisplayVersion.patch2 = DataBuff_pu8[3];
    VS_DisplayVersion.build1 = DataBuff_pu8[4];
    VS_DisplayVersion.build2 = DataBuff_pu8[5];
    VS_DisplayVersion.received = 1U;
}

uint8_t Dashboard_CAN_GetDisplayVersion(DashboardDisplayVersion_t *version)
{
    if (version == 0)
    {
        return 0U;
    }

    version->major = VS_DisplayVersion.major;
    version->minor = VS_DisplayVersion.minor;
    version->patch1 = VS_DisplayVersion.patch1;
    version->patch2 = VS_DisplayVersion.patch2;
    version->build1 = VS_DisplayVersion.build1;
    version->build2 = VS_DisplayVersion.build2;
    version->received = VS_DisplayVersion.received;

    return VS_DisplayVersion.received;
}

void Dashboard_CAN_ClearDisplayVersion(void)
{
    VS_DisplayVersion.received = 0U;
}