/*******************************************************************************
|-------------------------------------------------------------------------------
|   FILE DESCRIPTION
|-------------------------------------------------------------------------------
|    File           : pal_cancallback.c
|    Project        : VCU SDK
|    Description    : The file contains the callback function for the CAN module.
|-------------------------------------------------------------------------------
|
|-------------------------------------------------------------------------------
|               A U T H O R   I D E N T I T Y
|-------------------------------------------------------------------------------
|   Date              Name                        Company
| ----------     ---------------     -----------------------------------
| 13/12/2023       Jeevan Jestin N             Sloki Software Technologies LLP
|-------------------------------------------------------------------------------
|******************************************************************************/

/*******************************************************************************
 *  HEADER FILE INCLUDES
 ******************************************************************************/
#include "pal_cancallback.h"
#include "can_callback.h"

/*******************************************************************************
 *  FUNCTION DEFINITIONS
 ******************************************************************************/

/* -----------------------------------------------------------------------------
*  FUNCTION DESCRIPTION
*  -----------------------------------------------------------------------------
*   Function Name : CanMsgReceiveCallback
*   Description   : Dispatches received CAN messages to configured callbacks
*                   matching CAN ID against configured MaskId_u32.
*   Parameters    : CanNum_u8, DataBuff_pu8, Dlc_u8, CanMode_u8, Ide_u8, CanId_u32
*   Return Value  : None
*  ---------------------------------------------------------------------------*/
void CanMsgReceiveCallback(uint8_t CanNum_u8,
                           uint8_t *DataBuff_pu8,
                           uint8_t Dlc_u8,
                           uint8_t CanMode_u8,
                           uint8_t Ide_u8,
                           uint32_t CanId_u32)
{
    uint8_t i;

    switch (CanNum_u8)
    {
        case CAN_0:
            for (i = 0U; i < Can0_Rx_filter_u8; i++)
            {
                if ((CanId_u32 & Can0RxFilterConf_aSt[i].MaskId_u32) ==
                    (Can0RxFilterConf_aSt[i].CanId_u32 & Can0RxFilterConf_aSt[i].MaskId_u32))
                {
                    if (Can0RxFilterConf_aSt[i].CanRxCallback_Fptr != NULL)
                    {
                        Can0RxFilterConf_aSt[i].CanRxCallback_Fptr(UNIT_0,
                                                                   CanNum_u8,
                                                                   DataBuff_pu8,
                                                                   Dlc_u8,
                                                                   CanMode_u8,
                                                                   Ide_u8,
                                                                   CanId_u32);
                    }
                }
            }
            break;

        case CAN_1:
            for (i = 0U; i < Can1_Rx_filter_u8; i++)
            {
                if ((CanId_u32 & Can1RxFilterConf_aSt[i].MaskId_u32) ==
                    (Can1RxFilterConf_aSt[i].CanId_u32 & Can1RxFilterConf_aSt[i].MaskId_u32))
                {
                    if (Can1RxFilterConf_aSt[i].CanRxCallback_Fptr != NULL)
                    {
                        Can1RxFilterConf_aSt[i].CanRxCallback_Fptr(UNIT_0,
                                                                   CanNum_u8,
                                                                   DataBuff_pu8,
                                                                   Dlc_u8,
                                                                   CanMode_u8,
                                                                   Ide_u8,
                                                                   CanId_u32);
                    }
                }
            }
            break;

        case CAN_2:
            for (i = 0U; i < Can2_Rx_filter_u8; i++)
            {
                if ((CanId_u32 & Can2RxFilterConf_aSt[i].MaskId_u32) ==
                    (Can2RxFilterConf_aSt[i].CanId_u32 & Can2RxFilterConf_aSt[i].MaskId_u32))
                {
                    if (Can2RxFilterConf_aSt[i].CanRxCallback_Fptr != NULL)
                    {
                        Can2RxFilterConf_aSt[i].CanRxCallback_Fptr(UNIT_0,
                                                                   CanNum_u8,
                                                                   DataBuff_pu8,
                                                                   Dlc_u8,
                                                                   CanMode_u8,
                                                                   Ide_u8,
                                                                   CanId_u32);
                    }
                }
            }
            break;

        default:
            break;
    }

    DRV_HAL2CIL_CallBack(Ide_u8, Dlc_u8, CanNum_u8, CanId_u32, DataBuff_pu8);
}

/*---------------------- End of File -----------------------------------------*/
