/*******************************************************************************
|-------------------------------------------------------------------------------
|   FILE DESCRIPTION
|-------------------------------------------------------------------------------
|    File           : pal_can_conf.c
|    Project        : VCU_ASW
|    Description    : The file contains the configration data of CAN module
|-------------------------------------------------------------------------------
|
|-------------------------------------------------------------------------------
|               A U T H O R   I D E N T I T Y
|-------------------------------------------------------------------------------
|   Date              Name                        Company
| ----------     ---------------     -----------------------------------
| 08/12/2023       Jeevan Jestin N             Sloki Software Technologies Pvt Ltd
|-------------------------------------------------------------------------------
|******************************************************************************/

/*******************************************************************************
 *  HEADER FILE INCLUDES
 ******************************************************************************/
#include"pal_can_conf.h"
#include"AppTest.h"
#include"controller_can_rx.h"
#include"battery_can_rx.h"
#include"vehicle_config.h"
#include "NVM_config.h"
#include "dashboard_can_rx.h"
/*******************************************************************************
 *  MACRO DEFINITION
 ******************************************************************************/

/*******************************************************************************
 *  GLOBAL VARIABLES DEFNITION 
 ******************************************************************************/
#define TOTAL_RX_FILTER_CAN_0       11
#if (VC_BATTERY_PREFIX_FILTER_ENABLE == 1U)
#define TOTAL_RX_FILTER_CAN_1       15
#else
#define TOTAL_RX_FILTER_CAN_1       19
#endif
#define TOTAL_RX_FILTER_CAN_2       2



const uint8_t Can0_Rx_filter_u8 = 3;
const uint8_t Can1_Rx_filter_u8 = TOTAL_RX_FILTER_CAN_1;
const uint8_t Can2_Rx_filter_u8 = 2;


 
/*
 * Callback prototypes in controller_can_rx.h / battery_can_rx.h include CAN bus
 * number also, while the PAL filter table expects CanRxCallback_Fptr_t.
 * This cast keeps the same callback functions and removes W0520144 warnings
 * from the filter table initialization.
 */
#define CAN_RX_CB(callback_fn)   ((CanRxCallback_Fptr_t)(callback_fn))

void CAN1_UpdateBatteryRxFiltersFromNvm(void)
{
#if (VC_BATTERY_PREFIX_FILTER_ENABLE != 1U)
    const NVM_BatteryCfg_t *cfg = NVM_BatteryCfg_Get();

    Can1RxFilterConf_aSt[5].CanId_u32  = cfg->s1_btry1_id;
    Can1RxFilterConf_aSt[6].CanId_u32  = cfg->s1_btry2_id;
    Can1RxFilterConf_aSt[7].CanId_u32  = cfg->s2_btry1_id;
    Can1RxFilterConf_aSt[8].CanId_u32  = cfg->s2_btry2_id;
    Can1RxFilterConf_aSt[9].CanId_u32  = cfg->s3_btry1_id;
    Can1RxFilterConf_aSt[10].CanId_u32 = cfg->s3_btry2_id;
    Can1RxFilterConf_aSt[11].CanId_u32 = cfg->s4_btry1_id;
    Can1RxFilterConf_aSt[12].CanId_u32 = cfg->s4_btry2_id;
#endif
}


// CanRxFilterConf_St_t Can0RxFilterConf_aSt[];
// CanRxFilterConf_St_t Can1RxFilterConf_aSt[];
// CanRxFilterConf_St_t Can2RxFilterConf_aSt[];
/*******************************************************************************
 *  STRUCTURE AND ENUM DEFNITION 
 ****************************   **************************************************/
CanRxFilterConf_St_t Can0RxFilterConf_aSt[TOTAL_RX_FILTER_CAN_0] = 
{


    {0x0CF11E06, 0x1FFFFFFF, RX_FIFO_BUFF_E, EXT_IDE_E, CAN_RX_CB(Controller_CAN_RxCallback)},
    {0x0CF11F06, 0x1FFFFFFF, RX_FIFO_BUFF_E, EXT_IDE_E, CAN_RX_CB(Controller_CAN_RxCallback)},
    {0x500, 0x7FF, RX_FIFO_BUFF_E, STD_IDE_E, CAN_RX_CB(Dashboard_CAN_RxCallback)},


};

CanRxFilterConf_St_t Can1RxFilterConf_aSt[TOTAL_RX_FILTER_CAN_1] =  
{
    {0x200,0x7FF,RX_FIFO_BUFF_E,STD_IDE_E,NULL},   
    {0x7F0,0x7F1,RX_FIFO_BUFF_E,STD_IDE_E,NULL},
    {0x7DF,0x7FF,RX_FIFO_BUFF_E,STD_IDE_E,NULL},
    
    {0x530,0x7FF,RX_FIFO_BUFF_E,STD_IDE_E,NVM_BatteryCfg_RxCallback},
    {0x531,0x7FF,RX_FIFO_BUFF_E,STD_IDE_E,NVM_BatteryCfg_RxCallback},

#if (VC_BATTERY_PREFIX_FILTER_ENABLE == 1U)
    {VC_BATTERY_PREFIX_ID_BASE1, VC_BATTERY_PREFIX_ID_MASK, RX_FIFO_BUFF_E, EXT_IDE_E, CAN_RX_CB(Call_Back_Battery_Rx)},
    {VC_BATTERY_PREFIX_ID_BASE2, VC_BATTERY_PREFIX_ID_MASK, RX_FIFO_BUFF_E, EXT_IDE_E, CAN_RX_CB(Call_Back_Battery_Rx)},
    {VC_BATTERY_PREFIX_ID_BASE3, VC_BATTERY_PREFIX_ID_MASK, RX_FIFO_BUFF_E, EXT_IDE_E, CAN_RX_CB(Call_Back_Battery_Rx)},
    {VC_BATTERY_PREFIX_ID_BASE4, VC_BATTERY_PREFIX_ID_MASK, RX_FIFO_BUFF_E, EXT_IDE_E, CAN_RX_CB(Call_Back_Battery_Rx)},
#else
    {VC_S1_BTRY1_ID_DEFAULT, 0x00000000, RX_FIFO_BUFF_E, EXT_IDE_E, CAN_RX_CB(Call_Back_Battery_Rx)},
    {VC_S1_BTRY2_ID_DEFAULT, 0x00000000, RX_FIFO_BUFF_E, EXT_IDE_E, CAN_RX_CB(Call_Back_Battery_Rx)},
    {VC_S2_BTRY1_ID_DEFAULT, 0x00000000, RX_FIFO_BUFF_E, EXT_IDE_E, CAN_RX_CB(Call_Back_Battery_Rx)},
    {VC_S2_BTRY2_ID_DEFAULT, 0x00000000, RX_FIFO_BUFF_E, EXT_IDE_E, CAN_RX_CB(Call_Back_Battery_Rx)},
    {VC_S3_BTRY1_ID_DEFAULT, 0x00000000, RX_FIFO_BUFF_E, EXT_IDE_E, CAN_RX_CB(Call_Back_Battery_Rx)},
    {VC_S3_BTRY2_ID_DEFAULT, 0x00000000, RX_FIFO_BUFF_E, EXT_IDE_E, CAN_RX_CB(Call_Back_Battery_Rx)},
    {VC_S4_BTRY1_ID_DEFAULT, 0x00000000, RX_FIFO_BUFF_E, EXT_IDE_E, CAN_RX_CB(Call_Back_Battery_Rx)},
    {VC_S4_BTRY2_ID_DEFAULT, 0x00000000, RX_FIFO_BUFF_E, EXT_IDE_E, CAN_RX_CB(Call_Back_Battery_Rx)},
#endif
    {0x411, 0x7FF, RX_FIFO_BUFF_E, STD_IDE_E, CAN_RX_CB(Controller_CAN_RxCallback)},
    {0x412, 0x7FF, RX_FIFO_BUFF_E, STD_IDE_E, CAN_RX_CB(Controller_CAN_RxCallback)},

    {0x0CF11E06, 0x1FFFFFFF, RX_FIFO_BUFF_E, EXT_IDE_E, CAN_RX_CB(Controller_CAN_RxCallback)},
    {0x0CF11F06, 0x1FFFFFFF, RX_FIFO_BUFF_E, EXT_IDE_E, CAN_RX_CB(Controller_CAN_RxCallback)},

    {0x0CF11E04, 0x1FFFFFFF, RX_FIFO_BUFF_E, EXT_IDE_E, CAN_RX_CB(Controller_CAN_RxCallback)},
    {0x0CF11F04, 0x1FFFFFFF, RX_FIFO_BUFF_E, EXT_IDE_E, CAN_RX_CB(Controller_CAN_RxCallback)},


};

CanRxFilterConf_St_t Can2RxFilterConf_aSt[TOTAL_RX_FILTER_CAN_2] = 
{
    {0x0CF11E04, 0x1FFFFFFF, RX_FIFO_BUFF_E, EXT_IDE_E, CAN_RX_CB(Controller_CAN_RxCallback)},
    {0x0CF11F04, 0x1FFFFFFF, RX_FIFO_BUFF_E, EXT_IDE_E, CAN_RX_CB(Controller_CAN_RxCallback)},

};
  

RSCANchannelConf_St_t RSCANchannelConf_aSt[] = 
{
    /* Note:
            CAN configuration channel number should be in sequential order 
    */
    {true,CAN_FD_MODE_E,CAN_0,TOTAL_RX_FILTER_CAN_0,Can0RxFilterConf_aSt},
    {true,CAN_FD_MODE_E,CAN_1,TOTAL_RX_FILTER_CAN_1,Can1RxFilterConf_aSt},
    {true,CAN_FD_MODE_E,CAN_2,TOTAL_RX_FILTER_CAN_2,Can2RxFilterConf_aSt},
};

RSCANBaudconfg_St_t RSCANbaudconfg_aSt[] =
{
    /* Note:
            CAN Baudrate configuration should be in sequential order 
    */
    {_500K_BAUDRATE_E,_2MBPS_BAUDRATE_E},
    {_500K_BAUDRATE_E,_2MBPS_BAUDRATE_E},
    {_500K_BAUDRATE_E,_2MBPS_BAUDRATE_E},
};

/*******************************************************************************
 *  STATIC FUNCTION PROTOTYPES
 ******************************************************************************/

/*******************************************************************************
 *  FUNCTION DEFINITIONS
 ******************************************************************************/

/* -----------------------------------------------------------------------------
*  FUNCTION DESCRIPTION
*  -----------------------------------------------------------------------------
*   Function Name : NONE
*   Description   : NONE
*   Parameters    : None
*   Return Value  : None
*  ---------------------------------------------------------------------------*/

/*---------------------- End of File -----------------------------------------*/
