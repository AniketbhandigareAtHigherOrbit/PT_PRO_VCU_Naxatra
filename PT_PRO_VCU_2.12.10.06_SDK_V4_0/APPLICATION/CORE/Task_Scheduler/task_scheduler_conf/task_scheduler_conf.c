/*******************************************************************************
|-------------------------------------------------------------------------------
|   FILE DESCRIPTION
|-------------------------------------------------------------------------------
|    File           : task_scheduler_conf.c
|    Project        :  
|    Description    : The file implements .
|-------------------------------------------------------------------------------
|
|-------------------------------------------------------------------------------
|               A U T H O R   I D E N T I T Y
|-------------------------------------------------------------------------------
|   Date              Name                        Company
| ----------     ---------------     -----------------------------------
| 
|-------------------------------------------------------------------------------
|******************************************************************************/

/*******************************************************************************
 *  HEADER FILE INCLUDES
 ******************************************************************************/
#include "task_scheduler_conf.h"
#include "databank.h"
#include "ext_eeprom.h"
//#include "app_init.h"

#include "AppTest.h"


#include "adc_service.h"
#include "gpio_service.h"
#include "rpm_service.h"
//#include "on_road_logic.h"
#include "machine_drive_logic.h"
#include "vcu_sensors.h"
#include "nvm_userinterface.h"
#include "Can_Broadcast.h"
#include "NVM_config.h"
#include "error_code.h"
/*******************************************************************************
 *  MACRO DEFINITION
 ******************************************************************************/

/*******************************************************************************
 *  GLOBAL VARIABLES DEFNITION
 ******************************************************************************/

/*******************************************************************************
 *  STRUCTURE AND ENUM DEFNITION
 ******************************************************************************/
const uint8_t Total_task_u8=TOTAL_TASK;
const uint8_t Total_Init_task_u8 = TOTAL_INIT_TASK;
TaskConf_St_t TaskConf_St[TOTAL_TASK] =
{	
    //{Nvm_Scheduler,          TS_PERIODICTY(10),  INIT_VALUE},
    {Ea_Scheduler,                TS_PERIODICTY(10U),   INIT_VALUE},
    // {ReportDTCStatus        ,TS_PERIODICTY(50),  INIT_VALUE},
    {ADC_Service_Run,             TS_PERIODICTY(20U),   INIT_VALUE},
    {GPIO_Service_Run,            TS_PERIODICTY(20U),   INIT_VALUE},
    {Vehicle_Task_10ms,           TS_PERIODICTY(20U),   INIT_VALUE},
    //{VCU_Sensors_10ms,            TS_PERIODICTY(200U),  INIT_VALUE},

    {Battery_Timeout_Task_100ms,  TS_PERIODICTY(100U),  INIT_VALUE},
    {Broadcast_Task_100ms,        TS_PERIODICTY(100U),  INIT_VALUE},
    //{Controller_CAN_TimeoutTask, TS_PERIODICTY(10), INIT_VALUE},
    {NVM_BatteryCfg_Task,         TS_PERIODICTY(100U),  INIT_VALUE},
};

 InitTaskConf_St_t InitTaskConf_St[TOTAL_INIT_TASK] =
 {
    {Vehicle_Task_10ms, 0}, 
    {NVM_BatteryCfg_Init, 0},
    // {Test_GpioInput, 0},  // Initialization task for GPIO input, flag = 0
    {Broadcast_Init, 0},
    {EC_Init, 0},
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