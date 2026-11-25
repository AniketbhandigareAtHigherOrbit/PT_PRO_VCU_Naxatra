#include "NVM_config.h"

#include "nvm_userinterface.h"
#include "pal_can_if.h"
#include "vehicle_config.h"
#include "battery_can_rx.h"

#define NVM_BATTERY_CAN_CH              CAN_1

typedef enum
{
    NVM_BATTERY_STATE_IDLE = 0,
    NVM_BATTERY_STATE_LOAD,
    NVM_BATTERY_STATE_VALIDATE,
    NVM_BATTERY_STATE_DEFAULT_SAVE,
    NVM_BATTERY_STATE_TX_STATUS,
    NVM_BATTERY_STATE_TX_DATA,
    NVM_BATTERY_STATE_SAVE_CMD,
    NVM_BATTERY_STATE_DONE
    
} NVM_BatteryState_t;


static NVM_BatteryCfg_t g_batteryCfg;
static uint8_t g_nvmReadStatus = 0u;
static nvm_status_En_t g_nvmWriteStatus = (nvm_status_En_t)0;
static uint8_t g_nvmValid = 0u;
static uint8_t g_txFrameIndex = 0u;
static NVM_BatteryState_t g_batteryState = NVM_BATTERY_STATE_IDLE;

/* ---------------------------------------------------------
 *  LOCAL HELPERS
 * --------------------------------------------------------- */
static uint16_t NVM_BatteryCfg_CalcChecksum(const NVM_BatteryCfg_t *cfg)
{
    const uint8_t *raw = (const uint8_t *)cfg;
    uint16_t sum = 0u;
    uint8_t i;

    for (i = 0u; i < NVM_BATTERY_BLOCK_SIZE_BYTES; i++)
    {
        if ((i == 6u) || (i == 7u))
        {
            /* Skip checksum field bytes */
        }
        else
        {
            sum = (uint16_t)(sum + raw[i]);
        }
    }

    return sum;
}

static void NVM_BatteryCfg_TxStatus(void)
{
    uint8_t data[8];

    data[0] = (uint8_t)g_batteryState;
    data[1] = g_nvmReadStatus;
    data[2] = (uint8_t)g_nvmWriteStatus;
    data[3] = g_nvmValid;
    data[4] = g_batteryCfg.version;
    data[5] = (uint8_t)(g_batteryCfg.save_count & 0xFFu);
    data[6] = (uint8_t)((g_batteryCfg.save_count >> 8) & 0xFFu);
    data[7] = 0u;

    CanMsgTransmit(NVM_BATTERY_CAN_CH, data, 8u, 0u, 0u, NVM_BATTERY_CAN_STATUS_ID);
}

static void NVM_BatteryCfg_TxData(void)
{
    uint8_t data[8];

    switch (g_txFrameIndex)
    {
        case 0u:
        {

            
            data[0] = (uint8_t)(g_batteryCfg.s1_btry1_id & 0xFFu);
            data[1] = (uint8_t)((g_batteryCfg.s1_btry1_id >> 8) & 0xFFu);
            data[2] = (uint8_t)((g_batteryCfg.s1_btry1_id >> 16) & 0xFFu);
            data[3] = (uint8_t)((g_batteryCfg.s1_btry1_id >> 24) & 0xFFu);

            data[4] = (uint8_t)(g_batteryCfg.s1_btry2_id & 0xFFu);
            data[5] = (uint8_t)((g_batteryCfg.s1_btry2_id >> 8) & 0xFFu);
            data[6] = (uint8_t)((g_batteryCfg.s1_btry2_id >> 16) & 0xFFu);
            data[7] = (uint8_t)((g_batteryCfg.s1_btry2_id >> 24) & 0xFFu);


            

            /*
            data[0] = (uint8_t)(g_batteryCfg.NVM_Variable & 0xFFu);
            data[1] = (uint8_t)((g_batteryCfg.NVM_Variable >> 8) & 0xFFu);
            data[2] = (uint8_t)((g_batteryCfg.NVM_Variable >> 16) & 0xFFu);
            data[3] = (uint8_t)((g_batteryCfg.NVM_Variable >> 24) & 0xFFu);

            data[4] = 0u;
            data[5] = 0u;
            data[6] = 0u;
            data[7] = 0u;
            */


            CanMsgTransmit(NVM_BATTERY_CAN_CH, data, 8u, 0u, 0u, NVM_BATTERY_CAN_DATA1_ID);
            break;
        }

        case 1u:
        {
            data[0] = (uint8_t)(g_batteryCfg.s2_btry1_id & 0xFFu);
            data[1] = (uint8_t)((g_batteryCfg.s2_btry1_id >> 8) & 0xFFu);
            data[2] = (uint8_t)((g_batteryCfg.s2_btry1_id >> 16) & 0xFFu);
            data[3] = (uint8_t)((g_batteryCfg.s2_btry1_id >> 24) & 0xFFu);

            data[4] = (uint8_t)(g_batteryCfg.s2_btry2_id & 0xFFu);
            data[5] = (uint8_t)((g_batteryCfg.s2_btry2_id >> 8) & 0xFFu);
            data[6] = (uint8_t)((g_batteryCfg.s2_btry2_id >> 16) & 0xFFu);
            data[7] = (uint8_t)((g_batteryCfg.s2_btry2_id >> 24) & 0xFFu);

            CanMsgTransmit(NVM_BATTERY_CAN_CH, data, 8u, 0u, 0u, NVM_BATTERY_CAN_DATA2_ID);
            break;
        }

        case 2u:
        {
            data[0] = (uint8_t)(g_batteryCfg.s3_btry1_id & 0xFFu);
            data[1] = (uint8_t)((g_batteryCfg.s3_btry1_id >> 8) & 0xFFu);
            data[2] = (uint8_t)((g_batteryCfg.s3_btry1_id >> 16) & 0xFFu);
            data[3] = (uint8_t)((g_batteryCfg.s3_btry1_id >> 24) & 0xFFu);

            data[4] = (uint8_t)(g_batteryCfg.s3_btry2_id & 0xFFu);
            data[5] = (uint8_t)((g_batteryCfg.s3_btry2_id >> 8) & 0xFFu);
            data[6] = (uint8_t)((g_batteryCfg.s3_btry2_id >> 16) & 0xFFu);
            data[7] = (uint8_t)((g_batteryCfg.s3_btry2_id >> 24) & 0xFFu);

            CanMsgTransmit(NVM_BATTERY_CAN_CH, data, 8u, 0u, 0u, NVM_BATTERY_CAN_DATA3_ID);
            break;
        }

        case 3u:
        {
            data[0] = (uint8_t)(g_batteryCfg.s4_btry1_id & 0xFFu);
            data[1] = (uint8_t)((g_batteryCfg.s4_btry1_id >> 8) & 0xFFu);
            data[2] = (uint8_t)((g_batteryCfg.s4_btry1_id >> 16) & 0xFFu);
            data[3] = (uint8_t)((g_batteryCfg.s4_btry1_id >> 24) & 0xFFu);

            data[4] = (uint8_t)(g_batteryCfg.s4_btry2_id & 0xFFu);
            data[5] = (uint8_t)((g_batteryCfg.s4_btry2_id >> 8) & 0xFFu);
            data[6] = (uint8_t)((g_batteryCfg.s4_btry2_id >> 16) & 0xFFu);
            data[7] = (uint8_t)((g_batteryCfg.s4_btry2_id >> 24) & 0xFFu);

            CanMsgTransmit(NVM_BATTERY_CAN_CH, data, 8u, 0u, 0u, NVM_BATTERY_CAN_DATA4_ID);
            break;
        }

        default:
        {
            break;
        }
    }
}



static void NVM_BatteryCfg_TxAck(uint8_t cmd, uint8_t status, uint8_t param_id, uint32_t value)
{
    uint8_t data[8];

    data[0] = cmd;
    data[1] = status;
    data[2] = param_id;
    data[3] = 0u;
    data[4] = (uint8_t)(value & 0xFFu);
    data[5] = (uint8_t)((value >> 8) & 0xFFu);
    data[6] = (uint8_t)((value >> 16) & 0xFFu);
    data[7] = (uint8_t)((value >> 24) & 0xFFu);

    CanMsgTransmit(NVM_BATTERY_CAN_CH, data, 8u, 0u, 0u, NVM_BATTERY_CAN_ACK_ID);
}

/* ---------------------------------------------------------
 *  PUBLIC API
 * --------------------------------------------------------- */
void NVM_BatteryCfg_LoadDefaults(void)
{
    uint8_t i;
    uint8_t *raw = (uint8_t *)&g_batteryCfg;

    for (i = 0u; i < NVM_BATTERY_BLOCK_SIZE_BYTES; i++)
    {
        raw[i] = 0u;
    }

    g_batteryCfg.magic       = NVM_BATTERY_MAGIC;
    g_batteryCfg.version     = NVM_BATTERY_VERSION;
    g_batteryCfg.block_id    = NVM_BATTERY_BLOCK_ID;
    g_batteryCfg.save_count  = 1u;

    g_batteryCfg.s1_btry1_id = VC_S1_BTRY1_ID_DEFAULT;
    g_batteryCfg.s1_btry2_id = VC_S1_BTRY2_ID_DEFAULT;
    g_batteryCfg.s2_btry1_id = VC_S2_BTRY1_ID_DEFAULT;
    g_batteryCfg.s2_btry2_id = VC_S2_BTRY2_ID_DEFAULT;

    g_batteryCfg.s3_btry1_id = VC_S3_BTRY1_ID_DEFAULT;
    g_batteryCfg.s3_btry2_id = VC_S3_BTRY2_ID_DEFAULT;
    g_batteryCfg.s4_btry1_id = VC_S4_BTRY1_ID_DEFAULT;
    g_batteryCfg.s4_btry2_id = VC_S4_BTRY2_ID_DEFAULT;



    g_batteryCfg.checksum    = NVM_BatteryCfg_CalcChecksum(&g_batteryCfg);
}

uint8_t NVM_BatteryCfg_IsValid(void)
{
    if (g_batteryCfg.magic != NVM_BATTERY_MAGIC)
    {
        return 0u;
    }

    if (g_batteryCfg.version != NVM_BATTERY_VERSION)
    {
        return 0u;
    }

    if (g_batteryCfg.block_id != NVM_BATTERY_BLOCK_ID)
    {
        return 0u;
    }

    if (g_batteryCfg.checksum != NVM_BatteryCfg_CalcChecksum(&g_batteryCfg))
    {
        return 0u;
    }

    return 1u;
}

uint8_t NVM_BatteryCfg_Load(void)
{
    g_nvmReadStatus = NvmReadBlock(NVM_BATTERY_BLOCK_ID,
                                   (uint8_t *)&g_batteryCfg,  // (uint8_t *)&g_batteryCfg????
                                   NVM_BATTERY_BLOCK_SIZE_BYTES);

    return g_nvmReadStatus;
}

uint8_t NVM_BatteryCfg_Save(void)
{
    g_batteryCfg.checksum = NVM_BatteryCfg_CalcChecksum(&g_batteryCfg);

    g_nvmWriteStatus = NvmWriteBlock(NVM_BATTERY_BLOCK_ID,
                                     (uint8_t *)&g_batteryCfg,  // (uint8_t *)&g_batteryCfg????
                                     NVM_BATTERY_BLOCK_SIZE_BYTES);
                                                                        
    return (uint8_t)g_nvmWriteStatus;
}

const NVM_BatteryCfg_t *NVM_BatteryCfg_Get(void)
{
    return &g_batteryCfg;
}

void NVM_BatteryCfg_Init(void)
{
    g_nvmReadStatus = 0u;
    g_nvmWriteStatus = (nvm_status_En_t)0;
    g_nvmValid = 0u;
    g_txFrameIndex = 0u;
    g_batteryState = NVM_BATTERY_STATE_LOAD;
}

uint8_t NVM_BatteryCfg_SetParam(uint8_t param_id, uint32_t value)
{
    switch (param_id)
    {
        case NVM_PARAM_S1_BTRY1_ID:
        {
            g_batteryCfg.s1_btry1_id = value;
            return 1u;
        }

        case NVM_PARAM_S1_BTRY2_ID:
        {
            g_batteryCfg.s1_btry2_id = value;
            return 1u;
        }

        case NVM_PARAM_S2_BTRY1_ID:
        {
            g_batteryCfg.s2_btry1_id = value;
            return 1u;
        }

        case NVM_PARAM_S2_BTRY2_ID:
        {
            g_batteryCfg.s2_btry2_id = value;
            return 1u;
        }

        case NVM_PARAM_S3_BTRY1_ID:
        {
            g_batteryCfg.s3_btry1_id = value;
            return 1u;
        }

        case NVM_PARAM_S3_BTRY2_ID:
        {
            g_batteryCfg.s3_btry2_id = value;
            return 1u;
        }

        case NVM_PARAM_S4_BTRY1_ID:
        {
            g_batteryCfg.s4_btry1_id = value;
            return 1u;
        }

        case NVM_PARAM_S4_BTRY2_ID:
        {
            g_batteryCfg.s4_btry2_id = value;
            return 1u;
        }

        default:
        {
            return 0u;
        }
    }
}

void NVM_BatteryCfg_SaveCommand(void)
{
    g_batteryState = NVM_BATTERY_STATE_SAVE_CMD;
}

void NVM_BatteryCfg_CanRx(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    uint8_t param_id;
    uint32_t value;
    uint8_t ok;

    if ((data == 0) || (dlc < 8u))
    {
        return;
    }

    if (can_id == NVM_BATTERY_CAN_RX_SET_ID)
    {
        param_id = data[0];

        value =  ((uint32_t)data[2])
               | ((uint32_t)data[3] << 8u)
               | ((uint32_t)data[4] << 16u)
               | ((uint32_t)data[5] << 24u);

        ok = NVM_BatteryCfg_SetParam(param_id, value);

        if (ok != 0u)
        {
            CAN1_UpdateBatteryRxFiltersFromNvm();
            NVM_BatteryCfg_TxAck(0x01u, 0x00u, param_id, value);

            if ((param_id == NVM_PARAM_S1_BTRY1_ID) ||
                (param_id == NVM_PARAM_S1_BTRY2_ID))
            {
                g_txFrameIndex = 0u;
                NVM_BatteryCfg_TxData();   /* Sends 0x521 */
            }
            else if ((param_id == NVM_PARAM_S2_BTRY1_ID) ||
                    (param_id == NVM_PARAM_S2_BTRY2_ID))
            {
                g_txFrameIndex = 1u;
                NVM_BatteryCfg_TxData();   /* Sends 0x522 */
            }
            else if ((param_id == NVM_PARAM_S3_BTRY1_ID) ||
                    (param_id == NVM_PARAM_S3_BTRY2_ID))
            {
                g_txFrameIndex = 2u;
                NVM_BatteryCfg_TxData();   /* Sends 0x523 */
            }
            else if ((param_id == NVM_PARAM_S4_BTRY1_ID) ||
                    (param_id == NVM_PARAM_S4_BTRY2_ID))
            {
                g_txFrameIndex = 3u;
                NVM_BatteryCfg_TxData();   /* Sends 0x524 */
            }
            else
            {
            }
        }
        else
        {
            NVM_BatteryCfg_TxAck(0x01u, 0x01u, param_id, value);
        }
    }
    else if (can_id == NVM_BATTERY_CAN_RX_SAVE_ID)
    {
        if (data[0] == 0xA5u)
        {
            NVM_BatteryCfg_SaveCommand();
            NVM_BatteryCfg_TxAck(0x02u, 0x00u, 0u, 0u);

            NVM_BatteryCfg_TxStatus();
            g_txFrameIndex = 0u;
            NVM_BatteryCfg_TxData();

            g_txFrameIndex = 1u;
            NVM_BatteryCfg_TxData();

            g_txFrameIndex = 2u;
            NVM_BatteryCfg_TxData();

            g_txFrameIndex = 3u;
            NVM_BatteryCfg_TxData();
        }
        else if (data[0] == 0xA6u)
        {
            if (data[1] == 0x01u)
            {
                BatteryPrefixFilter_SetMode(1U);
                NVM_BatteryCfg_TxAck(0x03u, 0x00u, 0x01u, 0u);
            }
            else if ((data[1] == 0x02u) || (data[1] == 0x00u))
            {
                BatteryPrefixFilter_SetMode(0U);
                NVM_BatteryCfg_TxAck(0x03u, 0x00u, 0x02u, 0u);
            }
            else
            {
                NVM_BatteryCfg_TxAck(0x03u, 0x01u, data[1], 0u);
            }
        }
        else
        {
            NVM_BatteryCfg_TxAck(0x02u, 0x01u, 0u, 0u);
        }
    }
    else
    {
    }
}

void NVM_BatteryCfg_RxCallback(uint8_t CanNum_u8,
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

    NVM_BatteryCfg_CanRx(CanId_u32, DataBuff_pu8, Dlc_u8);
}



void NVM_BatteryCfg_Task(void)
{
    switch (g_batteryState)
    {
        case NVM_BATTERY_STATE_IDLE:
        {
            break;
        }

        case NVM_BATTERY_STATE_LOAD:
        {
            NVM_BatteryCfg_Load();
            g_batteryState = NVM_BATTERY_STATE_VALIDATE;
            break;
        }

        case NVM_BATTERY_STATE_VALIDATE:
        {
            g_nvmValid = NVM_BatteryCfg_IsValid();


            if (g_nvmValid == 0u)
            {
            
                g_batteryState = NVM_BATTERY_STATE_DEFAULT_SAVE;
            }
            else
            {
                CAN1_UpdateBatteryRxFiltersFromNvm();

                g_batteryState = NVM_BATTERY_STATE_TX_STATUS;
            }
            break;
        }

        case NVM_BATTERY_STATE_DEFAULT_SAVE:
        {
            NVM_BatteryCfg_LoadDefaults();
            NVM_BatteryCfg_Save();

            CAN1_UpdateBatteryRxFiltersFromNvm();

            g_batteryState = NVM_BATTERY_STATE_LOAD;
            break;
        }

        case NVM_BATTERY_STATE_TX_STATUS:
        {
            NVM_BatteryCfg_TxStatus();
            g_txFrameIndex = 0u;
            g_batteryState = NVM_BATTERY_STATE_TX_DATA;
            break;
        }

        case NVM_BATTERY_STATE_TX_DATA:
        {
            NVM_BatteryCfg_TxData();
            g_txFrameIndex++;

            if (g_txFrameIndex >= 4u)
            {
                g_batteryState = NVM_BATTERY_STATE_DONE;
            }
            break;
        }

        case NVM_BATTERY_STATE_SAVE_CMD:
        {
            g_batteryCfg.save_count++;
            NVM_BatteryCfg_Save();
            g_batteryState = NVM_BATTERY_STATE_LOAD;
            break;
        }

        case NVM_BATTERY_STATE_DONE:
        {
            break;
        }

        default:
        {
            g_batteryState = NVM_BATTERY_STATE_DONE;
            break;
        }
    }
}
