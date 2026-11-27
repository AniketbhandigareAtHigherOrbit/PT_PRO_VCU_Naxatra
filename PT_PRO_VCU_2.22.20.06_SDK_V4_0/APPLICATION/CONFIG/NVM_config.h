#ifndef NVM_CONFIG_H
#define NVM_CONFIG_H

#include "app_typedef.h"
#include "pal_can_conf.h"
#include <stdint.h>

#define NVM_BATTERY_BLOCK_ID             (0u)
#define NVM_BATTERY_BLOCK_SIZE_BYTES     (64u)

#define NVM_BATTERY_MAGIC                (0xA55Au)

//#define NVM_BATTERY_MAGIC               (0x1234Au)

#define NVM_BATTERY_VERSION              (1u)

/* CAN IDs */
#define NVM_BATTERY_CAN_STATUS_ID        (0x520u)

#define NVM_BATTERY_CAN_DATA1_ID         (0x521u)
#define NVM_BATTERY_CAN_DATA2_ID         (0x522u)

#define NVM_BATTERY_CAN_DATA3_ID         (0x523u)
#define NVM_BATTERY_CAN_DATA4_ID         (0x524u)

#define NVM_BATTERY_CAN_ACK_ID           (0x532u)

#define NVM_BATTERY_CAN_RX_SET_ID        (0x530u)
#define NVM_BATTERY_CAN_RX_SAVE_ID       (0x531u)

/* Parameter IDs */
#define NVM_PARAM_S1_BTRY1_ID            (1u)
#define NVM_PARAM_S1_BTRY2_ID            (2u)
#define NVM_PARAM_S2_BTRY1_ID            (3u)
#define NVM_PARAM_S2_BTRY2_ID            (4u)

#define NVM_PARAM_S3_BTRY1_ID            (5u)
#define NVM_PARAM_S3_BTRY2_ID            (6u)
#define NVM_PARAM_S4_BTRY1_ID            (7u)
#define NVM_PARAM_S4_BTRY2_ID            (8u)

#pragma pack(1)
typedef struct
{
    uint16_t magic;             /* 0-1   */
    uint8_t  version;           /* 2     */
    uint8_t  block_id;          /* 3     */
    uint16_t save_count;        /* 4-5   */
    uint16_t checksum;          /* 6-7   */

    uint32_t s1_btry1_id;       /* 8-11  */
    uint32_t s1_btry2_id;       /* 12-15 */
    uint32_t s2_btry1_id;       /* 16-19 */
    uint32_t s2_btry2_id;       /* 20-23 */

    uint32_t s3_btry1_id; 
    uint32_t s3_btry2_id; 
    uint32_t s4_btry1_id; 
    uint32_t s4_btry2_id; 

    uint8_t  reserved[24];      /* 40-63 */
} NVM_BatteryCfg_t;


#pragma unpack

void NVM_BatteryCfg_Init(void);
void NVM_BatteryCfg_Task(void);

void NVM_BatteryCfg_LoadDefaults(void);
uint8_t NVM_BatteryCfg_Load(void);
uint8_t NVM_BatteryCfg_Save(void);
uint8_t NVM_BatteryCfg_IsValid(void);

uint8_t NVM_BatteryCfg_SetParam(uint8_t param_id, uint32_t value);
void NVM_BatteryCfg_SaveCommand(void);
void NVM_BatteryCfg_CanRx(uint32_t can_id, const uint8_t *data, uint8_t dlc);

void NVM_BatteryCfg_RxCallback(uint8_t CanNum_u8,
                               uint8_t Channel_u8,
                               uint8_t *DataBuff_pu8,
                               uint8_t Dlc_u8,
                               uint8_t CanMode_u8,
                               uint8_t Ide_u8,
                               uint32_t CanId_u32);

const NVM_BatteryCfg_t *NVM_BatteryCfg_Get(void);

#endif /* NVM_CONFIG_H */
