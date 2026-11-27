#include "battery_can_rx.h"
#include "vehicle_state.h"
#include "NVM_config.h"
#include <string.h>
#include "vehicle_config.h"
/* =========================================================
 * BMS CAN MATRIX (shared by user)
 * CAN ID   : 0x00ABCDEF
 * DLC      : 8
 * D0       : MUX (00..0E)
 * Multi-battery: different CAN IDs can map to Battery1/Battery2
 * ========================================================= */

/* ---------- Helpers ---------- */
static inline uint16_t U16_MSB_LSB(uint8_t msb, uint8_t lsb)
{
    return (uint16_t)(((uint16_t)msb << 8) | (uint16_t)lsb);
}

static inline int8_t S8(uint8_t v)
{
    return (int8_t)v;
}

volatile uint8_t g_BatteryPrefixFilterActive = VC_BATTERY_PREFIX_FILTER_ENABLE;

#if (VC_BATTERY_PREFIX_FILTER_ENABLE == 1U)
#define VC_BATTERY_PREFIX_ID_INVALID  (0xFFFFFFFFUL)

static uint32_t s_PrefixBatteryHighId = VC_BATTERY_PREFIX_ID_INVALID;
static uint32_t s_PrefixBatteryLowId  = VC_BATTERY_PREFIX_ID_INVALID;

void BatteryPrefixFilter_SetMode(uint8_t enable)
{
    g_BatteryPrefixFilterActive = (enable != 0U) ? 1U : 0U;
    /* Reset auto-discovered IDs so it re-binds cleanly */
    s_PrefixBatteryLowId  = VC_BATTERY_PREFIX_ID_INVALID;
    s_PrefixBatteryHighId = VC_BATTERY_PREFIX_ID_INVALID;
}

uint8_t BatteryPrefixFilter_GetMode(void)
{
    return g_BatteryPrefixFilterActive;
}

static BatteryState_t *GetPrefixBattery(uint32_t can_id)
{
    bool valid_prefix = false;

    if (((can_id & VC_BATTERY_PREFIX_ID_MASK) == VC_BATTERY_PREFIX_ID_BASE1) ||
        ((can_id & VC_BATTERY_PREFIX_ID_MASK) == VC_BATTERY_PREFIX_ID_BASE2) ||
        ((can_id & VC_BATTERY_PREFIX_ID_MASK) == VC_BATTERY_PREFIX_ID_BASE3) ||
        ((can_id & VC_BATTERY_PREFIX_ID_MASK) == VC_BATTERY_PREFIX_ID_BASE4))
    {
        valid_prefix = true;
    }
#ifdef VC_BATTERY_PREFIX_ID_BASE_2
    else if ((can_id & VC_BATTERY_PREFIX_ID_MASK_2) == VC_BATTERY_PREFIX_ID_BASE_2)
    {
        valid_prefix = true;
    }
#endif

    if (!valid_prefix)
    {
        return NULL;
    }

    if (s_PrefixBatteryLowId == VC_BATTERY_PREFIX_ID_INVALID)
    {
        s_PrefixBatteryLowId = can_id;
        return &VS_Battery1;
    }

    if (can_id == s_PrefixBatteryLowId)
    {
        return &VS_Battery1;
    }

    if (s_PrefixBatteryHighId == VC_BATTERY_PREFIX_ID_INVALID)
    {
        if (can_id < s_PrefixBatteryLowId)
        {
            s_PrefixBatteryHighId = s_PrefixBatteryLowId;
            VS_Battery2 = VS_Battery1;
            s_PrefixBatteryLowId = can_id;
            return &VS_Battery1;
        }

        s_PrefixBatteryHighId = can_id;
        return &VS_Battery2;
    }

    if (can_id == s_PrefixBatteryHighId)
    {
        return &VS_Battery2;
    }

    return NULL;
}
#else
void BatteryPrefixFilter_SetMode(uint8_t enable)
{
    (void)enable;
    g_BatteryPrefixFilterActive = 0U;
}

uint8_t BatteryPrefixFilter_GetMode(void)
{
    return 0U;
}
#endif

/* Copy up to 7 ASCII bytes from D1..D7 into dst (null terminated) */
static void CopyAscii7(char *dst, uint8_t dst_size, const uint8_t *D)
{
    uint8_t i;
    uint8_t n = (dst_size == 0U) ? 0U : (uint8_t)(dst_size - 1U);
    if (n > 7U) n = 7U;

    for (i = 0U; i < n; i++)
    {
        dst[i] = (char)D[1U + i];
    }
    if (dst_size > 0U)
    {
        dst[n] = '\0';
    }
}

/* ---------------- Core decoder (MUX 00..0E) ---------------- */
static void Decode_BMS_Frame(BatteryState_t *bat, const uint8_t *D)
{
    uint8_t mux = D[0];
    uint16_t raw_u16;

    switch (mux)
    {
        /* 00: Unique ID bytes (sample shows D1..D5 used, D6/D7 reserved) */
        case 0x00U:
            /* Store all 8 bytes (including mux) so you can reconstruct as needed */
            memcpy(bat->BmsUniqueIdRaw, D, 8U);
            break;

        /* 01: Battery Voltage/Current/Temp/Load/Status
         * D1: V MSB, D2: V LSB  (Factor 100)
         * D3: I MSB, D4: I LSB  (Factor 100)
         * D5: Battery Temp (signed, factor 1)
         * D6: Load Status
         * D7: Status
         */
        case 0x01U:
            raw_u16 = U16_MSB_LSB(D[1], D[2]);
            bat->BatteryVoltage_raw = raw_u16;    /* physical V = raw/100 */
            bat->Voltage_dV = (uint16_t)(raw_u16 / 10U);

            raw_u16 = U16_MSB_LSB(D[3], D[4]);
            bat->BatteryCurrent_raw = raw_u16;    /* physical A = raw/100 */
            bat->Current_dA = (int16_t)(raw_u16 / 10U);

            bat->BatteryTemp_C = S8(D[5]);        /* -120..100 expected */
            bat->LoadStatus    = D[6];
            bat->Status        = D[7];
            break;

        /* 02: SOH/SOC/Cycles/Capacity/Charge
         * D1: SOH, D2: SOC
         * D3: Cycles MSB, D4: Cycles LSB
         * D5: Capacity MSB, D6: Capacity LSB
         * D7: Charge
         */
        case 0x02U:
            bat->SOH = D[1];
            bat->SOC = D[2];
            bat->Cycles = U16_MSB_LSB(D[3], D[4]);
            bat->BatteryCapacity_raw = U16_MSB_LSB(D[5], D[6]);
            bat->Capacity = bat->BatteryCapacity_raw;
            bat->Charge = D[7];
            break;

        /* 03: Switch bits + Thermistor 10/9/8
         * D1: bit0 Discharging switch, bit1 Charging switch
         * D5: Therm10, D6: Therm9, D7: Therm8
         */
        case 0x03U:
            bat->Switch_Status = D[1];
            bat->CellThermistor_C[9]  = S8(D[5]);  /* Therm10 -> index 9 */
            bat->CellThermistor_C[8]  = S8(D[6]);  /* Therm9  -> index 8 */
            bat->CellThermistor_C[7]  = S8(D[7]);  /* Therm8  -> index 7 */
            break;

        /* 04: Thermistor 1..7
         * D1..D7: Therm1..Therm7
         */
        case 0x04U:
            bat->CellThermistor_C[0] = S8(D[1]);
            bat->CellThermistor_C[1] = S8(D[2]);
            bat->CellThermistor_C[2] = S8(D[3]);
            bat->CellThermistor_C[3] = S8(D[4]);
            bat->CellThermistor_C[4] = S8(D[5]);
            bat->CellThermistor_C[5] = S8(D[6]);
            bat->CellThermistor_C[6] = S8(D[7]);
            break;

        /* 05: Protocol Version ASCII (example: 281124) in D1..D7 */
        case 0x05U:
            CopyAscii7(bat->ProtocolVersion, sizeof(bat->ProtocolVersion), D);
            break;

        /* 06: Battery Serial Number ASCII (example: B999999) in D1..D7 */
        case 0x06U:
            CopyAscii7(bat->BatterySerialNumber, sizeof(bat->BatterySerialNumber), D);
            break;

        /* 07: Ambient + MOSFET temps
         * D1: Ambient, D2: MOSFET
         */
        case 0x07U:
            bat->Ambient_Temp_C = S8(D[1]);
            bat->MOSFET_Temp_C  = S8(D[2]);
            break;

        /* 08: Alarms/Protections bitfields
         * D1..D5 contain groups (as per matrix)
         */
        case 0x08U:
            bat->AlarmBits_1 = D[1];
            bat->AlarmBits_2 = D[2];
            bat->ProtBits_1  = D[3];
            bat->ProtBits_2  = D[4];
            bat->FaultBits_1 = D[5];
            break;

        /* 09: Cell 1..3 voltages (Factor 1000)
         * C1: D1 MSB D2 LSB
         * C2: D3 MSB D4 LSB
         * C3: D5 MSB D6 LSB
         */
        case 0x09U:
            bat->CellVoltage_raw_mV[0] = U16_MSB_LSB(D[1], D[2]);
            bat->CellVoltage_raw_mV[1] = U16_MSB_LSB(D[3], D[4]);
            bat->CellVoltage_raw_mV[2] = U16_MSB_LSB(D[5], D[6]);
            break;

        /* 0A: Cell 4..6 */
        case 0x0AU:
            bat->CellVoltage_raw_mV[3] = U16_MSB_LSB(D[1], D[2]);
            bat->CellVoltage_raw_mV[4] = U16_MSB_LSB(D[3], D[4]);
            bat->CellVoltage_raw_mV[5] = U16_MSB_LSB(D[5], D[6]);
            break;

        /* 0B: Cell 7..9 */
        case 0x0BU:
            bat->CellVoltage_raw_mV[6] = U16_MSB_LSB(D[1], D[2]);
            bat->CellVoltage_raw_mV[7] = U16_MSB_LSB(D[3], D[4]);
            bat->CellVoltage_raw_mV[8] = U16_MSB_LSB(D[5], D[6]);
            break;

        /* 0C: Cell 10..12 */
        case 0x0CU:
            bat->CellVoltage_raw_mV[9]  = U16_MSB_LSB(D[1], D[2]);
            bat->CellVoltage_raw_mV[10] = U16_MSB_LSB(D[3], D[4]);
            bat->CellVoltage_raw_mV[11] = U16_MSB_LSB(D[5], D[6]);
            break;

        /* 0D: Cell 13..15 */
        case 0x0DU:
            bat->CellVoltage_raw_mV[12] = U16_MSB_LSB(D[1], D[2]);
            bat->CellVoltage_raw_mV[13] = U16_MSB_LSB(D[3], D[4]);
            bat->CellVoltage_raw_mV[14] = U16_MSB_LSB(D[5], D[6]);
            break;

        /* 0E: Cell 16 */
        case 0x0EU:
            bat->CellVoltage_raw_mV[15] = U16_MSB_LSB(D[1], D[2]);
            break;

        default:
            break;
    }
}

void Call_Back_Battery_Rx(uint8_t CanNum_u8,
                          uint8_t Channel_u8,
                          uint8_t *D,
                          uint8_t Dlc_u8,
                          uint8_t CanMode_u8,
                          uint8_t Ide_u8,
                          uint32_t CanId_u32)
{
    BatteryState_t *bat = NULL;
    const NVM_BatteryCfg_t *cfg = NVM_BatteryCfg_Get();

    if (Dlc_u8 < 8U)
    {
        return;
    }

#if (VC_BATTERY_PREFIX_FILTER_ENABLE == 1U)
    if (g_BatteryPrefixFilterActive != 0U)
    {
        bat = GetPrefixBattery(CanId_u32);
        if (bat == NULL)
        {
            return;
        }
    }
    else
#endif
    {
        if ((CanId_u32 == cfg->s1_btry1_id) ||
            (CanId_u32 == cfg->s2_btry1_id) ||
            (CanId_u32 == cfg->s3_btry1_id) ||
            (CanId_u32 == cfg->s4_btry1_id))
        {
            bat = &VS_Battery1;
        }
        else if ((CanId_u32 == cfg->s1_btry2_id) ||
                 (CanId_u32 == cfg->s2_btry2_id) ||
                 (CanId_u32 == cfg->s3_btry2_id) ||
                 (CanId_u32 == cfg->s4_btry2_id))
        {
            bat = &VS_Battery2;
        }
        else
        {
            return;
        }
    }

    Decode_BMS_Frame(bat, D);

    bat->Alive = 1U;
    bat->RxTimeoutMs = VC_BATT_FUSE_RX_TIMEOUT_MS;

    (void)CanNum_u8;
    (void)Channel_u8;
    (void)CanMode_u8;
    (void)Ide_u8;
}
