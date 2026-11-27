#include "app_typedef.h"
#include "vehicle_config.h"
/* =========================================================
 *  ADC CONFIG
 * ========================================================= */
const uint16_t VC_ADC_MAX_VALUE = 4095U;   /* 12-bit ADC */

/* =========================================================
 *  RPM CONFIG
 * ========================================================= */
const uint16_t VC_WHEEL_MAX_RPM = 1430U;
const uint16_t VC_ROTOR_MAX_RPM = 3000U;

/* =========================================================
 *  CONTROLLER DEVICE IDs
 * ========================================================= */
const uint8_t VC_WHEEL_LEFT_ID  = 4U;
const uint8_t VC_WHEEL_RIGHT_ID = 6U;
const uint8_t VC_ROTOR_ID       = 5U;

/* =========================================================
 *  CAN CONFIG
 * ========================================================= */
const uint8_t VC_CAN_ENABLE_DEFAULT = 1U;
const uint8_t VC_DIRECTION_DEFAULT  = 1U;   /* 1 = FWD */

/* =========================================================
 *  CAN BASE ID (Message 3)
 * ========================================================= */
const uint32_t VC_BASE_CAN_ID = 0x0CF1001EUL;


const uint16_t VC_TURN_REDUCTION_RPM = 100U;
const uint16_t VC_TURN_LOW_RPM = 100U;
uint16_t STARTUP_FRAME_COUNTER = 600U;
/* =========================================================
 *  BATTERY CAN ID DEFAULTS (factory defaults for NVM)
 *  Runtime code should use NVM_BatteryCfg_Get()->... values
 * ========================================================= */

const uint32_t VC_S1_BTRY1_ID = VC_S1_BTRY1_ID_DEFAULT;
const uint32_t VC_S1_BTRY2_ID = VC_S1_BTRY2_ID_DEFAULT;
const uint32_t VC_S2_BTRY1_ID = VC_S2_BTRY1_ID_DEFAULT;
const uint32_t VC_S2_BTRY2_ID = VC_S2_BTRY2_ID_DEFAULT;

const uint32_t VC_S3_BTRY1_ID = VC_S3_BTRY1_ID_DEFAULT;
const uint32_t VC_S3_BTRY2_ID = VC_S3_BTRY2_ID_DEFAULT;
const uint32_t VC_S4_BTRY1_ID = VC_S4_BTRY1_ID_DEFAULT;
const uint32_t VC_S4_BTRY2_ID = VC_S4_BTRY2_ID_DEFAULT;
