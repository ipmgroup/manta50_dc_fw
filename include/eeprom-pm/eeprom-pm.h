/*
 * eeprom-pm.h
 *
 *  Created on: 15.04.2016
 *      Author: Dmitri Ranfft
 */

#ifndef INCLUDE_EEPROM_PM_EEPROM_PM_H_
#define INCLUDE_EEPROM_PM_EEPROM_PM_H_
#ifdef EEPROM_25AA02

#include "main.h"
#include "dronecan.h"
#include "version.h"
// #include "hal.h"
// #include "Microchip_25AA02E48.h"

// ###############################################################

// These are the conversion factors for saving/reading corresponding values from EEPROM.
// They are used to fit numbers over 255 or under 1 into 1 byte of memory with sufficient precision.
#define RPM_LIMIT_STEP (200)
#define RPMPS_LIMIT_STEP (50)
#define KP_FACTOR (10.0)
#define KI_FACTOR (1000.0)

#define FLOAT 4
#define UINT8 1
#define UINT16 2

// ###############################################################

#define PACK_32BIT(value, buf, idx)      \
    buf[idx] = (value >> 24) & 0xFF;     \
    buf[idx + 1] = (value >> 16) & 0xFF; \
    buf[idx + 2] = (value >> 8) & 0xFF;  \
    buf[idx + 3] = value & 0xFF;

#define PACK_16BIT(value, buf, idx) \
    buf[idx] = (value >> 8) & 0xFF; \
    buf[idx + 1] = value & 0xFF;

#define UNPACK_32BIT_FROM_BUF(dest, src, offset)             \
    do {                                                     \
        *(uint32_t *)&(dest) =                               \
            (((uint32_t)(src)[(offset)] & 0xFF) << 24) |     \
            (((uint32_t)(src)[(offset) + 1] & 0xFF) << 16) | \
            (((uint32_t)(src)[(offset) + 2] & 0xFF) << 8) |  \
            (((uint32_t)(src)[(offset) + 3] & 0xFF));        \
    } while (0)

#define UNPACK_16BIT_FROM_BUF(dest, src, offset)        \
    do {                                                \
        (dest) =                                        \
            (((uint16_t)(src)[(offset)] & 0xFF) << 8) | \
            (((uint16_t)(src)[(offset) + 1] & 0xFF));   \
    } while (0)

// Macro to set a parameter in the buffer
#define EEPROM_BUF_SET_PARAM(buf, len, var)   \
    do {                                      \
        memcpy(&buf[len], &var, sizeof(var)); \
        len += sizeof(var);                   \
    } while (0)

// Macro to pack a parameter into the buffer
#define PACK_PARAM(TYPE, PARAM)              \
    {                                        \
        TYPE tmp = (TYPE)(PARAM);            \
        EEPROM_BUF_SET_PARAM(buf, len, tmp); \
    }

// Macro to get a parameter from the buffer
#define EEPROM_BUF_GET_PARAM(buf, len, var)   \
    do {                                      \
        memcpy(&var, &buf[len], sizeof(var)); \
        len += sizeof(var);                   \
    } while (0);
// ###############################################################

#define EEPROM_SIZE 256                     // Byte.
#define EEPROM_BLOCK_SIZE 16                // Block size in bytes.
#define EEPROM_PROFILE_START_BLOCK 1        // Block 0 will be reserved for metadata.
#define EEPROM_PROFILE_SIZE 5               // In blocks.
#define EEPROM_PROFILE_COUNT 2              // (((EEPROM_SIZE/EEPROM_BLOCK_SIZE)-EEPROM_PROFILE_START_BLOCK)/EEPROM_PROFILE_SIZE) // Number of profiles based on sizes.
#define EEPROM_PROFILE_SELECT_ADDRESS 0x04  // Address of the byte containing the id of the target profile.
#define START_ADDR 0x10
#define EEPROM_PROFILE_SIZE_IN_BYTES (EEPROM_PROFILE_SIZE_IN_BLOCKS * EEPROM_BLOCK_SIZE)

// #define EEPROM_VERIFICATION_NUMBER 0x13117535  // EEPROM verification number.
#define EEPROM_VERIFICATION_NUMBER GIT_HASH  // EEPROM verification number.
#define EEPROM_VERIFICATION_LENGTH 4      // Length of the verification number in bytes.
#define EEPROM_VERIFICATION_ADDRESS 0x00  // Starting address of the verification number.

// ###############################################################

void EEPROM_init_pm(HAL_Handle halHandle);
void EEPROM_disableWP(HAL_Handle halHandle);
void EEPROM_enableWP(HAL_Handle halHandle);
void EEPROM_save(HAL_Handle halHandle, uint16_t addr, uint_least8_t *data, int len);
void EEPROM_read(HAL_Handle halHandle, uint16_t addr, uint_least8_t *buf, int len);
uint32_t EEPROM_getVerificationNumber(HAL_Handle halHandle);

uint_least8_t EEPROM_getTargetProfile(HAL_Handle halHandle);
void EEPROM_setTargetProfile(HAL_Handle halHandle, uint_least8_t pn);
int EEPROM_loadProfile(HAL_Handle halHandle, int pn, USER_Params *u_params, MOTOR_Vars_t *m_params, settings_t *settings);
int EEPROM_saveProfile(HAL_Handle halHandle, int pn, USER_Params *u_params, MOTOR_Vars_t *m_params, settings_t *settings);
void EEPROM_initMem(HAL_Handle halHandle, USER_Params *u_params, MOTOR_Vars_t *m_params, settings_t *settings);
void EEPROM_startupProfileLoad(HAL_Handle halHandle, USER_Params *u_params, MOTOR_Vars_t *m_params, settings_t *settings);
void EEPROM_resetVerificationNumber(HAL_Handle halHandle);
void EEPROM_clear(HAL_Handle halHandle);

#endif  // EEPROM_25AA02
#endif  /* INCLUDE_EEPROM_PM_EEPROM_PM_H_ */
