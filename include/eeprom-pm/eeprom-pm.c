/*
 * eeprom-pm.c
 *
 *  Created on: 15.04.2016
 *      Author: Dmitri Ranfft
 */
#ifdef EEPROM_25AA02

#include <math.h>  // Needed for sqrt();
#include "eeprom-pm.h"

// uint_least8_t prof_buf[EEPROM_PROFILE_SIZE * EEPROM_BLOCK_SIZE] = {0};
// #ifdef F2802xF
// #pragma DATA_SECTION(prof_buf, "rom_accessed_data");
// #endif

void EEPROM_init_pm(HAL_Handle halHandle) {
    HAL_setupEEPROM25AA02(halHandle);
}

void EEPROM_disableWP(HAL_Handle halHandle) {
    HAL_setupSpi_25AA02(halHandle);

    // Disable write protection on the whole array: 0x00.
    // Enable  write protection on 0xC0 - 0xFF:     0x04.
    // Enable  write protection on 0x80 - 0xFF:     0x08.
    // Enable  write protection on the whole array: 0x0C.
    EEPROM25AA02_writeStatus(halHandle->eeprom25aa02Handle, 0x00);
#ifdef DRV8301_SPI
    HAL_setupSpiA(halHandle);
#else
    HAL_setupSpi_MCP2515(halHandle);
#endif
}

void EEPROM_enableWP(HAL_Handle halHandle) {
    HAL_setupSpi_25AA02(halHandle);

    // Disable write protection on the whole array: 0x00.
    // Enable  write protection on 0xC0 - 0xFF:     0x04.
    // Enable  write protection on 0x80 - 0xFF:     0x08.
    // Enable  write protection on the whole array: 0x0C.
    EEPROM25AA02_writeStatus(halHandle->eeprom25aa02Handle, 0x0C);
#ifdef DRV8301_SPI
    HAL_setupSpiA(halHandle);
#else
    HAL_setupSpi_MCP2515(halHandle);
#endif
}

void EEPROM_save(HAL_Handle halHandle, uint16_t addr, uint_least8_t *data, int len) {
    HAL_setupSpi_25AA02(halHandle);
    EEPROM25AA02_writeRegisterN(halHandle->eeprom25aa02Handle, addr, data, len);
#ifdef DRV8301_SPI
    HAL_setupSpiA(halHandle);
#else
    HAL_setupSpi_MCP2515(halHandle);
#endif
}

void EEPROM_read(HAL_Handle halHandle, uint16_t addr, uint_least8_t *buf, int len) {
    HAL_setupSpi_25AA02(halHandle);
    EEPROM25AA02_readRegisterN(halHandle->eeprom25aa02Handle, addr, buf, len);
#ifdef DRV8301_SPI
    HAL_setupSpiA(halHandle);
#else
    HAL_setupSpi_MCP2515(halHandle);
#endif
}

uint32_t EEPROM_getVerificationNumber(HAL_Handle halHandle) {
    int i = 0;
    uint32_t verNum = 0;
    uint_least8_t buf[EEPROM_VERIFICATION_LENGTH];
    EEPROM_read(halHandle, EEPROM_VERIFICATION_ADDRESS, buf, EEPROM_VERIFICATION_LENGTH);

    for (i = 0; i < EEPROM_VERIFICATION_LENGTH; i++) {
        volatile uint16_t shift = ((EEPROM_VERIFICATION_LENGTH - 1) - i) << 3;  // Number of bits to shift when saving data from the buffer to the variable.
        verNum |= (((uint32_t)buf[i]) & 0xFF) << shift;
    }

    return verNum;
}

void EEPROM_resetVerificationNumber(HAL_Handle halHandle) {
    int i = 0;
    uint_least8_t buf[EEPROM_BLOCK_SIZE];
    for (i = 0; i < EEPROM_BLOCK_SIZE; i++) {
        buf[i] = 0;
    }
    EEPROM_save(halHandle, EEPROM_VERIFICATION_ADDRESS, buf, EEPROM_BLOCK_SIZE);
}

void EEPROM_clear(HAL_Handle halHandle) {
    int page = 0;
    uint_least8_t buf[EEPROM_BLOCK_SIZE] = {0};

    for (page = 0; page < EEPROM_SIZE; page += EEPROM_BLOCK_SIZE) {
        EEPROM_save(halHandle, page, buf, EEPROM_BLOCK_SIZE);
    }
}

void EEPROM_setVerificationNumber(HAL_Handle halHandle, uint32_t verNum) {
    int i = 0;
    uint_least8_t buf[EEPROM_VERIFICATION_LENGTH];

    for (i = 0; i < EEPROM_VERIFICATION_LENGTH; i++) {
        uint16_t shift = ((EEPROM_VERIFICATION_LENGTH - 1) - i) << 3;  // Number of bits to shift when saving data from the buffer to the variable.
        buf[i] = (verNum >> shift) & 0xFF;
    }

    EEPROM_save(halHandle, EEPROM_VERIFICATION_ADDRESS, buf, EEPROM_VERIFICATION_LENGTH);
}

int EEPROM_isNew(HAL_Handle halHandle) {
    uint32_t verNum = EEPROM_getVerificationNumber(halHandle);
    return !(verNum == EEPROM_VERIFICATION_NUMBER);
}

uint_least8_t EEPROM_getTargetProfile(HAL_Handle halHandle) {
    uint_least8_t profileNumber = 0;
    EEPROM_read(halHandle, EEPROM_PROFILE_SELECT_ADDRESS, &profileNumber, 1);
    return profileNumber;
}

void EEPROM_setTargetProfile(HAL_Handle halHandle, uint_least8_t pn) {
    EEPROM_save(halHandle, EEPROM_PROFILE_SELECT_ADDRESS, &pn, 1);
}

int EEPROM_loadProfile(HAL_Handle halHandle, int pn, USER_Params *u_params, MOTOR_Vars_t *m_params, settings_t *settings) {
    // Check if the profile number is within the valid range
    if (pn < 0 || pn >= EEPROM_PROFILE_COUNT) {
        return -1;  // Invalid profile number
    }
    uint32_t tmp;

    // Create a buffer to hold the profile data
    uint_least8_t buf[EEPROM_PROFILE_SIZE * EEPROM_BLOCK_SIZE] = {0};

    // Calculate the start address for the requested profile
    uint16_t startAddress = (EEPROM_PROFILE_SIZE * pn + EEPROM_PROFILE_START_BLOCK) * EEPROM_BLOCK_SIZE;

    // Read the profile data from EEPROM block by block
    uint_least8_t *bufPtr = buf;
    for (int i = 0; i < EEPROM_PROFILE_SIZE; i++) {
        EEPROM_read(halHandle, startAddress, bufPtr, EEPROM_BLOCK_SIZE);
        startAddress += EEPROM_BLOCK_SIZE;
        bufPtr += EEPROM_BLOCK_SIZE;
    }

    // Unpacking settings data
    settings->can_node = buf[0];    // 1 byte for CAN node
    settings->esc_index = buf[1];   // 1 byte for ESC index
    settings->telem_rate = buf[2];  // 1 byte for telemetry rate
    settings->can_speed = buf[3];   // 1 byte for CAN speed
    UNPACK_32BIT_FROM_BUF(settings->midle_point, buf, 4);
    settings->controll_word = buf[8];  // 1 byte for control word

    // Unpacking motor type
    u_params->motor_type = buf[9];  // 1 byte for motor type

    // Unpacking 32-bit settings values using the macro
    UNPACK_32BIT_FROM_BUF(settings->max_speed, buf, 10);     // 4 bytes for max speed
    UNPACK_32BIT_FROM_BUF(settings->acseleration, buf, 14);  // 4 bytes for acceleration
    UNPACK_32BIT_FROM_BUF(settings->motor_poles, buf, 18);   // 4 bytes for motor poles
    UNPACK_32BIT_FROM_BUF(settings->Kp, buf, 22);            // 4 bytes for Kp parameter
    UNPACK_32BIT_FROM_BUF(settings->Ki, buf, 26);            // 4 bytes for Ki parameter

    // Unpacking 16-bit value for number of pole pairs using the macro
    UNPACK_16BIT_FROM_BUF(u_params->motor_numPolePairs, buf, 30);  // 2 bytes for number of pole pairs

    // Unpacking 32-bit motor parameters using the macro
    UNPACK_32BIT_FROM_BUF(u_params->motor_Rr, buf, 32);           // 4 bytes for motor Rr
    UNPACK_32BIT_FROM_BUF(u_params->motor_Rs, buf, 36);           // 4 bytes for motor Rs
    UNPACK_32BIT_FROM_BUF(u_params->motor_Ls_d, buf, 40);         // 4 bytes for motor Ls_d
    UNPACK_32BIT_FROM_BUF(u_params->motor_Ls_q, buf, 44);         // 4 bytes for motor Ls_q
    UNPACK_32BIT_FROM_BUF(u_params->motor_ratedFlux, buf, 48);    // 4 bytes for motor rated flux
    UNPACK_32BIT_FROM_BUF(u_params->IdRated, buf, 52);            // 4 bytes for IdRated
    UNPACK_32BIT_FROM_BUF(u_params->maxCurrent_resEst, buf, 56);  // 4 bytes for max current for resistance estimation
    UNPACK_32BIT_FROM_BUF(u_params->maxCurrent_indEst, buf, 60);  // 4 bytes for max current for inductance estimation
    UNPACK_32BIT_FROM_BUF(u_params->maxCurrent, buf, 64);         // 4 bytes for maximum current
    UNPACK_32BIT_FROM_BUF(u_params->fluxEstFreq_Hz, buf, 68);     // 4 bytes for flux estimation frequency

    // Unpacking speed control parameters as 32-bit integers
    UNPACK_32BIT_FROM_BUF(tmp, buf, 72);  // Kp speed
    m_params->Kp_spd = _IQ(tmp);
    UNPACK_32BIT_FROM_BUF(tmp, buf, 76);  // Ki speed
    m_params->Ki_spd = _IQ(tmp);

    // Calculate RsOnLineCurrent_A
    m_params->RsOnLineCurrent_A = _IQ(0.1 * u_params->maxCurrent);

    // Calculate power warp gain
    if ((u_params->motor_Rr > (float_t)0.0) && (u_params->motor_Rs > (float_t)0.0)) {
        u_params->powerWarpGain = sqrt((float_t)1.0 + u_params->motor_Rr / u_params->motor_Rs);
    } else {
        u_params->powerWarpGain = USER_POWERWARP_GAIN;
    }

    return 0;
}

int EEPROM_saveProfile(HAL_Handle halHandle, int pn, USER_Params *u_params, MOTOR_Vars_t *m_params, settings_t *settings) {
    // Check if the profile number is within the valid range
    if (pn < 0 || pn >= EEPROM_PROFILE_COUNT) {
        return -1;  // Invalid profile number
    }

    // Create a buffer to hold the profile data
    uint_least8_t buf[EEPROM_PROFILE_SIZE * EEPROM_BLOCK_SIZE] = {0};

    // Packing settings data
    buf[0] = (uint_least8_t)settings->can_node;    // 1 byte for CAN node
    buf[1] = (uint_least8_t)settings->esc_index;   // 1 byte for ESC index
    buf[2] = (uint_least8_t)settings->telem_rate;  // 1 byte for telemetry rate
    buf[3] = (uint_least8_t)settings->can_speed;   // 1 byte for CAN speed

    // Packing 32-bit values (assumed to be important settings)
    PACK_32BIT(*((uint32_t *)&settings->midle_point), buf, 4);  // 4 bytes for midle point
    buf[8] = (uint_least8_t)settings->controll_word;            // 1 byte for control word

    // Packing motor type
    buf[9] = u_params->motor_type;  // 1 byte for motor type

    // Packing 32-bit settings values
    PACK_32BIT(*((uint32_t *)&settings->max_speed), buf, 10);     // 4 bytes for max speed
    PACK_32BIT(*((uint32_t *)&settings->acseleration), buf, 14);  // 4 bytes for acceleration
    PACK_32BIT(*((uint32_t *)&settings->motor_poles), buf, 18);   // 4 bytes for motor poles
    PACK_32BIT(*((uint32_t *)&settings->Kp), buf, 22);            // 4 bytes for Kp parameter
    PACK_32BIT(*((uint32_t *)&settings->Ki), buf, 26);            // 4 bytes for Ki parameter

    // Packing 16-bit value for number of pole pairs
    PACK_16BIT(u_params->motor_numPolePairs, buf, 30);  // 2 bytes for number of pole pairs

    // Packing 32-bit motor parameters
    PACK_32BIT(*((uint32_t *)&u_params->motor_Rr), buf, 32);           // 4 bytes for motor Rr
    PACK_32BIT(*((uint32_t *)&u_params->motor_Rs), buf, 36);           // 4 bytes for motor Rs
    PACK_32BIT(*((uint32_t *)&u_params->motor_Ls_d), buf, 40);         // 4 bytes for motor Ls_d
    PACK_32BIT(*((uint32_t *)&u_params->motor_Ls_q), buf, 44);         // 4 bytes for motor Ls_q
    PACK_32BIT(*((uint32_t *)&u_params->motor_ratedFlux), buf, 48);    // 4 bytes for motor rated flux
    PACK_32BIT(*((uint32_t *)&u_params->IdRated), buf, 52);            // 4 bytes for IdRated
    PACK_32BIT(*((uint32_t *)&u_params->maxCurrent_resEst), buf, 56);  // 4 bytes for max current for resistance estimation
    PACK_32BIT(*((uint32_t *)&u_params->maxCurrent_indEst), buf, 60);  // 4 bytes for max current for inductance estimation
    PACK_32BIT(*((uint32_t *)&u_params->maxCurrent), buf, 64);         // 4 bytes for maximum current
    PACK_32BIT(*((uint32_t *)&u_params->fluxEstFreq_Hz), buf, 68);     // 4 bytes for flux estimation frequency

    // Packing speed control parameters as 32-bit integers
    uint32_t tmp;
    tmp = (uint32_t)(_IQtoF(m_params->Kp_spd));  // Convert Kp_spd to uint32_t
    PACK_32BIT(tmp, buf, 72);                    // 4 bytes for Kp speed
    tmp = (uint32_t)(_IQtoF(m_params->Ki_spd));  // Convert Ki_spd to uint32_t
    PACK_32BIT(tmp, buf, 76);                    // 4 bytes for Ki speed

    // Calculate the start address for the requested profile
    uint16_t startAddress = (EEPROM_PROFILE_SIZE * pn + EEPROM_PROFILE_START_BLOCK) * EEPROM_BLOCK_SIZE;

    // Write the profile data to EEPROM block by block
    uint_least8_t *bufPtr = buf;
    for (int i = 0; i < EEPROM_PROFILE_SIZE; i++) {
        EEPROM_save(halHandle, startAddress, bufPtr, EEPROM_BLOCK_SIZE);
        startAddress += EEPROM_BLOCK_SIZE;
        bufPtr += EEPROM_BLOCK_SIZE;
    }
    return 0;
}

void EEPROM_initMem(HAL_Handle halHandle, USER_Params *u_params, MOTOR_Vars_t *m_params, settings_t *settings) {
    int i = 0;
    for (i = 0; i < EEPROM_PROFILE_COUNT; i++) {
        EEPROM_saveProfile(halHandle, i, u_params, m_params, settings);
    }
}

/*
 * If EEPROM is verified, loads profile from EEPROM. Otherwise it
 * initializes EEPROM with the default parameters.
 */
void EEPROM_startupProfileLoad(HAL_Handle halHandle, USER_Params *u_params, MOTOR_Vars_t *m_params, settings_t *settings) {
    if (EEPROM_isNew(halHandle)) {
        EEPROM_initMem(halHandle, u_params, m_params, settings);
        EEPROM_setTargetProfile(halHandle, 0);
        EEPROM_setVerificationNumber(halHandle, EEPROM_VERIFICATION_NUMBER);
    } else {
        uint_least8_t lastProfile = EEPROM_getTargetProfile(halHandle);
        EEPROM_loadProfile(halHandle, (int)lastProfile, u_params, m_params, settings);
    }
}

#endif  // EEPROM_25AA02
