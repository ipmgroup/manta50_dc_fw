/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//! \file  sw\modules\hal\boards\boostxldrv8305evm_revA\f28x\f2802x\src\hal.c
//! \brief Contains the various functions related to the HAL object (everything
//! outside the CTRL system)
//!
//! (C) Copyright 2015, Texas Instruments, Inc.

// **************************************************************************
// the includes

// drivers

// modules

// platforms
#include "hal.h"
#include "hal_obj.h"
#include "user.h"

#ifdef FLASH
#pragma CODE_SECTION(HAL_setupFlash, "ramfuncs");
#endif

// **************************************************************************
// the defines

// **************************************************************************
// the globals

#ifdef F2802xF
#pragma DATA_SECTION(hal, "rom_accessed_data");
#endif
HAL_Obj hal;

// **************************************************************************
// the functions

void HAL_cal(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    // enable the ADC clock
    CLK_enableAdcClock(obj->clkHandle);

    // Run the Device_cal() function
    // This function copies the ADC and oscillator calibration values from TI
    // reserved OTP into the appropriate trim registers This boot ROM
    // automatically calls this function to calibrate the interal oscillators and
    // ADC with device specific calibration data. If the boot ROM is bypassed by
    // Code Composer Studio during the development process, then the calibration
    // must be initialized by the application
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;
    (*Device_cal)();
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    // run offsets calibration in user's memory
    HAL_AdcOffsetSelfCal(handle);

    // run oscillator compensation
    HAL_OscTempComp(handle);

    // disable the ADC clock
    CLK_disableAdcClock(obj->clkHandle);

    return;
}  // end of HAL_cal() function

void HAL_OscTempComp(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;
    uint16_t Temperature;

    // disable the ADCs
    ADC_disable(obj->adcHandle);

    // power up the bandgap circuit
    ADC_enableBandGap(obj->adcHandle);

    // set the ADC voltage reference source to internal
    ADC_setVoltRefSrc(obj->adcHandle, ADC_VoltageRefSrc_Int);

    // enable the ADC reference buffers
    ADC_enableRefBuffers(obj->adcHandle);

    // Set main clock scaling factor (max45MHz clock for the ADC module)
    ADC_setDivideSelect(obj->adcHandle, ADC_DivideSelect_ClkIn_by_2);

    // power up the ADCs
    ADC_powerUp(obj->adcHandle);

    // enable the ADCs
    ADC_enable(obj->adcHandle);

    // enable non-overlap mode
    ADC_enableNoOverlapMode(obj->adcHandle);

    // connect channel A5 internally to the temperature sensor
    ADC_setTempSensorSrc(obj->adcHandle, ADC_TempSensorSrc_Int);

    // set SOC0 channel select to ADCINA5
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_0, ADC_SocChanNumber_A5);

    // set SOC0 acquisition period to 26 ADCCLK
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_0,
                          ADC_SocSampleDelay_64_cycles);

    // connect ADCINT1 to EOC0
    ADC_setIntSrc(obj->adcHandle, ADC_IntNumber_1, ADC_IntSrc_EOC0);

    // clear ADCINT1 flag
    ADC_clearIntFlag(obj->adcHandle, ADC_IntNumber_1);

    // enable ADCINT1
    ADC_enableInt(obj->adcHandle, ADC_IntNumber_1);

    // force start of conversion on SOC0
    ADC_setSocFrc(obj->adcHandle, ADC_SocFrc_0);

    // wait for end of conversion
    while (ADC_getIntFlag(obj->adcHandle, ADC_IntNumber_1) == 0) {
    }

    // clear ADCINT1 flag
    ADC_clearIntFlag(obj->adcHandle, ADC_IntNumber_1);

    Temperature = ADC_readResult(obj->adcHandle, ADC_ResultNumber_0);

    HAL_osc1Comp(handle, Temperature);

    HAL_osc2Comp(handle, Temperature);

    return;
}  // end of HAL_OscTempComp() function

void HAL_osc1Comp(HAL_Handle handle, const int16_t sensorSample) {
    int16_t compOscFineTrim;
    HAL_Obj *obj = (HAL_Obj *)handle;

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    compOscFineTrim =
        ((sensorSample - getRefTempOffset()) * (int32_t)getOsc1FineTrimSlope() +
         OSC_POSTRIM_OFF + FP_ROUND) /
            FP_SCALE +
        getOsc1FineTrimOffset() - OSC_POSTRIM;

    if (compOscFineTrim > 31) {
        compOscFineTrim = 31;
    } else if (compOscFineTrim < -31) {
        compOscFineTrim = -31;
    }

    OSC_setTrim(obj->oscHandle, OSC_Number_1,
                HAL_getOscTrimValue(getOsc1CoarseTrim(), compOscFineTrim));

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
}  // end of HAL_osc1Comp() function

void HAL_osc2Comp(HAL_Handle handle, const int16_t sensorSample) {
    int16_t compOscFineTrim;
    HAL_Obj *obj = (HAL_Obj *)handle;

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    compOscFineTrim =
        ((sensorSample - getRefTempOffset()) * (int32_t)getOsc2FineTrimSlope() +
         OSC_POSTRIM_OFF + FP_ROUND) /
            FP_SCALE +
        getOsc2FineTrimOffset() - OSC_POSTRIM;

    if (compOscFineTrim > 31) {
        compOscFineTrim = 31;
    } else if (compOscFineTrim < -31) {
        compOscFineTrim = -31;
    }

    OSC_setTrim(obj->oscHandle, OSC_Number_2,
                HAL_getOscTrimValue(getOsc2CoarseTrim(), compOscFineTrim));

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
}  // end of HAL_osc2Comp() function

uint16_t HAL_getOscTrimValue(int16_t coarse, int16_t fine) {
    uint16_t regValue = 0;

    if (fine < 0) {
        regValue = ((-fine) | 0x20) << 9;
    } else {
        regValue = fine << 9;
    }

    if (coarse < 0) {
        regValue |= ((-coarse) | 0x80);
    } else {
        regValue |= coarse;
    }

    return regValue;
}  // end of HAL_getOscTrimValue() function

void HAL_AdcOffsetSelfCal(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;
    uint16_t AdcConvMean;

    // disable the ADCs
    ADC_disable(obj->adcHandle);

    // power up the bandgap circuit
    ADC_enableBandGap(obj->adcHandle);

    // set the ADC voltage reference source to internal
    ADC_setVoltRefSrc(obj->adcHandle, ADC_VoltageRefSrc_Int);

    // enable the ADC reference buffers
    ADC_enableRefBuffers(obj->adcHandle);

    // power up the ADCs
    ADC_powerUp(obj->adcHandle);

    // enable the ADCs
    ADC_enable(obj->adcHandle);

    // Select VREFLO internal connection on B5
    ADC_enableVoltRefLoConv(obj->adcHandle);

    // Select channel B5 for all SOC
    HAL_AdcCalChanSelect(handle, ADC_SocChanNumber_B5);

    // Apply artificial offset (+80) to account for a negative offset that may
    // reside in the ADC core
    ADC_setOffTrim(obj->adcHandle, 80);

    // Capture ADC conversion on VREFLO
    AdcConvMean = HAL_AdcCalConversion(handle);

    // Set offtrim register with new value (i.e remove artical offset (+80) and
    // create a two's compliment of the offset error)
    ADC_setOffTrim(obj->adcHandle, 80 - AdcConvMean);

    // Select external ADCIN5 input pin on B5
    ADC_disableVoltRefLoConv(obj->adcHandle);

    return;
}  // end of HAL_AdcOffsetSelfCal() function

void HAL_AdcCalChanSelect(HAL_Handle handle,
                          const ADC_SocChanNumber_e chanNumber) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_0, chanNumber);
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_1, chanNumber);
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_2, chanNumber);
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_3, chanNumber);
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_4, chanNumber);
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_5, chanNumber);
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_6, chanNumber);
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_7, chanNumber);
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_8, chanNumber);
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_9, chanNumber);
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_10, chanNumber);
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_11, chanNumber);
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_12, chanNumber);
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_13, chanNumber);
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_14, chanNumber);
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_15, chanNumber);

    return;
}  // end of HAL_AdcCalChanSelect() function

uint16_t HAL_AdcCalConversion(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;
    uint16_t index, SampleSize, Mean;
    uint32_t Sum;
    ADC_SocSampleDelay_e ACQPS_Value;

    index = 0;         // initialize index to 0
    SampleSize = 256;  // set sample size to 256 (**NOTE: Sample size must be
                       // multiples of 2^x where is an integer >= 4)
    Sum = 0;           // set sum to 0
    Mean = 999;        // initialize mean to known value

    // Set the ADC sample window to the desired value (Sample window = ACQPS + 1)
    ACQPS_Value = ADC_SocSampleDelay_7_cycles;

    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_0, ACQPS_Value);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_1, ACQPS_Value);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_2, ACQPS_Value);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_3, ACQPS_Value);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_4, ACQPS_Value);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_5, ACQPS_Value);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_6, ACQPS_Value);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_7, ACQPS_Value);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_8, ACQPS_Value);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_9, ACQPS_Value);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_10, ACQPS_Value);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_11, ACQPS_Value);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_12, ACQPS_Value);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_13, ACQPS_Value);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_14, ACQPS_Value);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_15, ACQPS_Value);

    // Enabled ADCINT1 and ADCINT2
    ADC_enableInt(obj->adcHandle, ADC_IntNumber_1);
    ADC_enableInt(obj->adcHandle, ADC_IntNumber_2);

    // Disable continuous sampling for ADCINT1 and ADCINT2
    ADC_setIntMode(obj->adcHandle, ADC_IntNumber_1, ADC_IntMode_EOC);
    ADC_setIntMode(obj->adcHandle, ADC_IntNumber_2, ADC_IntMode_EOC);

    // ADCINTs trigger at end of conversion
    ADC_setIntPulseGenMode(obj->adcHandle, ADC_IntPulseGenMode_Prior);

    // Setup ADCINT1 and ADCINT2 trigger source
    ADC_setIntSrc(obj->adcHandle, ADC_IntNumber_1, ADC_IntSrc_EOC6);
    ADC_setIntSrc(obj->adcHandle, ADC_IntNumber_2, ADC_IntSrc_EOC14);

    // Setup each SOC's ADCINT trigger source
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_0, ADC_Int2TriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_1, ADC_Int2TriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_2, ADC_Int2TriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_3, ADC_Int2TriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_4, ADC_Int2TriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_5, ADC_Int2TriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_6, ADC_Int2TriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_7, ADC_Int2TriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_8, ADC_Int1TriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_9, ADC_Int1TriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_10, ADC_Int1TriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_11, ADC_Int1TriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_12, ADC_Int1TriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_13, ADC_Int1TriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_14, ADC_Int1TriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_15, ADC_Int1TriggersSOC);

    // Delay before converting ADC channels
    usDelay(ADC_DELAY_usec);

    ADC_setSocFrcWord(obj->adcHandle, 0x00FF);

    while (index < SampleSize) {
        // Wait for ADCINT1 to trigger, then add ADCRESULT0-7 registers to sum
        while (ADC_getIntFlag(obj->adcHandle, ADC_IntNumber_1) == 0) {
        }

        // Must clear ADCINT1 flag since INT1CONT = 0
        ADC_clearIntFlag(obj->adcHandle, ADC_IntNumber_1);

        Sum += ADC_readResult(obj->adcHandle, ADC_ResultNumber_0);
        Sum += ADC_readResult(obj->adcHandle, ADC_ResultNumber_1);
        Sum += ADC_readResult(obj->adcHandle, ADC_ResultNumber_2);
        Sum += ADC_readResult(obj->adcHandle, ADC_ResultNumber_3);
        Sum += ADC_readResult(obj->adcHandle, ADC_ResultNumber_4);
        Sum += ADC_readResult(obj->adcHandle, ADC_ResultNumber_5);
        Sum += ADC_readResult(obj->adcHandle, ADC_ResultNumber_6);
        Sum += ADC_readResult(obj->adcHandle, ADC_ResultNumber_7);

        // Wait for ADCINT2 to trigger, then add ADCRESULT8-15 registers to sum
        while (ADC_getIntFlag(obj->adcHandle, ADC_IntNumber_2) == 0) {
        }

        // Must clear ADCINT2 flag since INT2CONT = 0
        ADC_clearIntFlag(obj->adcHandle, ADC_IntNumber_2);

        Sum += ADC_readResult(obj->adcHandle, ADC_ResultNumber_8);
        Sum += ADC_readResult(obj->adcHandle, ADC_ResultNumber_9);
        Sum += ADC_readResult(obj->adcHandle, ADC_ResultNumber_10);
        Sum += ADC_readResult(obj->adcHandle, ADC_ResultNumber_11);
        Sum += ADC_readResult(obj->adcHandle, ADC_ResultNumber_12);
        Sum += ADC_readResult(obj->adcHandle, ADC_ResultNumber_13);
        Sum += ADC_readResult(obj->adcHandle, ADC_ResultNumber_14);
        Sum += ADC_readResult(obj->adcHandle, ADC_ResultNumber_15);

        index += 16;

    }  // end data collection

    // Disable ADCINT1 and ADCINT2 to STOP the ping-pong sampling
    ADC_disableInt(obj->adcHandle, ADC_IntNumber_1);
    ADC_disableInt(obj->adcHandle, ADC_IntNumber_2);

    // Calculate average ADC sample value
    Mean = Sum / SampleSize;

    // Clear start of conversion trigger
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_0, ADC_NoIntTriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_1, ADC_NoIntTriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_2, ADC_NoIntTriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_3, ADC_NoIntTriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_4, ADC_NoIntTriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_5, ADC_NoIntTriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_6, ADC_NoIntTriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_7, ADC_NoIntTriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_8, ADC_NoIntTriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_9, ADC_NoIntTriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_10, ADC_NoIntTriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_11, ADC_NoIntTriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_12, ADC_NoIntTriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_13, ADC_NoIntTriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_14, ADC_NoIntTriggersSOC);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_15, ADC_NoIntTriggersSOC);

    // return the average
    return (Mean);
}  // end of HAL_AdcCalConversion() function

void HAL_disableWdog(HAL_Handle halHandle) {
    HAL_Obj *hal = (HAL_Obj *)halHandle;

    WDOG_disable(hal->wdogHandle);

    return;
}  // end of HAL_disableWdog() function

void HAL_disableGlobalInts(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    CPU_disableGlobalInts(obj->cpuHandle);

    return;
}  // end of HAL_disableGlobalInts() function

void HAL_enableAdcInts(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    // enable the PIE interrupts associated with the ADC interrupts
    PIE_enableAdcInt(obj->pieHandle, ADC_IntNumber_1);

    // enable the ADC interrupts
    ADC_enableInt(obj->adcHandle, ADC_IntNumber_1);

    // enable the cpu interrupt for ADC interrupts
    CPU_enableInt(obj->cpuHandle, CPU_IntNumber_10);

    return;
}  // end of HAL_enableAdcInts() function

void HAL_enableDebugInt(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    CPU_enableDebugInt(obj->cpuHandle);

    return;
}  // end of HAL_enableDebugInt() function

void HAL_enableDrv(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    DRV8305_enable(obj->drv8305Handle);

    return;
}  // end of HAL_enableDrv() function

void HAL_enableGlobalInts(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    CPU_enableGlobalInts(obj->cpuHandle);

    return;
}  // end of HAL_enableGlobalInts() function

void HAL_enablePwmInt(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    PIE_enablePwmInt(obj->pieHandle, PWM_Number_1);

    // enable the interrupt
    PWM_enableInt(obj->pwmHandle[PWM_Number_1]);

    // enable the cpu interrupt for EPWM1_INT
    CPU_enableInt(obj->cpuHandle, CPU_IntNumber_3);

    return;
}  // end of HAL_enablePwmInt() function

void HAL_setupFaults(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;
    uint_least8_t cnt;

    // Configure Trip Mechanism for the Motor control software
    // -Cycle by cycle trip on CPU halt
    // -One shot fault trip zone
    // These trips need to be repeated for EPWM1 ,2 & 3

    for (cnt = 0; cnt < 3; cnt++) {
        PWM_enableTripZoneSrc(obj->pwmHandle[cnt],
                              PWM_TripZoneSrc_CycleByCycle_TZ6_NOT);

        PWM_enableTripZoneSrc(obj->pwmHandle[cnt],
                              PWM_TripZoneSrc_CycleByCycle_TZ1_NOT);

        // What do we want the OST/CBC events to do?
        // TZA events can force EPWMxA
        // TZB events can force EPWMxB

        PWM_setTripZoneState_TZA(obj->pwmHandle[cnt], PWM_TripZoneState_EPWM_Low);
        PWM_setTripZoneState_TZB(obj->pwmHandle[cnt], PWM_TripZoneState_EPWM_Low);
    }

    return;
}  // end of HAL_setupFaults() function

HAL_Handle HAL_init(void *pMemory, const size_t numBytes) {
    uint_least8_t cnt;
    HAL_Handle handle;
    HAL_Obj *obj;

    if (numBytes < sizeof(HAL_Obj))
        return ((HAL_Handle)NULL);

    // assign the handle
    handle = (HAL_Handle)pMemory;

    // assign the object
    obj = (HAL_Obj *)handle;

    // initialize the watchdog driver
    obj->wdogHandle = WDOG_init((void *)WDOG_BASE_ADDR, sizeof(WDOG_Obj));

    // disable watchdog
    HAL_disableWdog(handle);

    // initialize the ADC
    obj->adcHandle = ADC_init((void *)ADC_BASE_ADDR, sizeof(ADC_Obj));

    // Added by Dmitri Ranfft on 16.09.2015
#ifdef CAP
    // Initialize the eCAP
    obj->capHandle = CAP_init((void *)CAP1_BASE_ADDR, sizeof(CAP_Obj));
#endif

    // initialize the clock handle
    obj->clkHandle = CLK_init((void *)CLK_BASE_ADDR, sizeof(CLK_Obj));

    // initialize the CPU handle
    obj->cpuHandle = CPU_init(&cpu, sizeof(cpu));

    // initialize the FLASH handle
    obj->flashHandle = FLASH_init((void *)FLASH_BASE_ADDR, sizeof(FLASH_Obj));

    // initialize the GPIO handle
    obj->gpioHandle = GPIO_init((void *)GPIO_BASE_ADDR, sizeof(GPIO_Obj));

    // initialize the current offset estimator handles
    for (cnt = 0; cnt < USER_NUM_CURRENT_SENSORS; cnt++) {
        obj->offsetHandle_I[cnt] =
            OFFSET_init(&obj->offset_I[cnt], sizeof(obj->offset_I[cnt]));
    }

    // initialize the voltage offset estimator handles
    for (cnt = 0; cnt < USER_NUM_VOLTAGE_SENSORS; cnt++) {
        obj->offsetHandle_V[cnt] =
            OFFSET_init(&obj->offset_V[cnt], sizeof(obj->offset_V[cnt]));
    }

    // initialize the oscillator handle
    obj->oscHandle = OSC_init((void *)OSC_BASE_ADDR, sizeof(OSC_Obj));

    // initialize the PIE handle
    obj->pieHandle = PIE_init((void *)PIE_BASE_ADDR, sizeof(PIE_Obj));

    // initialize the PLL handle
    obj->pllHandle = PLL_init((void *)PLL_BASE_ADDR, sizeof(PLL_Obj));

// initialize the SCI handle
#ifdef SCI
    // Init SCI A registers
    obj->sciaHandle = SCI_init((void *)SCIA_BASE_ADDR, sizeof(SCI_Obj));
#endif

    // initialize the SPIA handle
    obj->spiAHandle = SPI_init((void *)SPIA_BASE_ADDR, sizeof(SPI_Obj));

    // initialize PWM handle
    obj->pwmHandle[0] = PWM_init((void *)PWM_ePWM1_BASE_ADDR, sizeof(PWM_Obj));
    obj->pwmHandle[1] = PWM_init((void *)PWM_ePWM2_BASE_ADDR, sizeof(PWM_Obj));
    obj->pwmHandle[2] = PWM_init((void *)PWM_ePWM4_BASE_ADDR, sizeof(PWM_Obj));

    // initialize power handle
    obj->pwrHandle = PWR_init((void *)PWR_BASE_ADDR, sizeof(PWR_Obj));

    // initialize timer drivers
    obj->timerHandle[0] = TIMER_init((void *)TIMER0_BASE_ADDR, sizeof(TIMER_Obj));
    obj->timerHandle[1] = TIMER_init((void *)TIMER1_BASE_ADDR, sizeof(TIMER_Obj));
    obj->timerHandle[2] = TIMER_init((void *)TIMER2_BASE_ADDR, sizeof(TIMER_Obj));

    // initialize drv8305 interface
    obj->drv8305Handle = DRV8305_init(&obj->drv8305, sizeof(obj->drv8305));

#ifdef MCP2515
    // initialize MCP2515 interface
    obj->mcp2515Handle =
        MCP2515_init(&obj->mcp2515, sizeof(obj->mcp2515));  // init Handle
#endif

#ifdef EEPROM_25AA02
    obj->eeprom25aa02Handle =
        EEPROM25AA02_init(&obj->eeprom25aa02, sizeof(obj->eeprom25aa02));
#endif

    return (handle);
}  // end of HAL_init() function

void HAL_setParams(HAL_Handle handle, const USER_Params *pUserParams) {
    uint_least8_t cnt;
    HAL_Obj *obj = (HAL_Obj *)handle;
    _iq beta_lp_pu =
        _IQ(pUserParams->offsetPole_rps / (float_t)pUserParams->ctrlFreq_Hz);

    HAL_setNumCurrentSensors(handle, pUserParams->numCurrentSensors);
    HAL_setNumVoltageSensors(handle, pUserParams->numVoltageSensors);

    for (cnt = 0; cnt < HAL_getNumCurrentSensors(handle); cnt++) {
        HAL_setOffsetBeta_lp_pu(handle, HAL_SensorType_Current, cnt, beta_lp_pu);
        HAL_setOffsetInitCond(handle, HAL_SensorType_Current, cnt, _IQ(0.0));
        HAL_setOffsetValue(handle, HAL_SensorType_Current, cnt, _IQ(0.0));
    }

    for (cnt = 0; cnt < HAL_getNumVoltageSensors(handle); cnt++) {
        HAL_setOffsetBeta_lp_pu(handle, HAL_SensorType_Voltage, cnt, beta_lp_pu);
        HAL_setOffsetInitCond(handle, HAL_SensorType_Voltage, cnt, _IQ(0.0));
        HAL_setOffsetValue(handle, HAL_SensorType_Voltage, cnt, _IQ(0.0));
    }

    // disable global interrupts
    CPU_disableGlobalInts(obj->cpuHandle);

    // disable cpu interrupts
    CPU_disableInts(obj->cpuHandle);

    // clear cpu interrupt flags
    CPU_clearIntFlags(obj->cpuHandle);

    // setup the clocks
    HAL_setupClks(handle);

    // Setup the PLL
    HAL_setupPll(handle, PLL_ClkFreq_60_MHz);

    // setup the PIE
    HAL_setupPie(handle);

    // run the device calibration
    HAL_cal(handle);

    // setup the peripheral clocks
    HAL_setupPeripheralClks(handle);

    // setup the GPIOs
    HAL_setupGpios(handle);

    // setup the flash
    HAL_setupFlash(handle);

    // setup the ADCs
    HAL_setupAdcs(handle);

    // setup the PWMs
    HAL_setupPwms(handle, (float_t)pUserParams->systemFreq_MHz,
                  pUserParams->pwmPeriod_usec, USER_NUM_PWM_TICKS_PER_ISR_TICK);

#ifndef SPI_TEST_MCP2515
    // setup the spiA
    HAL_setupSpiA(handle);
#else

    HAL_setupSpi_MCP2515(handle);
#endif

    // setup the timers
    HAL_setupTimers(handle, (float_t)pUserParams->systemFreq_MHz);

    // setup the drv8305 interface
    HAL_setupGate(handle);

#ifdef MCP2515
    // setup the mcp2515 interface
    HAL_setupMCP2515(handle);  // init PIN and Interrupt
#endif

    // set the default current bias
    {
        uint_least8_t cnt;
        _iq bias = _IQ12mpy(ADC_dataBias, _IQ(pUserParams->current_sf));

        for (cnt = 0; cnt < HAL_getNumCurrentSensors(handle); cnt++) {
            HAL_setBias(handle, HAL_SensorType_Current, cnt, bias);
        }
    }

    // setup the SCI interface
#ifdef SCI
    HAL_setupSCI(handle);
#endif

    // setup the eCAP interface. Added by Dmitri
#ifdef CAP
    HAL_setupCAP(handle);
#endif

    //  set the current scale factor
    {
        _iq current_sf = _IQ(pUserParams->current_sf);

        HAL_setCurrentScaleFactor(handle, current_sf);
    }

    // set the default voltage bias
    {
        uint_least8_t cnt;
        _iq bias = _IQ(0.0);

        for (cnt = 0; cnt < HAL_getNumVoltageSensors(handle); cnt++) {
            HAL_setBias(handle, HAL_SensorType_Voltage, cnt, bias);
        }
    }

    //  set the voltage scale factor
    {
        _iq voltage_sf = _IQ(pUserParams->voltage_sf);

        HAL_setVoltageScaleFactor(handle, voltage_sf);
    }

    return;
}  // end of HAL_setParams() function

void HAL_setupAdcs(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    // disable the ADCs
    ADC_disable(obj->adcHandle);

    // power up the bandgap circuit
    ADC_enableBandGap(obj->adcHandle);

    // set the ADC voltage reference source to internal
    ADC_setVoltRefSrc(obj->adcHandle, ADC_VoltageRefSrc_Int);

    // enable the ADC reference buffers
    ADC_enableRefBuffers(obj->adcHandle);

    // power up the ADCs
    ADC_powerUp(obj->adcHandle);

    // enable the ADCs
    ADC_enable(obj->adcHandle);

    // set the ADC interrupt pulse generation to prior
    ADC_setIntPulseGenMode(obj->adcHandle, ADC_IntPulseGenMode_Prior);

    // set the temperature sensor source to external
    ADC_setTempSensorSrc(obj->adcHandle, ADC_TempSensorSrc_Ext);

    // configure the interrupt sources
    ADC_disableInt(obj->adcHandle, ADC_IntNumber_1);
    ADC_setIntMode(obj->adcHandle, ADC_IntNumber_1, ADC_IntMode_ClearFlag);
    ADC_setIntSrc(obj->adcHandle, ADC_IntNumber_1, ADC_IntSrc_EOC7);

    // configure the SOCs for boostxldrv8305evm_revA
    //  sample the first sample twice due to errata sprz342f
    //  ISEN_A
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_0, ADC_SocChanNumber_B1);
    ADC_setSocTrigSrc(obj->adcHandle, ADC_SocNumber_0,
                      ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_0,
                          ADC_SocSampleDelay_7_cycles);

    // ISEN_A
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_1, ADC_SocChanNumber_B1);
    ADC_setSocTrigSrc(obj->adcHandle, ADC_SocNumber_1,
                      ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_1,
                          ADC_SocSampleDelay_7_cycles);

    // ISEN_B
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_2, ADC_SocChanNumber_B3);
    ADC_setSocTrigSrc(obj->adcHandle, ADC_SocNumber_2,
                      ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_2,
                          ADC_SocSampleDelay_7_cycles);

    // ISEN_C
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_3, ADC_SocChanNumber_B7);
    ADC_setSocTrigSrc(obj->adcHandle, ADC_SocNumber_3,
                      ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_3,
                          ADC_SocSampleDelay_7_cycles);

    // VSEN_A
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_4, ADC_SocChanNumber_A7);
    ADC_setSocTrigSrc(obj->adcHandle, ADC_SocNumber_4,
                      ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_4,
                          ADC_SocSampleDelay_7_cycles);

    // VSEN_B
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_5, ADC_SocChanNumber_A3);
    ADC_setSocTrigSrc(obj->adcHandle, ADC_SocNumber_5,
                      ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_5,
                          ADC_SocSampleDelay_7_cycles);

    // VSEN_C
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_6, ADC_SocChanNumber_A1);
    ADC_setSocTrigSrc(obj->adcHandle, ADC_SocNumber_6,
                      ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_6,
                          ADC_SocSampleDelay_7_cycles);

    // V_PVDD
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_7, ADC_SocChanNumber_A0);
    ADC_setSocTrigSrc(obj->adcHandle, ADC_SocNumber_7,
                      ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_7,
                          ADC_SocSampleDelay_7_cycles);

    // Humidity
    ADC_setSocChanNumber(obj->adcHandle, ADC_SocNumber_8, ADC_SocChanNumber_A6);
    ADC_setupSocTrigSrc(obj->adcHandle, ADC_SocNumber_8, ADC_Int1TriggersSOC);
    ADC_setSocSampleDelay(obj->adcHandle, ADC_SocNumber_8,
                          ADC_SocSampleDelay_9_cycles);
    return;
}  // end of HAL_setupAdcs() function

void HAL_setupClks(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    // enable internal oscillator 1
    CLK_enableOsc1(obj->clkHandle);

    // set the oscillator source
    CLK_setOscSrc(obj->clkHandle, CLK_OscSrc_Internal);

    // disable the external clock in
    CLK_disableClkIn(obj->clkHandle);

    // disable the crystal oscillator
    CLK_disableCrystalOsc(obj->clkHandle);

    // disable oscillator 2
    CLK_disableOsc2(obj->clkHandle);

    // set the low speed clock prescaler
    CLK_setLowSpdPreScaler(obj->clkHandle, CLK_LowSpdPreScaler_SysClkOut_by_4);

    // set the clock out prescaler
    CLK_setClkOutPreScaler(obj->clkHandle, CLK_ClkOutPreScaler_SysClkOut_by_1);

    return;
}  // end of HAL_setupClks() function

void HAL_setupFlash(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    FLASH_enablePipelineMode(obj->flashHandle);

    FLASH_setNumPagedReadWaitStates(obj->flashHandle, FLASH_NumPagedWaitStates_2);

    FLASH_setNumRandomReadWaitStates(obj->flashHandle,
                                     FLASH_NumRandomWaitStates_2);

    FLASH_setOtpWaitStates(obj->flashHandle, FLASH_NumOtpWaitStates_3);

    FLASH_setStandbyWaitCount(obj->flashHandle, FLASH_STANDBY_WAIT_COUNT_DEFAULT);

    FLASH_setActiveWaitCount(obj->flashHandle, FLASH_ACTIVE_WAIT_COUNT_DEFAULT);

    return;
}  // HAL_setupFlash() function

void HAL_setupGate(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    DRV8305_setSpiHandle(obj->drv8305Handle, obj->spiAHandle);
    DRV8305_setGpioHandle(obj->drv8305Handle, obj->gpioHandle);
    DRV8305_setGpioNumber(obj->drv8305Handle, GPIO_Number_4);
#ifdef CS_GPIO
    DRV8305_setSPI_CS_Number(obj->drv8305Handle, GPIO_Number_19);
#endif
}  // HAL_setupGate() function

#ifdef MCP2515
void HAL_setupMCP2515(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    MCP2515_setSpiHandle(obj->mcp2515Handle, obj->spiAHandle);
    MCP2515_setGpioHandle(obj->mcp2515Handle, obj->gpioHandle);
    MCP2515_setGpio_INT(obj->mcp2515Handle, GPIO_Number_28);
    MCP2515_setGpio_CS(obj->mcp2515Handle, GPIO_Number_29);

    PIE_enableInt(obj->pieHandle, PIE_GroupNumber_1, PIE_InterruptSource_XINT_1);
    CPU_enableInt(obj->cpuHandle, CPU_IntNumber_1);
    GPIO_setExtInt(obj->gpioHandle, GPIO_Number_28, CPU_ExtIntNumber_1);
    PIE_setExtIntPolarity(obj->pieHandle, CPU_ExtIntNumber_1,
                          PIE_ExtIntPolarity_FallingEdge);
    PIE_enableExtInt(obj->pieHandle, CPU_ExtIntNumber_1);
}  // HAL_setupMCP2515() function
#endif

#ifdef EEPROM_25AA02
void HAL_setupEEPROM25AA02(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    EEPROM25AA02_setSpiHandle(obj->eeprom25aa02Handle, obj->spiAHandle);
    EEPROM25AA02_setGpioHandle(obj->eeprom25aa02Handle, obj->gpioHandle);
    EEPROM25AA02_setGpio_CS(obj->eeprom25aa02Handle, GPIO_Number_34);
}
#endif

void HAL_setupGpios(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    // PWMA_H
    GPIO_setMode(obj->gpioHandle, GPIO_Number_0, GPIO_0_Mode_EPWM1A);

    // PWMA_L
    GPIO_setMode(obj->gpioHandle, GPIO_Number_1, GPIO_1_Mode_EPWM1B);

    // PWMB_H
    GPIO_setMode(obj->gpioHandle, GPIO_Number_2, GPIO_2_Mode_EPWM2A);

    // PWMB_L
    GPIO_setMode(obj->gpioHandle, GPIO_Number_3, GPIO_3_Mode_EPWM2B);

    // PWMC_H
    // Edited by Dmitri  (changed from EPWM3 to 4)
    GPIO_setMode(obj->gpioHandle, GPIO_Number_6, GPIO_6_Mode_EPWM4A);

    // PWMC_L
    // Edited by Dmitri  (changed from EPWM3 to 4)
    GPIO_setMode(obj->gpioHandle, GPIO_Number_7, GPIO_7_Mode_EPWM4B);

#ifdef CAP
    // eCAP1
    // Added by Dmitri
    // GPIO_setPullUp(obj->gpioHandle, GPIO_Number_5, GPIO_PullUp_Enable);
    // GPIO_setQualification(obj->gpioHandle, GPIO_Number_5, GPIO_Qual_Sync);
    GPIO_setMode(obj->gpioHandle, GPIO_Number_5, GPIO_5_Mode_ECAP1);
#endif

    // EN_GATE
    GPIO_setMode(obj->gpioHandle, GPIO_Number_4, GPIO_4_Mode_GeneralPurpose);
    GPIO_setLow(obj->gpioHandle, GPIO_Number_4);
    GPIO_setDirection(obj->gpioHandle, GPIO_Number_4, GPIO_Direction_Output);

    // WAKE
    GPIO_setMode(obj->gpioHandle, GPIO_Number_33, GPIO_33_Mode_GeneralPurpose);
    GPIO_setHigh(obj->gpioHandle, GPIO_Number_33);
    GPIO_setDirection(obj->gpioHandle, GPIO_Number_33, GPIO_Direction_Output);

    // No Connection
    GPIO_setMode(obj->gpioHandle, GPIO_Number_32, GPIO_32_Mode_GeneralPurpose);
    GPIO_setDirection(obj->gpioHandle, GPIO_Number_32, GPIO_Direction_Output);
    GPIO_setHigh(obj->gpioHandle, GPIO_Number_32);

    // SPI_SDI
    GPIO_setMode(obj->gpioHandle, GPIO_Number_16, GPIO_16_Mode_SPISIMOA);

    // SPI_SDO
    GPIO_setMode(obj->gpioHandle, GPIO_Number_17, GPIO_17_Mode_SPISOMIA);
    GPIO_setPullUp(obj->gpioHandle, GPIO_Number_17, GPIO_PullUp_Disable);

    //  GPIO_setMode(obj->gpioHandle,GPIO_Number_17,GPIO_17_Mode_GeneralPurpose);
    //  GPIO_setDirection(obj->gpioHandle, GPIO_Number_17, GPIO_Direction_Output);
    //  GPIO_setHigh(obj->gpioHandle, GPIO_Number_17);

    // SPI_CLK
    GPIO_setMode(obj->gpioHandle, GPIO_Number_18, GPIO_18_Mode_SPICLKA);

    // SPI_SCS
#ifdef CS_GPIO
    GPIO_setMode(obj->gpioHandle, GPIO_Number_19, GPIO_19_Mode_GeneralPurpose);
    GPIO_setDirection(obj->gpioHandle, GPIO_Number_19, GPIO_Direction_Output);
    GPIO_setHigh(obj->gpioHandle, GPIO_Number_19);
#else
    GPIO_setMode(obj->gpioHandle, GPIO_Number_19, GPIO_19_Mode_SPISTEA_NOT);
#endif
    // SPI_SCS_EPROM
    GPIO_setMode(obj->gpioHandle, GPIO_Number_34, GPIO_34_Mode_GeneralPurpose);
    GPIO_setDirection(obj->gpioHandle, GPIO_Number_34, GPIO_Direction_Output);
    // GPIO_setLow(obj->gpioHandle, GPIO_Number_34);
    GPIO_setHigh(obj->gpioHandle, GPIO_Number_34);

    // nFAULT
    GPIO_setMode(obj->gpioHandle, GPIO_Number_12, GPIO_12_Mode_TZ1_NOT);

#ifdef SCI
    // SCI_RX
    GPIO_setMode(obj->gpioHandle, GPIO_Number_28, GPIO_28_Mode_SCIRXDA);

    // SCI_TX
    GPIO_setMode(obj->gpioHandle, GPIO_Number_29, GPIO_29_Mode_SCITXDA);
#endif

#ifdef MCP2515
    // MCP2515_CS
    GPIO_setMode(obj->gpioHandle, GPIO_Number_29, GPIO_29_Mode_GeneralPurpose);
    GPIO_setDirection(obj->gpioHandle, GPIO_Number_29, GPIO_Direction_Output);
    GPIO_setHigh(obj->gpioHandle, GPIO_Number_29);

    // MCP2515_INT
    GPIO_setMode(obj->gpioHandle, GPIO_Number_28, GPIO_28_Mode_GeneralPurpose);
    GPIO_setDirection(obj->gpioHandle, GPIO_Number_28, GPIO_Direction_Input);
    // GPIO_setPullUp(obj->gpioHandle, GPIO_Number_28, GPIO_PullUp_Enable);
    GPIO_setPullUp(obj->gpioHandle, GPIO_Number_28, GPIO_PullUp_Disable);
    GPIO_setQualification(obj->gpioHandle, GPIO_Number_28, GPIO_Qual_Sync);

#endif

    // JTAG
    GPIO_setMode(obj->gpioHandle, GPIO_Number_35, GPIO_35_Mode_JTAG_TDI);
    GPIO_setMode(obj->gpioHandle, GPIO_Number_36, GPIO_36_Mode_JTAG_TMS);
    GPIO_setMode(obj->gpioHandle, GPIO_Number_37, GPIO_37_Mode_JTAG_TDO);
    GPIO_setMode(obj->gpioHandle, GPIO_Number_38, GPIO_38_Mode_JTAG_TCK);

    return;
}  // end of HAL_setupGpios() function

void HAL_setupPie(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    PIE_disable(obj->pieHandle);

    PIE_disableAllInts(obj->pieHandle);

    PIE_clearAllInts(obj->pieHandle);

    PIE_clearAllFlags(obj->pieHandle);

    PIE_setDefaultIntVectorTable(obj->pieHandle);

    PIE_enable(obj->pieHandle);

    return;
}  // end of HAL_setupPie() function

void HAL_setupPeripheralClks(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    CLK_enableAdcClock(obj->clkHandle);

    CLK_enableCompClock(obj->clkHandle, CLK_CompNumber_1);
    CLK_enableCompClock(obj->clkHandle, CLK_CompNumber_2);
    CLK_enableCompClock(obj->clkHandle, CLK_CompNumber_3);

#ifdef CAP
    CLK_enableEcap1Clock(obj->clkHandle);
#endif

    CLK_enablePwmClock(obj->clkHandle, PWM_Number_1);
    CLK_enablePwmClock(obj->clkHandle, PWM_Number_2);

#ifndef CAP
    CLK_enablePwmClock(obj->clkHandle, PWM_Number_3);
#endif

    CLK_enablePwmClock(obj->clkHandle, PWM_Number_4);

    CLK_disableHrPwmClock(obj->clkHandle);

    CLK_disableI2cClock(obj->clkHandle);

#ifdef SCI
    CLK_enableSciaClock(obj->clkHandle);
#endif
    CLK_enableSpiaClock(obj->clkHandle);

    CLK_enableTbClockSync(obj->clkHandle);

    return;
}  // end of HAL_setupPeripheralClks() function

void HAL_setupPll(HAL_Handle handle, const PLL_ClkFreq_e clkFreq) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    // make sure PLL is not running in limp mode
    if (PLL_getClkStatus(obj->pllHandle) != PLL_ClkStatus_Normal) {
        // reset the clock detect
        PLL_resetClkDetect(obj->pllHandle);

        // ???????
        asm("        ESTOP0");
    }

    // Divide Select must be ClkIn/4 before the clock rate can be changed
    if (PLL_getDivideSelect(obj->pllHandle) != PLL_DivideSelect_ClkIn_by_4) {
        PLL_setDivideSelect(obj->pllHandle, PLL_DivideSelect_ClkIn_by_4);
    }

    if (PLL_getClkFreq(obj->pllHandle) != clkFreq) {
        // disable the clock detect
        PLL_disableClkDetect(obj->pllHandle);

        // set the clock rate
        PLL_setClkFreq(obj->pllHandle, clkFreq);
    }

    // wait until locked
    while (PLL_getLockStatus(obj->pllHandle) != PLL_LockStatus_Done) {
    }

    // enable the clock detect
    PLL_enableClkDetect(obj->pllHandle);

    // set divide select to ClkIn/2 to get desired clock rate
    // NOTE: clock must be locked before setting this register
    PLL_setDivideSelect(obj->pllHandle, PLL_DivideSelect_ClkIn_by_2);

    return;
}  // end of HAL_setupPll() function

void HAL_setupPwms(HAL_Handle handle, const float_t systemFreq_MHz,
                   const float_t pwmPeriod_usec,
                   const uint_least16_t numPwmTicksPerIsrTick) {
    HAL_Obj *obj = (HAL_Obj *)handle;
    uint16_t halfPeriod_cycles = (uint16_t)(systemFreq_MHz * pwmPeriod_usec) >> 1;
    uint_least8_t cnt;

    // turns off the outputs of the EPWM peripherals which will put the power
    // switches into a high impedance state.
    PWM_setOneShotTrip(obj->pwmHandle[PWM_Number_1]);
    PWM_setOneShotTrip(obj->pwmHandle[PWM_Number_2]);
    PWM_setOneShotTrip(obj->pwmHandle[PWM_Number_3]);

    for (cnt = 0; cnt < 3; cnt++) {
        // setup the Time-Base Control Register (TBCTL)
        PWM_setCounterMode(obj->pwmHandle[cnt], PWM_CounterMode_UpDown);
        PWM_disableCounterLoad(obj->pwmHandle[cnt]);
        PWM_setPeriodLoad(obj->pwmHandle[cnt], PWM_PeriodLoad_Immediate);
        PWM_setSyncMode(obj->pwmHandle[cnt], PWM_SyncMode_EPWMxSYNC);
        PWM_setHighSpeedClkDiv(obj->pwmHandle[cnt], PWM_HspClkDiv_by_1);
        PWM_setClkDiv(obj->pwmHandle[cnt], PWM_ClkDiv_by_1);
        PWM_setPhaseDir(obj->pwmHandle[cnt], PWM_PhaseDir_CountUp);
        PWM_setRunMode(obj->pwmHandle[cnt], PWM_RunMode_FreeRun);

        // setup the Timer-Based Phase Register (TBPHS)
        PWM_setPhase(obj->pwmHandle[cnt], 0);

        // setup the Time-Base Counter Register (TBCTR)
        PWM_setCount(obj->pwmHandle[cnt], 0);

        // setup the Time-Base Period Register (TBPRD)
        // set to zero initially
        PWM_setPeriod(obj->pwmHandle[cnt], 0);

        // setup the Counter-Compare Control Register (CMPCTL)
        PWM_setLoadMode_CmpA(obj->pwmHandle[cnt], PWM_LoadMode_Zero);
        PWM_setLoadMode_CmpB(obj->pwmHandle[cnt], PWM_LoadMode_Zero);
        PWM_setShadowMode_CmpA(obj->pwmHandle[cnt], PWM_ShadowMode_Shadow);
        PWM_setShadowMode_CmpB(obj->pwmHandle[cnt], PWM_ShadowMode_Immediate);

        // setup the Action-Qualifier Output A Register (AQCTLA)
        PWM_setActionQual_CntUp_CmpA_PwmA(obj->pwmHandle[cnt], PWM_ActionQual_Set);
        PWM_setActionQual_CntDown_CmpA_PwmA(obj->pwmHandle[cnt],
                                            PWM_ActionQual_Clear);

        // setup the Dead-Band Generator Control Register (DBCTL)
        PWM_setDeadBandOutputMode(
            obj->pwmHandle[cnt],
            PWM_DeadBandOutputMode_EPWMxA_Rising_EPWMxB_Falling);
        PWM_setDeadBandPolarity(obj->pwmHandle[cnt],
                                PWM_DeadBandPolarity_EPWMxB_Inverted);

        // setup the Dead-Band Rising Edge Delay Register (DBRED)
        PWM_setDeadBandRisingEdgeDelay(obj->pwmHandle[cnt], HAL_PWM_DBRED_CNT);

        // setup the Dead-Band Falling Edge Delay Register (DBFED)
        PWM_setDeadBandFallingEdgeDelay(obj->pwmHandle[cnt], HAL_PWM_DBFED_CNT);
        // setup the PWM-Chopper Control Register (PCCTL)
        PWM_disableChopping(obj->pwmHandle[cnt]);

        // setup the Trip Zone Select Register (TZSEL)
        PWM_disableTripZones(obj->pwmHandle[cnt]);
    }

    // setup the Event Trigger Selection Register (ETSEL)
    PWM_disableInt(obj->pwmHandle[PWM_Number_1]);
    PWM_setSocAPulseSrc(obj->pwmHandle[PWM_Number_1],
                        PWM_SocPulseSrc_CounterEqualZero);
    PWM_enableSocAPulse(obj->pwmHandle[PWM_Number_1]);

    // setup the Event Trigger Prescale Register (ETPS)
    if (numPwmTicksPerIsrTick == 3) {
        PWM_setIntPeriod(obj->pwmHandle[PWM_Number_1], PWM_IntPeriod_ThirdEvent);
        PWM_setSocAPeriod(obj->pwmHandle[PWM_Number_1], PWM_SocPeriod_ThirdEvent);
    } else if (numPwmTicksPerIsrTick == 2) {
        PWM_setIntPeriod(obj->pwmHandle[PWM_Number_1], PWM_IntPeriod_SecondEvent);
        PWM_setSocAPeriod(obj->pwmHandle[PWM_Number_1], PWM_SocPeriod_SecondEvent);
    } else {
        PWM_setIntPeriod(obj->pwmHandle[PWM_Number_1], PWM_IntPeriod_FirstEvent);
        PWM_setSocAPeriod(obj->pwmHandle[PWM_Number_1], PWM_SocPeriod_FirstEvent);
    }

    // setup the Event Trigger Clear Register (ETCLR)
    PWM_clearIntFlag(obj->pwmHandle[PWM_Number_1]);
    PWM_clearSocAFlag(obj->pwmHandle[PWM_Number_1]);

    // first step to synchronize the pwms
    CLK_disableTbClockSync(obj->clkHandle);

    // since the PWM is configured as an up/down counter, the period register is
    // set to one-half of the desired PWM period
    PWM_setPeriod(obj->pwmHandle[PWM_Number_1], halfPeriod_cycles);
    PWM_setPeriod(obj->pwmHandle[PWM_Number_2], halfPeriod_cycles);
    PWM_setPeriod(obj->pwmHandle[PWM_Number_3], halfPeriod_cycles);

    // last step to synchronize the pwms
    CLK_enableTbClockSync(obj->clkHandle);

    return;
}  // end of HAL_setupPwms() function

#ifdef CS_GPIO

void HAL_setupSpiA(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    SPI_reset(obj->spiAHandle);
    SPI_setMode(obj->spiAHandle, SPI_Mode_Master);
    SPI_setClkPolarity(obj->spiAHandle,
                       SPI_ClkPolarity_OutputRisingEdge_InputFallingEdge);
    SPI_setClkPhase(obj->spiAHandle, SPI_ClkPhase_Normal);
    SPI_enableTx(obj->spiAHandle);
    SPI_disableTxFifoEnh(obj->spiAHandle);
    SPI_disableRxFifoInt(obj->spiAHandle);
    //  SPI_setTxDelay(obj->spiAHandle,0x0010); //???????????????????
    //  SPI_enableInt(obj->spiAHandle);
    SPI_setBaudRate(obj->spiAHandle, (SPI_BaudRate_e)(0x0001));  // 1
    SPI_setCharLength(obj->spiAHandle, SPI_CharLength_16_Bits);
    SPI_setSuspend(obj->spiAHandle, SPI_TxSuspend_free);
    SPI_enable(obj->spiAHandle);

    return;
}  // end of HAL_setupSpiA() function
#ifdef MCP2515
void HAL_setupSpi_MCP2515(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    SPI_reset(obj->spiAHandle);
    SPI_setMode(obj->spiAHandle, SPI_Mode_Master);
    SPI_setClkPolarity(obj->spiAHandle,
                       SPI_ClkPolarity_OutputRisingEdge_InputFallingEdge);
    SPI_setClkPhase(obj->spiAHandle, SPI_ClkPhase_Delayed);
    SPI_enableTx(obj->spiAHandle);
    SPI_disableTxFifoEnh(obj->spiAHandle);
    SPI_disableRxFifoInt(obj->spiAHandle);
    SPI_enableInt(obj->spiAHandle);
    //  SPI_setTxDelay(obj->spiAHandle,0x0010); ////??????????????
    SPI_setBaudRate(obj->spiAHandle, (SPI_BaudRate_e)(0x0001));  // 1
    SPI_setCharLength(obj->spiAHandle, SPI_CharLength_8_Bits);
    SPI_setSuspend(obj->spiAHandle, SPI_TxSuspend_free);
    SPI_enable(obj->spiAHandle);

    return;
}  // end of HAL_setupSpi_MCP2515() function
#endif
#ifdef EEPROM_25AA02
void HAL_setupSpi_25AA02(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    SPI_reset(obj->spiAHandle);
    SPI_setMode(obj->spiAHandle, SPI_Mode_Master);
    SPI_setClkPolarity(obj->spiAHandle,
                       SPI_ClkPolarity_OutputRisingEdge_InputFallingEdge);
    SPI_setClkPhase(obj->spiAHandle, SPI_ClkPhase_Delayed);
    SPI_enableTx(obj->spiAHandle);
    SPI_disableTxFifoEnh(obj->spiAHandle);
    SPI_disableRxFifoInt(obj->spiAHandle);
    SPI_enableInt(obj->spiAHandle);
    //  SPI_setTxDelay(obj->spiAHandle,0x0010); ////??????????????
    SPI_setBaudRate(obj->spiAHandle, (SPI_BaudRate_e)(0x0001));  // 1
    SPI_setCharLength(obj->spiAHandle, SPI_CharLength_8_Bits);
    SPI_setSuspend(obj->spiAHandle, SPI_TxSuspend_free);
    SPI_enable(obj->spiAHandle);

    return;
}  // end of HAL_setupSpi_25AA02() function
#endif
#else
void HAL_setupSpiA(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    SPI_reset(obj->spiAHandle);
    SPI_setMode(obj->spiAHandle, SPI_Mode_Master);
    SPI_setClkPolarity(obj->spiAHandle,
                       SPI_ClkPolarity_OutputRisingEdge_InputFallingEdge);
    SPI_setClkPhase(obj->spiAHandle, SPI_ClkPhase_Normal);
    SPI_enableTx(obj->spiAHandle);
    SPI_disableTxFifoEnh(obj->spiAHandle);
    SPI_disableRxFifoInt(obj->spiAHandle);
    SPI_setTxDelay(obj->spiAHandle, 0x0010);  //???????????????????
    SPI_enableInt(obj->spiAHandle);
    SPI_setBaudRate(obj->spiAHandle, (SPI_BaudRate_e)(0x0001));  // 1
    SPI_setCharLength(obj->spiAHandle, SPI_CharLength_16_Bits);
    SPI_setSuspend(obj->spiAHandle, SPI_TxSuspend_free);
    SPI_enable(obj->spiAHandle);

    return;
}  // end of HAL_setupSpiA() function
#endif

void HAL_Spi_SW_DRV8305(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;
    SPI_reset(obj->spiAHandle);
    SPI_setClkPhase(obj->spiAHandle, SPI_ClkPhase_Normal);
    SPI_setCharLength(obj->spiAHandle, SPI_CharLength_16_Bits);
    SPI_enable(obj->spiAHandle);
    return;
}  // end of HAL_Spi_SW_DRV8305() function

void HAL_Spi_SW_MCP2515(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;
    SPI_reset(obj->spiAHandle);
    SPI_setClkPhase(obj->spiAHandle, SPI_ClkPhase_Delayed);
    SPI_setCharLength(obj->spiAHandle, SPI_CharLength_8_Bits);
    SPI_enable(obj->spiAHandle);
    return;
}  // end of HAL_Spi_SW_MCP2515() function

void HAL_Spi_SW_EEPROM25AA02(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;
    SPI_reset(obj->spiAHandle);
    SPI_setClkPhase(obj->spiAHandle, SPI_ClkPhase_Normal);
    SPI_setCharLength(obj->spiAHandle, SPI_CharLength_8_Bits);
    SPI_enable(obj->spiAHandle);
    return;
}  // end of HAL_Spi_SW_MCP2515() function

void HAL_setupTimers(HAL_Handle handle, const float_t systemFreq_MHz) {
    HAL_Obj *obj = (HAL_Obj *)handle;
    uint32_t timerPeriod_cnts = (uint32_t)(systemFreq_MHz * (float_t)1000000.0) - 1;

    // use timer 0 for frequency diagnostics
    TIMER_setDecimationFactor(obj->timerHandle[0], 0);
    TIMER_setEmulationMode(obj->timerHandle[0], TIMER_EmulationMode_RunFree);
    TIMER_setPeriod(obj->timerHandle[0], timerPeriod_cnts);
    TIMER_setPreScaler(obj->timerHandle[0], 0);

    // use timer 1 for CPU usage diagnostics
    TIMER_setDecimationFactor(obj->timerHandle[1], 0);
    TIMER_setEmulationMode(obj->timerHandle[1], TIMER_EmulationMode_RunFree);
    TIMER_setPeriod(obj->timerHandle[1], timerPeriod_cnts);
    TIMER_setPreScaler(obj->timerHandle[1], 0);

    // use timer 2 for general purpose
    TIMER_setDecimationFactor(obj->timerHandle[2], 0);
    TIMER_setEmulationMode(obj->timerHandle[2], TIMER_EmulationMode_StopAtZero);
    TIMER_setPeriod(obj->timerHandle[2], timerPeriod_cnts);
    TIMER_setPreScaler(obj->timerHandle[2], 0);

    return;
}  // end of HAL_setupTimers() function

#ifdef SCI
void HAL_setupSCI(HAL_Handle handle) {
    // SCIFFTX = 0xE040
    HAL_Obj *obj = (HAL_Obj *)handle;
    SCI_enableChannels(obj->sciaHandle);    // SCI reset
    SCI_enableTxFifoEnh(obj->sciaHandle);   // SCI FIFO transmit enable
    SCI_enableTxFifo(obj->sciaHandle);      // SCI FIFO transmit reset/reenable
    SCI_clearTxFifoInt(obj->sciaHandle);    // SCI clear FIFO transmit interrupt flag
    SCI_disableTxFifoInt(obj->sciaHandle);  // disable FIFO transmit interrupt
    SCI_setTxFifoIntLevel(obj->sciaHandle,
                          SCI_FifoLevel_Empty);  // FIFO interrupt level
    // SCIFFRX = 0x2041
    SCI_enableRxFifo(obj->sciaHandle);      // SCI FIFO receive reset/reenable
    SCI_clearRxFifoInt(obj->sciaHandle);    // SCI clear FIFO receive interrupt flag
    SCI_disableRxFifoInt(obj->sciaHandle);  // disable FIFO receive interrupt
    SCI_setRxFifoIntLevel(obj->sciaHandle,
                          SCI_FifoLevel_1_Word);  // FIFO interrupt level
    SCI_enableRxFifoInt(obj->sciaHandle);
    // SCI stop bit, parity, loopback, char bits, idle/address mode (SCICCR =
    // 0x07)
    SCI_setNumStopBits(obj->sciaHandle, SCI_NumStopBits_One);  // SCICCR bit 7
    SCI_setParity(obj->sciaHandle, SCI_Parity_Odd);            // SCICCR bit 6
    SCI_disableParity(obj->sciaHandle);                        // SCICCR bit 5
    // SCI_enableParity(obj->sciaHandle);
    SCI_disableLoopBack(obj->sciaHandle);                       // SCICCR bit 4
    SCI_setMode(obj->sciaHandle, SCI_Mode_IdleLine);            // SCICCR bit 3
    SCI_setCharLength(obj->sciaHandle, SCI_CharLength_8_Bits);  // SCICCR bit 0-2
    // TX enable, RX enable, RX ERR INT enable, SLEEP, TXWAKE (SCICTL1 = 0x03)
    SCI_disableRxErrorInt(obj->sciaHandle);  // SCICTL1 bit 6
    SCI_disable(obj->sciaHandle);            // SCICTL1 bit 5
    SCI_disableTxWake(obj->sciaHandle);      // SCICTL1 bit 3
    SCI_disableSleep(obj->sciaHandle);       // SCICTL1 bit 2
    SCI_enableTx(obj->sciaHandle);           // SCICTL1 bit 1
    SCI_enableRx(obj->sciaHandle);           // SCICTL1 bit 0
    // TXINT enable, RXINT enable, TXEMPTY, TXRDY (SCICTL2 = 0x03)
    SCI_enableRxInt(obj->sciaHandle);   // SCICTL2 bit 1
    SCI_disableTxInt(obj->sciaHandle);  // SCICTL2 bit 0
    // SCIH-SCIL BAUD - SCI_BAUD = (LSPCLK/(SCI_BRR*8)) - 1
    SCI_setBaudRate(obj->sciaHandle, SCI_BaudRate_9_6_kBaud);

    // Reset SCI
    SCI_enable(obj->sciaHandle);
    PIE_enableInt(obj->pieHandle, PIE_GroupNumber_9, PIE_InterruptSource_SCIARX);
    CPU_enableInt(obj->cpuHandle, CPU_IntNumber_9);  // enable CPU interrupt
    return;
}
#endif  // of SCI

#ifdef CAP
void HAL_setupCAP(HAL_Handle handle) {
    HAL_Obj *obj = (HAL_Obj *)handle;
    CAP_setModeCap(obj->capHandle);  // set mode to CAP
    // Disables counter synchronization
    CAP_disableSyncIn(obj->capHandle);
    // Sets the capture event polarity
    CAP_setCapEvtPolarity(obj->capHandle, CAP_Event_1, CAP_Polarity_Rising);
    // Sets the capture event polarity
    CAP_setCapEvtPolarity(obj->capHandle, CAP_Event_2, CAP_Polarity_Falling);
    // Sets the capture event counter reset configuration
    CAP_setCapEvtReset(obj->capHandle, CAP_Event_1, CAP_Reset_Disable);
    // Sets the capture event counter reset configuration (reset counting here)
    CAP_setCapEvtReset(obj->capHandle, CAP_Event_2, CAP_Reset_Enable);
    // continous timer
    CAP_setCapContinuous(obj->capHandle);
    // Set the stop/wrap mode to 2 events
    CAP_setStopWrap(obj->capHandle, CAP_Stop_Wrap_CEVT2);
    // Enables loading of CAP1-4 on capture event
    CAP_enableCaptureLoad(obj->capHandle);
    // Enables Time Stamp counter to running
    CAP_enableTimestampCounter(obj->capHandle);
    // Enables capture (CAP) interrupt source
    CAP_enableInt(obj->capHandle, CAP_Int_Type_CEVT2);
    // enable eCAP interrupt
    PIE_enableInt(obj->pieHandle, PIE_GroupNumber_4, PIE_InterruptSource_ECAP1);
    // enable CPU ECAP Group interrupts
    CPU_enableInt(obj->cpuHandle, CPU_IntNumber_4);
    return;
}  // end of HAL_setupCAP() function

#endif

void HAL_writeDrvData(HAL_Handle handle, DRV_SPI_8305_Vars_t *Spi_8305_Vars) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    DRV8305_writeData(obj->drv8305Handle, Spi_8305_Vars);

    return;
}  // end of HAL_writeDrvData() function

void HAL_readDrvData(HAL_Handle handle, DRV_SPI_8305_Vars_t *Spi_8305_Vars) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    DRV8305_readData(obj->drv8305Handle, Spi_8305_Vars);

    return;
}  // end of HAL_readDrvData() function

void HAL_setupDrvSpi(HAL_Handle handle, DRV_SPI_8305_Vars_t *Spi_8305_Vars) {
    HAL_Obj *obj = (HAL_Obj *)handle;

    DRV8305_setupSpi(obj->drv8305Handle, Spi_8305_Vars);

    return;
}  // end of HAL_setupDrvSpi() function

// end of file
