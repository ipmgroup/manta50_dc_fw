#ifndef __ADC_TEMP_CONVERSION

// Useful definitions
#define ADC_FP_SCALE 32768       //Scale factor for Q15 fixed point numbers (2^15)
#define ADC_FP_ROUND ADC_FP_SCALE/2  //Added to Q15 numbers before converting to integer to round the number

// Amount to add to Q15 fixed point numbers to shift from Celsius to Kelvin
// (Converting guarantees number is positive, which makes rounding more efficient)
#define ADC_KELVIN 273
#define ADC_KELVIN_OFF (ADC_FP_SCALE * ADC_KELVIN)

// The folloing pointers to function calls are:
//Slope of temperature sensor (deg. C / ADC code).  Stored in fixed point Q15 format.
#define ADC_getTempSlope() (*(int (*)(void))0x3D7E80)()
//ADC code corresponding to temperature sensor output at 0 deg. C
#define ADC_getTempOffset() (*(int (*)(void))0x3D7E83)()

//! \brief     Converts a temperature sensor sample into a temperature in Celcius
//! \param[in] adcHandle  The analog-to-digital converter (ADC) object handle
//! \return    Temperature in degrees Celcius
inline int16_t ADC_getTemperatureC(ADC_Handle adcHandle, int16_t sensorSample)
{
    return ((sensorSample - ADC_getTempOffset())*(int32_t)ADC_getTempSlope() + ADC_FP_ROUND + ADC_KELVIN_OFF)/ADC_FP_SCALE - ADC_KELVIN;
}

//! \brief     Converts a temperature sensor sample into a temperature in Kelvin
//! \param[in] adcHandle  The analog-to-digital converter (ADC) object handle
//! \return    Temperature in degrees Kelvin
inline int16_t ADC_getTemperatureK(ADC_Handle adcHandle, int16_t sensorSample)
{
    return ((sensorSample - ADC_getTempOffset())*(int32_t)ADC_getTempSlope() + ADC_FP_ROUND + ADC_KELVIN_OFF)/ADC_FP_SCALE;
}

#endif
