/*
Microchip 25AA02E48 library

Copyright (C) 2016 Dmitri Ranfft

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef MICROCHIP_25AA02E48_H
#define MICROCHIP_25AA02E48_H

#include "spi.h"
#include "sw/drivers/gpio/src/32b/f28x/f2802x/gpio.h"

#ifndef uint8_t
#define uint8_t uint_least8_t
#endif

typedef struct _EEPROM25AA02_Obj_ {
    SPI_Handle spiHandle;    //!< the handle for the serial peripheral interface
    GPIO_Handle gpioHandle;  //!< the gpio handle
    GPIO_Number_e gpio_CS;   //!< the gpio number that is connected to the 25AA02 CS SPI pin
} EEPROM25AA02_Obj;

typedef struct _EEPROM25AA02_Obj_ *EEPROM25AA02_Handle;

EEPROM25AA02_Handle EEPROM25AA02_init(void *pMemory, const size_t numBytes);
void EEPROM25AA02_setSpiHandle(EEPROM25AA02_Handle handle, SPI_Handle spiHandle);
void EEPROM25AA02_setGpioHandle(EEPROM25AA02_Handle handle, GPIO_Handle gpioHandle);
void EEPROM25AA02_setGpio_CS(EEPROM25AA02_Handle handle, GPIO_Number_e gpio_CS);
EEPROM25AA02_Handle EEPROM25AA02_begin(void *pMemory, const size_t numBytes, GPIO_Number_e gpio_CS, GPIO_Handle gpioHandle, SPI_Handle spiHandle);

uint16_t EEPROM25AA02_spiTransferByte(EEPROM25AA02_Handle handle, const uint16_t data);
uint8_t EEPROM25AA02_readStatus(EEPROM25AA02_Handle handle);
void EEPROM25AA02_writeStatus(EEPROM25AA02_Handle handle, uint8_t value);
uint8_t EEPROM25AA02_readRegister(EEPROM25AA02_Handle handle, uint8_t addr);
uint8_t EEPROM25AA02_readRegisterN(EEPROM25AA02_Handle handle, uint8_t addr, uint8_t *buffer, int len);
void EEPROM25AA02_getEUI48(EEPROM25AA02_Handle handle, uint8_t *buffer);
void EEPROM25AA02_getEUI64(EEPROM25AA02_Handle handle, uint8_t *buffer);
void EEPROM25AA02_writeRegister(EEPROM25AA02_Handle handle, uint8_t addr, uint8_t value);
void EEPROM25AA02_writeRegisterN(EEPROM25AA02_Handle handle, uint8_t addr, uint8_t *buffer, int len);

#endif  // MICROCHIP_25AA02E48_H
