#include "Microchip_25AA02E48.h"

#define READ_instruction 0x03     // 25AA02A's read command
#define WRITE_instruction 0x02    // 25AA02A's write command
#define READ_STATUS_instruction  0x05
#define WRITE_STATUS_instruction 0x01
#define WREN 0x06
#define WRDI 0x04

#define MAX_WAIT_CYCLES 600000    // Maximum number of cycles waiting for previous write operation to finish. Corresponds at least 10ms at 60MHz.

EEPROM25AA02_Handle EEPROM25AA02_init(void *pMemory, const size_t numBytes) {
	EEPROM25AA02_Handle handle;

	if(numBytes < sizeof(EEPROM25AA02_Obj)){
		return((EEPROM25AA02_Handle)NULL);
	}

	// assign the handle
	handle = (EEPROM25AA02_Handle)pMemory;

	return(handle);
}

void EEPROM25AA02_setSpiHandle(EEPROM25AA02_Handle handle, SPI_Handle spiHandle){
	EEPROM25AA02_Obj *obj = (EEPROM25AA02_Obj *)handle;

	// initialize the serial peripheral interface object
	obj->spiHandle = spiHandle;

	return;
}


void EEPROM25AA02_setGpioHandle(EEPROM25AA02_Handle handle, GPIO_Handle gpioHandle){
	EEPROM25AA02_Obj *obj = (EEPROM25AA02_Obj *)handle;

	// initialize the gpio interface object
	obj->gpioHandle = gpioHandle;

	return;
}


void EEPROM25AA02_setGpio_CS(EEPROM25AA02_Handle handle, GPIO_Number_e gpio_CS){
	EEPROM25AA02_Obj *obj = (EEPROM25AA02_Obj *)handle;

	// set CS pin
	obj->gpio_CS = gpio_CS;

	return;
}

EEPROM25AA02_Handle EEPROM25AA02_begin(void *pMemory, const size_t numBytes, GPIO_Number_e gpio_CS, GPIO_Handle gpioHandle, SPI_Handle spiHandle){
	EEPROM25AA02_Handle handle = EEPROM25AA02_init(pMemory, numBytes);

	if(handle != NULL){
		EEPROM25AA02_setSpiHandle(handle, spiHandle);
		EEPROM25AA02_setGpioHandle(handle, gpioHandle);
		EEPROM25AA02_setGpio_CS(handle, gpio_CS);
	}

	return handle;
}

uint16_t EEPROM25AA02_spiTransferByte(EEPROM25AA02_Handle handle, const uint16_t data){
	EEPROM25AA02_Obj *obj = (EEPROM25AA02_Obj *)handle;
	volatile uint16_t ReadByte;

	SPI_write(obj->spiHandle, (data & 0xFF) << 8);

	while(1){
	    if(SPI_getIntFlagStatus(obj->spiHandle)==SPI_IntFlagStatus_Completed){
			ReadByte = SPI_read(obj->spiHandle);
			break;
		}
	}
	return ReadByte;
}

//#########################################################################################

uint8_t EEPROM25AA02_readStatus(EEPROM25AA02_Handle handle) {
	EEPROM25AA02_Obj *obj = (EEPROM25AA02_Obj *)handle;
	uint16_t n = 0;

	uint8_t result = 0;   // result to return

	// take the chip select low to select the device:
	GPIO_setLow(obj->gpioHandle, obj->gpio_CS);

	EEPROM25AA02_spiTransferByte(handle, READ_STATUS_instruction);
	result = EEPROM25AA02_spiTransferByte(handle, 0x00);

	for(n = 0; n < 0xf; n++){            //OK
		asm(" NOP");
	}

	GPIO_setHigh(obj->gpioHandle, obj->gpio_CS);

	return result;
}

// Disable write protection on the whole array: 0x00.
// Enable  write protection on 0xC0 - 0xFF:     0x04.
// Enable  write protection on 0x80 - 0xFF:     0x08.
// Enable  write protection on the whole array: 0x0C.
void EEPROM25AA02_writeStatus(EEPROM25AA02_Handle handle, uint8_t value) {
	EEPROM25AA02_Obj *obj = (EEPROM25AA02_Obj *)handle;
	uint16_t n = 0;

	GPIO_setLow(obj->gpioHandle, obj->gpio_CS);

	EEPROM25AA02_spiTransferByte(handle, WRITE_STATUS_instruction);
	EEPROM25AA02_spiTransferByte(handle, value);  //Send value to record into register

	for(n = 0; n < 0xf; n++){            //OK
		asm(" NOP");
	}

	GPIO_setHigh(obj->gpioHandle, obj->gpio_CS);
}

uint8_t EEPROM25AA02_readRegister(EEPROM25AA02_Handle handle, uint8_t addr) {
	uint32_t n = 0;
	for(n = 0; (n < MAX_WAIT_CYCLES) && (EEPROM25AA02_readStatus(handle) & 0x01); n++){
		asm(" NOP");
	}

	EEPROM25AA02_Obj *obj = (EEPROM25AA02_Obj *)handle;

	uint8_t result = 0;   // result to return
	// take the chip select low to select the device:
	GPIO_setLow(obj->gpioHandle, obj->gpio_CS);
	EEPROM25AA02_spiTransferByte(handle, READ_instruction);
	// send the device the register you want to read:
	EEPROM25AA02_spiTransferByte(handle, addr);
	// send a value of 0 to read the first byte returned:
	result = EEPROM25AA02_spiTransferByte(handle, 0x00);

	for(n = 0; n < 0xf; n++){            //OK
		asm(" NOP");
	}

	// take the chip select high to de-select:
	GPIO_setHigh(obj->gpioHandle, obj->gpio_CS);

	// return the result:
	return result;
}

uint8_t EEPROM25AA02_readRegisterN(EEPROM25AA02_Handle handle, uint8_t addr, uint8_t *buffer, int len) {
	uint32_t n = 0;
	for(n = 0; (n < MAX_WAIT_CYCLES) && (EEPROM25AA02_readStatus(handle) & 0x01); n++){
		asm(" NOP");
	}

	EEPROM25AA02_Obj *obj = (EEPROM25AA02_Obj *)handle;

	if(buffer == 0){
		return 1;
	}
	int index = 0;
	uint8_t result = 0;   // result to return
	// take the chip select low to select the device:
	GPIO_setLow(obj->gpioHandle, obj->gpio_CS);
	EEPROM25AA02_spiTransferByte(handle, READ_instruction);
	// send the device the register you want to read:
	EEPROM25AA02_spiTransferByte(handle, addr);
	while(len > index){
		result = EEPROM25AA02_spiTransferByte(handle, 0x00);
		buffer[index] = result;
		index++;
	}

	for(n = 0; n < 0xf; n++){            //OK
		asm(" NOP");
	}

	GPIO_setHigh(obj->gpioHandle, obj->gpio_CS);

	// return the result:
	return 0;
}


void EEPROM25AA02_getEUI48(EEPROM25AA02_Handle handle, uint8_t *buffer) {
	uint32_t n = 0;
	for(n = 0; (n < MAX_WAIT_CYCLES) && (EEPROM25AA02_readStatus(handle) & 0x01); n++){
		asm(" NOP");
	}

	EEPROM25AA02_Obj *obj = (EEPROM25AA02_Obj *)handle;
	uint8_t i = 0;

	// take the chip select low to select the device:
	GPIO_setLow(obj->gpioHandle, obj->gpio_CS);
	EEPROM25AA02_spiTransferByte(handle, READ_instruction);
	// send the device the register you want to read:
	EEPROM25AA02_spiTransferByte(handle, 0xFA);
	for(i = 0; i < 6; i++){
		buffer[i] = EEPROM25AA02_spiTransferByte(handle, 0x00);
	}
	
	for(n = 0; n < 0xf; n++){            //OK
		asm(" NOP");
	}

	GPIO_setHigh(obj->gpioHandle, obj->gpio_CS);
}

void EEPROM25AA02_getEUI64(EEPROM25AA02_Handle handle, uint8_t *buffer) {
	uint32_t n = 0;
	for(n = 0; (n < MAX_WAIT_CYCLES) && (EEPROM25AA02_readStatus(handle) & 0x01); n++){
		asm(" NOP");
	}

	EEPROM25AA02_Obj *obj = (EEPROM25AA02_Obj *)handle;
	uint8_t i = 0;

	// take the chip select low to select the device:
	GPIO_setLow(obj->gpioHandle, obj->gpio_CS);
	EEPROM25AA02_spiTransferByte(handle, READ_instruction);
	// send the device the register you want to read:
	EEPROM25AA02_spiTransferByte(handle, 0xFA);
	for(i = 0; i < 8; i++){
		if(i == 3){
			buffer[i] = 0xFF;
		}else if(i == 4){
			buffer[i] = 0xFE;
		}else{
			buffer[i] = EEPROM25AA02_spiTransferByte(handle, 0x00);
		}
	}
	
	for(n = 0; n < 0xf; n++){            //OK
		asm(" NOP");
	}

	GPIO_setHigh(obj->gpioHandle, obj->gpio_CS);
}

void EEPROM25AA02_writeRegister(EEPROM25AA02_Handle handle, uint8_t addr, uint8_t value) {
	uint32_t n = 0;
	for(n = 0; (n < MAX_WAIT_CYCLES) && (EEPROM25AA02_readStatus(handle) & 0x01); n++){
		asm(" NOP");
	}

	EEPROM25AA02_Obj *obj = (EEPROM25AA02_Obj *)handle;

	// take the chip select low to select the device:
	GPIO_setLow(obj->gpioHandle, obj->gpio_CS);
	EEPROM25AA02_spiTransferByte(handle, WREN);
	GPIO_setHigh(obj->gpioHandle, obj->gpio_CS);

	GPIO_setLow(obj->gpioHandle, obj->gpio_CS);

	EEPROM25AA02_spiTransferByte(handle, WRITE_instruction);
	EEPROM25AA02_spiTransferByte(handle, addr); //Send register location
	EEPROM25AA02_spiTransferByte(handle, value);  //Send value to record into register

	for(n = 0; n < 0xf; n++){            //OK
		asm(" NOP");
	}

	// take the chip select high to de-select:
	GPIO_setHigh(obj->gpioHandle, obj->gpio_CS);
}

// The following must be true (see datasheet): 0xX0 <= addr; len <= 0x10.
void EEPROM25AA02_writeRegisterN(EEPROM25AA02_Handle handle, uint8_t addr, uint8_t *buffer, int len) {
	uint32_t n = 0;
	for(n = 0; (n < MAX_WAIT_CYCLES) && (EEPROM25AA02_readStatus(handle) & 0x01); n++){
		asm(" NOP");
	}

	int index = 0;
	EEPROM25AA02_Obj *obj = (EEPROM25AA02_Obj *)handle;

	// take the chip select low to select the device:
	GPIO_setLow(obj->gpioHandle, obj->gpio_CS);
	EEPROM25AA02_spiTransferByte(handle, WREN);

	GPIO_setHigh(obj->gpioHandle, obj->gpio_CS);

	GPIO_setLow(obj->gpioHandle, obj->gpio_CS);

	EEPROM25AA02_spiTransferByte(handle, WRITE_instruction);
	EEPROM25AA02_spiTransferByte(handle, addr); //Send register location
	while(len > index){
		EEPROM25AA02_spiTransferByte(handle, buffer[index]);
		index++;
	}

	//EEPROM25AA02_spiTransferByte(handle, WRDI);

	for(n = 0; n < 0xf; n++){            //OK
		asm(" NOP");
	}

	// take the chip select high to de-select:
	GPIO_setHigh(obj->gpioHandle, obj->gpio_CS);
}
