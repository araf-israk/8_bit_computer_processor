/*
 * eeprom_rw.c
 *
 *  Created on: May 30, 2025
 *      Author: araf
 */
#include "eeprom_rw.h"

void _GREEN_LED_ON(){
	HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, SET);
}

void _GREEN_LED_OFF(){
	HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, RESET);
}

void _READ_LED_ON(){
	HAL_GPIO_WritePin(READ_LED_GPIO_Port, READ_LED_Pin, SET);
}

void _READ_LED_OFF(){
	HAL_GPIO_WritePin(READ_LED_GPIO_Port, READ_LED_Pin, RESET);
}

void _WRITE_LED_ON(){
	HAL_GPIO_WritePin(WRITE_LED_GPIO_Port, WRITE_LED_Pin, SET);
}

void _WRITE_LED_OFF(){
	HAL_GPIO_WritePin(WRITE_LED_GPIO_Port, WRITE_LED_Pin, RESET);
}

void delay_ns (uint16_t ns)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < ns);  // wait for the counter to reach the us input in the parameter
}

void set_eeprom_addr(uint16_t addr){
	GPIOA->ODR &= ~(0x1FFF);
	GPIOA->ODR |= (0x1FFF) & addr;
}

void set_eeprom_io(uint8_t q){
	GPIOB->ODR &= ~(0xF << 4);
	GPIOB->ODR &= ~(0xF << 12);
	GPIOB->ODR |= (((0x0F) & q) << 4);
	GPIOB->ODR |= ((q >> 4) << 12);
}

void set_eeprom_io_state(uint8_t state){
	if(state == IO_READ){
		GPIOB->MODER &= ~(0xFF << 8);
		GPIOB->MODER &= ~(0xFF << 24);
	}
	if(state == IO_WRITE){
		GPIOB->MODER &= ~(0xFF << 8);
		GPIOB->MODER &= ~(0xFF << 24);
		GPIOB->MODER |= (0b01010101 << 8);
		GPIOB->MODER |= (0b01010101 << 24);
	}
}

uint8_t read_eeprom_io(){
	uint8_t data = 0;
	data = ((GPIOB->IDR & 0xF000) >> 8);
	data |= ((GPIOB->IDR & 0x00F0) >> 4);
	return data;
}

void set_eeprom_control_bits(uint8_t state){
	if(state == IO_READ){
		  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
		  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, RESET);
		  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, RESET);
	}
	if(state == IO_WRITE){
		  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
		  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);
		  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, RESET);
		  delay_ns(3);
		  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, RESET);
		  delay_ns(1);
		  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
		  HAL_Delay(8);

	}
	if(state == IO_IDLE){
		  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
		  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);
		  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, SET);
	}
}


uint8_t read_eeprom_addr(uint16_t addr){
	_READ_LED_ON();
	uint8_t _temp;
	set_eeprom_io_state(IO_READ);
	set_eeprom_addr(addr);
	set_eeprom_control_bits(IO_READ);
	_temp = read_eeprom_io();
	set_eeprom_control_bits(IO_IDLE);
	_READ_LED_OFF();
	return _temp;

}

uint8_t write_eeprom_addr(uint16_t addr, uint8_t data){
	_WRITE_LED_ON();
	uint8_t _temp;
	set_eeprom_io_state(IO_WRITE);
	set_eeprom_addr(addr);
	set_eeprom_control_bits(IO_IDLE);
	set_eeprom_io(data);
	set_eeprom_control_bits(IO_WRITE);
	set_eeprom_control_bits(IO_IDLE);
	_temp = read_eeprom_addr(addr);
	_WRITE_LED_OFF();
	if(_temp == data){
		return OK;
	}else{
		return ERROR;
	}
}

void write_eeprom_addr_nocheck(uint16_t addr, uint8_t data){
	set_eeprom_io_state(IO_WRITE);
	set_eeprom_addr(addr);
	set_eeprom_control_bits(IO_IDLE);
	set_eeprom_io(data);
	HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
	HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);
	HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, RESET);
	delay_ns(3);
	HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, RESET);
	delay_ns(1);
	HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
	HAL_Delay(3);
	set_eeprom_control_bits(IO_IDLE);
}

void SDP_Disable(){
	write_eeprom_addr_nocheck(0x1555, 0xAA);
	write_eeprom_addr_nocheck(0x0AAA, 0x55);
	write_eeprom_addr_nocheck(0x1555, 0x80);
	write_eeprom_addr_nocheck(0x1555, 0xAA);
	write_eeprom_addr_nocheck(0x0AAA, 0x55);
	write_eeprom_addr_nocheck(0x1555, 0x20);
	write_eeprom_addr_nocheck(0x0000, 0x00);
	write_eeprom_addr_nocheck(0x0000, 0x00);
}

void SDP_Enable(){
	write_eeprom_addr_nocheck(0x1555, 0xAA);
	write_eeprom_addr_nocheck(0x0AAA, 0x55);
	write_eeprom_addr_nocheck(0x1555, 0xA0);
	write_eeprom_addr_nocheck(0x0000, 0x00);
	write_eeprom_addr_nocheck(0x0000, 0x00);
}

void check_SDP(void){
	printf("A 1555 %02x \n", read_eeprom_addr(0x1555));
}

void erase_eeprom(){
	printf("\nErasing the EEPROM...");
	int _temp = 0;
	for(int i = 0; i <= 0x1FFF; i++){
		_temp += write_eeprom_addr(i, 0x00);
//		if(i%1000 == 0){
//			printf(".");
//		}
	}
	printf("Done\n");
	printf("\nDeleted %d data\n\n", _temp);

}

void printContents(){
	  for(int base = 0; base < 256; base += 16){
		  uint8_t data[16];
		  for(int offset = 0; offset < 16; offset += 1){
			  data[offset] = read_eeprom_addr(base + offset);
		  }

		  printf("  %04x: %02x %02x %02x %02x %02x %02x %02x %02x   %02x %02x %02x %02x %02x %02x %02x %02x \n",
				  	base, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
						  data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);
	  }
}

uint8_t eeprom_control_unit_converter(uint16_t control_bits, uint8_t eeprom){
	if(eeprom == EEPROM_MSB){
		return (control_bits >> 8);
	}
	if(eeprom == EEPROM_LSB){
		return (0x00FF & control_bits);
	}
	return 0;
}

void noen(){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
			  HAL_Delay(100);
	//		  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
	//		  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, RESET);
	//		  delay_ns(5);
	//		  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);

			  read_io_addr = 0x1555;
			  set_eeprom_io_state(IO_WRITE);
			  set_eeprom_addr(read_io_addr);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
			  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);
			  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, SET);
			  set_eeprom_io(0xAA);
			  delay_ns(3);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
			  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);
			  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, RESET);
			  delay_ns(3);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, RESET);
			  delay_ns(1);
			  //HAL_Delay(1);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);

			  delay_ns(3);

			  read_io_addr = 0x0AAA;
			  set_eeprom_io_state(IO_WRITE);
			  set_eeprom_addr(read_io_addr);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
			  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);
			  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, SET);
			  set_eeprom_io(0x55);
			  delay_ns(3);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
			  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);
			  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, RESET);
			  delay_ns(3);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, RESET);
			  delay_ns(1);
			  //HAL_Delay(1);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);

			  delay_ns(3);

			  read_io_addr = 0x1555;
			  set_eeprom_io_state(IO_WRITE);
			  set_eeprom_addr(read_io_addr);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
			  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);
			  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, SET);
			  set_eeprom_io(0x80);
			  delay_ns(3);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
			  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);
			  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, RESET);
			  delay_ns(3);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, RESET);
			  delay_ns(1);
			  //HAL_Delay(1);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);

			  delay_ns(3);

			  read_io_addr = 0x1555;
			  set_eeprom_io_state(IO_WRITE);
			  set_eeprom_addr(read_io_addr);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
			  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);
			  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, SET);
			  set_eeprom_io(0xAA);
			  delay_ns(3);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
			  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);
			  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, RESET);
			  delay_ns(3);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, RESET);
			  delay_ns(1);
			  //HAL_Delay(1);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);

			  delay_ns(3);

			  read_io_addr = 0x0AAA;
			  set_eeprom_io_state(IO_WRITE);
			  set_eeprom_addr(read_io_addr);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
			  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);
			  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, SET);
			  set_eeprom_io(0x55);
			  delay_ns(3);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
			  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);
			  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, RESET);
			  delay_ns(3);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, RESET);
			  delay_ns(1);
			  //HAL_Delay(1);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);


			  delay_ns(3);

			  read_io_addr = 0x1555;
			  set_eeprom_io_state(IO_WRITE);
			  set_eeprom_addr(read_io_addr);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
			  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);
			  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, SET);
			  set_eeprom_io(0x20);
			  delay_ns(3);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
			  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);
			  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, RESET);
			  delay_ns(3);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, RESET);
			  delay_ns(1);
			  //HAL_Delay(1);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);


			  delay_ns(3);

			  read_io_addr = 0x0000;
			  set_eeprom_io_state(IO_WRITE);
			  set_eeprom_addr(read_io_addr);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
			  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);
			  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, SET);
			  set_eeprom_io(0x00);
			  delay_ns(3);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
			  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);
			  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, RESET);
			  delay_ns(3);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, RESET);
			  delay_ns(1);
			  //HAL_Delay(1);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);

			  delay_ns(3);

			  read_io_addr = 0x0000;
			  set_eeprom_io_state(IO_WRITE);
			  set_eeprom_addr(read_io_addr);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
			  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);
			  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, SET);
			  set_eeprom_io(0x00);
			  delay_ns(3);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
			  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);
			  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, RESET);
			  delay_ns(3);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, RESET);
			  delay_ns(1);
			  //HAL_Delay(1);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);

			  delay_ns(3);





	//		  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
			  HAL_GPIO_WritePin(WE_GPIO_Port, WE_Pin, SET);
			  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, SET);
			  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, SET);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);

}
