/*
 * eeprom_rw.h
 *
 *  Created on: May 30, 2025
 *      Author: araf
 */

#ifndef EEPROM_RW_EEPROM_RW_H_
#define EEPROM_RW_EEPROM_RW_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdio.h>

extern TIM_HandleTypeDef htim1;
extern uint8_t read_io_data;
extern uint16_t read_io_addr;


#define IO_READ 0
#define IO_WRITE 1
#define IO_IDLE 2

#define EEPROM_MSB 22
#define EEPROM_LSB 11

#define OK 1
#define ERROR 0

void _GREEN_LED_ON();
void _GREEN_LED_OFF();
void _READ_LED_ON();
void _READ_LED_OFF();
void _WRITE_LED_ON();
void _WRITE_LED_OFF();
void delay_ns (uint16_t ns);
void set_eeprom_addr(uint16_t addr);
void set_eeprom_io(uint8_t q);
void set_eeprom_io_state(uint8_t state);
uint8_t read_eeprom_io();
void set_eeprom_control_bits(uint8_t state);
uint8_t read_eeprom_addr(uint16_t addr);
uint8_t write_eeprom_addr(uint16_t addr, uint8_t data);
void write_eeprom_addr_nocheck(uint16_t addr, uint8_t data);
void SDP_Disable();
void SDP_Enable();
void check_SDP(void);
void erase_eeprom();
void printContents();
uint8_t eeprom_control_unit_converter(uint16_t control_bits, uint8_t eeprom);
void noen();

#endif /* EEPROM_RW_EEPROM_RW_H_ */
