#ifndef EEPROM_H
#define EEPROM_H
#include "stdint.h"

uint8_t AA24x_ReadOneByte(uint16_t addr);
void AA24x_WriteOneByte(uint16_t addr,uint8_t dt);
void AA24x_WriteBytes(uint16_t addr, uint8_t *dt, uint16_t length);

#endif
