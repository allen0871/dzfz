#include "utility.h"
#include "main.h"

void delay_ms(uint32_t value) {
	uint32_t tick = HAL_GetTick();
	while(HAL_GetTick()-tick<value);
}

void delay_us(uint32_t value) {
	__IO uint32_t tick = value;
	while(value--);
}
