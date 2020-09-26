#include "LTC1660.h"
#include "main.h"

void setDAC(uint16_t value, uint8_t ch) {
	uint16_t tmp;
  tmp	= value<<2;
	tmp = tmp & 0xFFF;
	tmp = tmp | (ch<<12);
	HAL_GPIO_WritePin(LCD_D0_GPIO_Port,LCD_D0_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DAC_CS_GPIO_Port,DAC_CS_Pin,GPIO_PIN_RESET);
	for(int i=0;i<16;i++) {
		if(tmp & (1<<15)) {
			HAL_GPIO_WritePin(LCD_D1_GPIO_Port,LCD_D1_Pin,GPIO_PIN_SET);
		}
		else {
			HAL_GPIO_WritePin(LCD_D1_GPIO_Port,LCD_D1_Pin,GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(LCD_D0_GPIO_Port,LCD_D0_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(LCD_D0_GPIO_Port,LCD_D0_Pin,GPIO_PIN_RESET);
		tmp=tmp<<1;
	}
	HAL_GPIO_WritePin(DAC_CS_GPIO_Port,DAC_CS_Pin,GPIO_PIN_SET);
}

void setChValue(uint32_t value, uint8_t ch) {
	uint8_t cha = 0;
	uint8_t chb = 0;
	if(ch == 0) {
		cha = 7;
		chb = 8;
	}
	else {
		cha = 5;
		chb = 6;
	}
	uint16_t valuea,valueb;
	valueb = value % 1000;
	valuea = (value-valueb)/1000;
	valueb = 1023*(valueb/999.0);
	valuea = 1023*(valuea/999.0);
	setDAC(valuea,cha);
	setDAC(valueb,chb);
}