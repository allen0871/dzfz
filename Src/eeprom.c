#include "eeprom.h"
#include "ioi2c.h"
#include "utility.h"


uint8_t AA24x_ReadOneByte(uint16_t addr)
{
    uint8_t temp=0;
		uint8_t address = addr>>8;
    I2C_Start(); 
    I2C_Send_Byte(0xA0);//1010000
    I2C_Wait_Ack();
    I2C_Send_Byte(address);
    I2C_Wait_Ack();
	  address = addr&0xFF;
		I2C_Send_Byte(address); 
    I2C_Wait_Ack();
    I2C_Start();
    I2C_Send_Byte(0xA1);//10100001
    I2C_Wait_Ack();
    temp=I2C_Read_Byte(0); 
    I2C_NAck();
    I2C_Stop(); 
    return temp; 
}

void AA24x_WriteOneByte(uint16_t addr,uint8_t dt)
{
		uint8_t address = addr>>8;
    I2C_Start();
    I2C_Send_Byte(0xA0);
    I2C_Wait_Ack();
    I2C_Send_Byte(address); 
    I2C_Wait_Ack();
		address = addr&0xFF;
		I2C_Send_Byte(address); 
    I2C_Wait_Ack();
    I2C_Send_Byte(dt);
    I2C_Wait_Ack();
    I2C_Stop();
    delay_ms(10);
}

//addr需要安32对齐
void AA24x_WriteBytes(uint16_t addr, uint8_t *dt, uint16_t length) {
	uint16_t curAddr = addr;
	uint16_t endAddr = addr+length;
	uint16_t remain = length;
	uint8_t *p = dt;
	for(;curAddr<endAddr;curAddr+=32) {
		uint8_t address = curAddr>>8;
		I2C_Start();
		I2C_Send_Byte(0xA0);
		I2C_Wait_Ack();
		I2C_Send_Byte(address); 
		I2C_Wait_Ack();
		address = curAddr&0xFF;
		I2C_Send_Byte(address); 
		I2C_Wait_Ack();
		uint16_t len = remain>32?32:remain;
		for(int i=0;i<len;i++) {
			I2C_Send_Byte(*(p+i));
			I2C_Wait_Ack();
		}
		I2C_Stop();
		p+=len;
		remain-=len;
    delay_ms(10);
	}
}

