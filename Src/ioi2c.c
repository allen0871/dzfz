#include "ioi2c.h"
#include "main.h"
#include "utility.h"

#define I2C_SCL GPIO_PIN_6	  //PB6
#define I2C_SDA GPIO_PIN_7	  //PB7
 
#define I2C_SCL_H HAL_GPIO_WritePin(GPIOB,I2C_SCL,GPIO_PIN_SET)
#define I2C_SCL_L HAL_GPIO_WritePin(GPIOB,I2C_SCL,GPIO_PIN_RESET)
 
#define I2C_SDA_H HAL_GPIO_WritePin(GPIOB,I2C_SDA,GPIO_PIN_SET)
#define I2C_SDA_L HAL_GPIO_WritePin(GPIOB,I2C_SDA,GPIO_PIN_RESET)
//0.50673

void IOI2C_INIT()
{
    GPIO_InitTypeDef GPIO_InitStructure;
 
    GPIO_InitStructure.Pin= I2C_SCL|I2C_SDA;
    GPIO_InitStructure.Mode=GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull=GPIO_NOPULL;
    GPIO_InitStructure.Speed= GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB,&GPIO_InitStructure);
 
    I2C_SCL_H;
    I2C_SDA_H;
}

void I2C_SDA_OUT()
{
    GPIO_InitTypeDef GPIO_InitStructure; 
 
    GPIO_InitStructure.Pin =I2C_SDA;
    GPIO_InitStructure.Mode =GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull =GPIO_PULLUP;
    GPIO_InitStructure.Speed =GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB,&GPIO_InitStructure);
}

void I2C_SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStructure; 
 
    GPIO_InitStructure.Pin =I2C_SDA;
    GPIO_InitStructure.Mode =GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull =GPIO_NOPULL;
    GPIO_InitStructure.Speed =GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB,&GPIO_InitStructure);
}

void I2C_Start(void)
{
		I2C_SDA_OUT();
 
    I2C_SDA_H;
    I2C_SCL_H;
    delay_us(5);
    I2C_SDA_L;
    delay_us(6);
    I2C_SCL_L;
}

void I2C_Stop(void)
{
 I2C_SDA_OUT();
 
 
 I2C_SCL_L;
 I2C_SDA_L;
 I2C_SCL_H;
 delay_us(6);
 I2C_SDA_H;
 delay_us(6);
}

void I2C_Ack(void)
{
	I2C_SCL_L;
	I2C_SDA_OUT();
	I2C_SDA_L;
	delay_us(2);
	I2C_SCL_H;
	delay_us(5);
	I2C_SCL_L;
}

void I2C_NAck(void)
{
	I2C_SCL_L;
	I2C_SDA_OUT();
	I2C_SDA_H;
	delay_us(2);
	I2C_SCL_H;
	delay_us(5);
	I2C_SCL_L;
}

uint8_t I2C_Wait_Ack(void)
{
    uint8_t tempTime=0;
 
    I2C_SDA_IN();
    I2C_SDA_H;
    delay_us(1);
    I2C_SCL_H;
    delay_us(1);
 
 
    while(HAL_GPIO_ReadPin(GPIOB,I2C_SDA))
    {
        tempTime++;
        if(tempTime>250)
        {
            I2C_Stop();
            return 1;
        } 
    }
    I2C_SCL_L;
    return 0;
}

void I2C_Send_Byte(uint8_t txd)
{
    uint8_t i=0;
 
    I2C_SDA_OUT();
    I2C_SCL_L;
 
    for(i=0;i<8;i++)
    {
        if((txd&0x80)>0) //0x80  1000 0000
            I2C_SDA_H;
        else
            I2C_SDA_L;
 
        txd<<=1;
        I2C_SCL_H;
        delay_us(2); 
        I2C_SCL_L;
        delay_us(2);
    }
}

uint8_t I2C_Read_Byte(uint8_t ack)
{
	uint8_t i=0,receive=0;
 
	I2C_SDA_IN();
	for(i=0;i<8;i++)
	{
			I2C_SCL_L;
      delay_us(2);
      I2C_SCL_H;
        receive<<=1;
        if(HAL_GPIO_ReadPin(GPIOB,I2C_SDA))
					receive++;
        delay_us(1); 
	}
	if(ack==0)
    I2C_NAck();
  else
      I2C_Ack();
  return receive;
}

