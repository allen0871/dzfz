#include "ADS1254.h"
#include "main.h"
#include "stdio.h"

typedef enum {
	ch1v,
	ch1i,
	ch2v,
	ch2i
} adc_status;

int32_t ch1_v;
int32_t ch1_i;
int32_t ch2_v;
int32_t ch2_i;
int32_t adc_value;
volatile uint8_t ch1_v_ready;
volatile uint8_t ch1_i_ready;
volatile uint8_t ch2_v_ready;
volatile uint8_t ch2_i_ready;

static uint16_t ignore;
static uint16_t count;
static int32_t *current;
static volatile uint8_t *curReady;

static adc_status status;

void ADS1254_Init(void) {
	ch1_v = 0;
	ch1_i = 0;
	ch2_v = 0;
	ch2_i = 0;
	ch1_v_ready = 0;
	ch2_v_ready = 0;
	ch1_i_ready = 0;
	ch2_i_ready = 0;
	ignore = 6;
	count = 0;
	current = &ch1_i;
	curReady = &ch1_i_ready;
	HAL_GPIO_WritePin(ADC_SEL1_GPIO_Port,ADC_SEL1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ADC_SEL2_GPIO_Port,ADC_SEL2_Pin,GPIO_PIN_RESET);
	status = ch1i;
}

void ADS1254_Update(uint32_t value) {
	int32_t tmp = value;
	if(ignore)
	{
		ignore--;
		return;
	}
	if(value >= 16777215) {
		tmp = value - 16777215;
	}
	*current += tmp>>2;
	count++;
	if(count == 1024) {
		count = 0;
		ignore = 6;
		adc_value = (*current)>>8;
		*current = 0;
		*curReady = 1;
		switch(status) {
			case ch1i:
			{
				HAL_GPIO_WritePin(ADC_SEL1_GPIO_Port,ADC_SEL1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(ADC_SEL2_GPIO_Port,ADC_SEL2_Pin,GPIO_PIN_RESET);
				current = &ch1_v;
				curReady = &ch1_v_ready;
				status = ch1v;
				break;
			}
			case ch1v:
			{
				HAL_GPIO_WritePin(ADC_SEL1_GPIO_Port,ADC_SEL1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ADC_SEL2_GPIO_Port,ADC_SEL2_Pin,GPIO_PIN_SET);
				current = &ch2_i;
				curReady = &ch2_i_ready;
				status = ch2i;
				break;
			}
			case ch2i:
			{
				/*HAL_GPIO_WritePin(ADC_SEL1_GPIO_Port,ADC_SEL1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(ADC_SEL2_GPIO_Port,ADC_SEL2_Pin,GPIO_PIN_SET);
				current = &ch2_v;
				curReady = &ch2_v_ready;
				status = ch2v;*/
				break;
			}
			case ch2v:
			{
				HAL_GPIO_WritePin(ADC_SEL1_GPIO_Port,ADC_SEL1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ADC_SEL2_GPIO_Port,ADC_SEL2_Pin,GPIO_PIN_RESET);
				current = &ch1_i;
				curReady = &ch1_i_ready;
				status = ch1i;
				break;
			}
			default:
				break;
		}
	}
}
