#include "sys_io.h"
#include "main.h"
#include "utility.h"
#include <stdio.h>

#define EC11_A HAL_GPIO_ReadPin(E11_A_GPIO_Port, E11_A_Pin)
#define EC11_B HAL_GPIO_ReadPin(E11_B_GPIO_Port, E11_B_Pin)
#define EC11_KEY HAL_GPIO_ReadPin(E11_KEY_GPIO_Port,E11_KEY_Pin)
#define KEY_CH1 HAL_GPIO_ReadPin(CH1_KEY_GPIO_Port,CH1_KEY_Pin)
#define KEY_CH2 HAL_GPIO_ReadPin(CH2_KEY_GPIO_Port,CH2_KEY_Pin)


IO_State_s g_io_state = {KEY_FREE,KEY_FREE,KEY_FREE, CODER_FREE}; //编码器状态结构体，具体含义见头文件

void EC11_Capture(void)
{
    static unsigned char Aold = 0, Bold = 0;
    static unsigned char RotatingFlag = 0;
    static unsigned int KeyDown_Counter = 0, KeyDown_Counter_Old = 0;
    
    //如果同时为高电平，说明转了
    if (EC11_A && EC11_B)
    {
        RotatingFlag = 1;
    }
    //如果转了之后
    if (RotatingFlag)
    {
        //同时为低电平，判断上次的值，即可得出转的方向
        if (!EC11_A && !EC11_B)
        {
            if (Aold)
            {
                RotatingFlag = 0;
                g_io_state.EC11_Coder = CODER_ADD;
							//printf("add....\n");
            }
            if (Bold)
            {
                RotatingFlag = 0;
                g_io_state.EC11_Coder = CODER_SUB;
							//printf("sub.....\n");
            }
        }
    }

    //保存上次的值
    Aold = EC11_A;
    Bold = EC11_B;

    //记录按键按下时间
    if (!EC11_KEY)
    {
		delay_us(10);
        if(!EC11_KEY) {
            KeyDown_Counter++;
        }
    }
    else
    {
        KeyDown_Counter_Old = KeyDown_Counter;
        KeyDown_Counter = 0;
    }

    //根据按下时间判断长按短按
    if ((KeyDown_Counter_Old < KEYHOLD_TIME_CODER) && (KeyDown_Counter_Old > KEYDOWN_TIME_CODER))
    {
        g_io_state.EC11_Key = KEY_DOWN;
			///printf("cooper key down/\n");
    }
    else if (KeyDown_Counter_Old >= KEYHOLD_TIME_CODER)
    {
        g_io_state.EC11_Key = KEY_HOLD;
        KeyDown_Counter_Old = 0;
			//printf("cooper key hold\n");
    }
}

void Key_Scan(void)
{
    static unsigned char KeyCH1Down_Counter = 0, KeyCh2Down_Counter = 0;
    static unsigned char KeyCH1Down_Counter_Old = 0, KeyCH2Down_Counter_Old = 0;
    //记录按键CH1按下时间
    if (!KEY_CH1)
    {
        delay_us(10);
        if (!KEY_CH1)
        {
            KeyCH1Down_Counter++;
        }
    }
    else
    {
        KeyCH1Down_Counter_Old = KeyCH1Down_Counter;
        KeyCH1Down_Counter = 0;
    }
    //记录按键CH2按下时间
    if (!KEY_CH2)
    {
        delay_us(10);
        if (!KEY_CH2)
        {
            KeyCh2Down_Counter++;
        }
    }
    else
    {
        KeyCH2Down_Counter_Old = KeyCh2Down_Counter;
        KeyCh2Down_Counter = 0;
    } 
 
    //CH1键短按
    if ((KeyCH1Down_Counter_Old < KEYHOLD_TIME) && (KeyCH1Down_Counter_Old > KEYDOWN_TIME))
    {
        g_io_state.CH1_Key = KEY_DOWN;
    }
    //CH1键长按
    else if (KeyCH1Down_Counter_Old >= KEYHOLD_TIME)
    {
        g_io_state.CH1_Key = KEY_HOLD;
    }
    //CH2键短按
    if ((KeyCH2Down_Counter_Old < KEYHOLD_TIME) && (KeyCH2Down_Counter_Old > KEYDOWN_TIME))
    {
        g_io_state.CH2_Key = KEY_DOWN;
    }
    //CH2键长按
    else if (KeyCH2Down_Counter_Old >= KEYHOLD_TIME)
    {
        g_io_state.CH2_Key = KEY_HOLD;
    }
}
