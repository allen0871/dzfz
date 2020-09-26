#ifndef SYS_IO_H
#define SYS_IO_H
#include <stdint.h>


//宏定义
#define KEYDOWN_TIME_CODER 50u
#define KEYHOLD_TIME_CODER 500u

#define KEYDOWN_TIME 2u
#define KEYHOLD_TIME 15u

//按键状态
typedef enum
{
    KEY_FREE = 0,
    KEY_DOWN = 1,
    KEY_HOLD = 2
} KEY_STATUS_e;

//旋转状态
typedef enum
{
    CODER_FREE = 0,
    CODER_ADD = 1,
    CODER_SUB = 2
} CODER_STATUS_e;

//结构体声明
typedef struct
{
    KEY_STATUS_e CH1_Key;
    KEY_STATUS_e CH2_Key;
    KEY_STATUS_e EC11_Key;
    CODER_STATUS_e EC11_Coder;
} IO_State_s;

//全局变量声明
extern IO_State_s g_io_state;

//函数声明
void EC11_Capture(void);
void Key_Scan(void);


#endif
