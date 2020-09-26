#include "sys_control.h"
#include "stdint.h"
#include "sys_io.h"

static uint32_t timeTick = 0;

void HAL_IncTick(void)
{
  timeTick++;
  EC11_Capture();
  //32ms扫描一次
  if((timeTick & 0x1F)==0)
  {
      Key_Scan();
  }
}

uint32_t HAL_GetTick(void)
{
  return timeTick;
}

