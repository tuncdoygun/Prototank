#include <stdio.h>
#include <stdint.h>

#include "stm32f10x.h"

#define _IOS_
#include "io.h"

// ST Periph Lib kullanarak I/O ucunu yapılandırıyoruz
void IO_Init(int idx, int mode)
{
  GPIO_InitTypeDef ioInit;
  int port;
  
  ioInit.GPIO_Mode = (GPIOMode_TypeDef)mode;
  ioInit.GPIO_Speed = GPIO_Speed_50MHz;
  ioInit.GPIO_Pin = (1 << _ios[idx].pin);
  
  port = _ios[idx].port;
  GPIO_Init(_GPIO_Ports[port], &ioInit);
}

void IO_Write(int idx, int val)
{
  int port;
  
  port = _ios[idx].port;
  
  if (val)
    _GPIO_Ports[port]->BSRR = (1 << _ios[idx].pin);
  else
    _GPIO_Ports[port]->BRR = (1 << _ios[idx].pin);    
}

void IO_Toggle(int idx)
{
  int port, pin_val;
  
  port = _ios[idx].port;
  
  pin_val = ((_GPIO_Ports[port]->ODR) >>  (_ios[idx].pin)) & 0x0001; 
  
  IO_Write(idx, (~pin_val & 0x00000001));
}

int IO_Read(int idx)
{
  int port;
  
  port = _ios[idx].port;
  
  return (_GPIO_Ports[port]->IDR & (1 << _ios[idx].pin)) != 0;
}
