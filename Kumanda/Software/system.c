#include <stdio.h>
#include <stdint.h>

#include "stm32f10x.h"
#include "system.h"
#include "button.h"
#include "uart.h"

#define _STDIN  0
#define _STDOUT 1
#define _STDERR 2

static volatile clock_t _tmTick;

static const int _GPIO_Clocks[] = {
  RCC_APB2Periph_GPIOA,
  RCC_APB2Periph_GPIOB,
  RCC_APB2Periph_GPIOC,
  RCC_APB2Periph_GPIOD,
  RCC_APB2Periph_GPIOE,
  RCC_APB2Periph_GPIOF,
  RCC_APB2Periph_GPIOG,
};

#define N_PORTS (sizeof(_GPIO_Clocks) / sizeof(int))

void Sys_IoInit(void)
{
  int i;
  
  // GPIOx Clock baþlangýç
  for (i = 0; i < N_PORTS; ++i)
    RCC_APB2PeriphClockCmd(_GPIO_Clocks[i], ENABLE);
  
  // AFIO clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  
  // JTAG disable
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
}

void Sys_ClockTick(void)
{
  // elapsed time counter
  ++_tmTick;
  
  BTN_ScanButtons();
}

clock_t clock(void)
{
  return _tmTick;
}

void Sys_ClockInit(void)
{
  SysTick_Config(SystemCoreClock / CLOCKS_PER_SEC);
}

///////////////////////////////////////////////////
// KONSOL FONKSÝYONLARI
void Sys_ConsoleInit(void)
{
  UART_Init(g_conUART, 9600);

#ifndef __IAR_SYSTEMS_ICC__
  setvbuf(stdout, NULL, _IONBF, 0);
#endif
}

void _putch(unsigned char c)
{
  UART_putch(c);
}

#ifdef __IAR_SYSTEMS_ICC__
size_t __write(int handle, const unsigned char *buffer, size_t size)
#else
size_t _write(int handle, const unsigned char *buffer, size_t size)
#endif
{
  size_t nChars = 0;
  
  if (buffer == NULL)
    return 0;
  
  if (handle != _STDOUT && handle != _STDERR)
    return 0;
  
  while (size--) {
    _putch(*buffer++);
    ++nChars;
  }
  
  return nChars;
}



