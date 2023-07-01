#include <stdio.h>
#include <stdint.h>

#include "stm32f10x.h"
#include "system.h"
#include "oled.h"
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

#define N_PORTS (sizeof(_GPIO_Clocks) / sizeof(*_GPIO_Clocks))

void Sys_IoInit(void)
{
  int i;
  
  for(i = 0; i < N_PORTS; ++i) // Bütün GPIO Clock iþaretlerinin açýlmasý.Extreme güç tasarrufu gerekmediðinden hepsini enable etmek birþey deðiþtirmez
    RCC_APB2PeriphClockCmd(_GPIO_Clocks[i], ENABLE);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // AFIO Clock
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // JTAG disable.Biz STLINK kullandýðýmýzdan fazla pin kullanan JTAG'i disable ettik.
}

void Sys_ClockTick(void)
{
  // elapsed time counter.1 ms'de bir deðer 1 artacak.
  ++_tmTick;
}

clock_t clock(void)
{
  return _tmTick;
}

void Sys_ClockInit(void)
{
  SysTick_Config(SystemCoreClock / CLOCKS_PER_SEC); // buraya verilen deðer geriye sarýlýyor.0'a ulaþýnca kesme oluyor.
                                                    // Buradaki SystemCoreClock deðeri CLOCKS_PER_SEC'a bölünmeseydi 1 sn'de bir kesme oluþurdu.
                                                    // Çünkü SystemCoreClock = 72 MHz, 1 saniyede 72 milyon pulse, 1 saniye sonra 0'a düþer deðer.
                                                    // Biz burda CLOCKS_PER_SEC'a böldük böylece 1ms'de bir kesme olacak. 
  
                                                    // Kesme oluþtuðunda stm32f10x_it.c'deki SysTick_Handler fonksiyonu çalýþýr.
}

///////////////////////////////////////////////////
// KONSOL FONKSÝYONLARI
void Sys_ConsoleInit(void)
{
  //OLED_Start(0);
  UART_Init(g_conUART, 9600);
  
#ifndef __IAR_SYSTEMS_ICC__
  setvbuf(stdout, NULL, _IONBF, 0);
#endif
}

void _putch(unsigned char c)
{
  //OLED_putch(c);
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




