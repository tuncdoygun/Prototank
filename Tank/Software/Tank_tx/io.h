#ifndef _IO_H
#define _IO_H

#include "stm32f10x_gpio.h"

enum {
  IO_PORT_A,
  IO_PORT_B,
  IO_PORT_C,
  IO_PORT_D,
  IO_PORT_E,
  IO_PORT_F,
  IO_PORT_G,
};

enum { // stm32f10x_gpio.h GPIOMode_TypeDef struct
 IO_MODE_INPUT = GPIO_Mode_IN_FLOATING,
 IO_MODE_OUTPUT = GPIO_Mode_Out_PP,
 IO_MODE_ALTERNATE = GPIO_Mode_AF_PP,
 IO_MODE_ANALOG = GPIO_Mode_AIN,
 
 IO_MODE_INPUT_PD = GPIO_Mode_IPD,
 IO_MODE_INPUT_PU = GPIO_Mode_IPU,
 IO_MODE_OUTPUT_OD = GPIO_Mode_Out_OD,
 IO_MODE_ALTERNATE_OD = GPIO_Mode_AF_OD,
};

typedef struct {
  int port;
  int pin;
} IO_PIN;

enum {
  IOP_LED,      // Bluepill LED (C13)
  IOP_LED_KIRMIZI,      // LED-KIRMIZI
  IOP_LED_BEYAZ,        // LED-BEYAZ
  
  // SPI1
  IOP_SPI_SCK,
  IOP_SPI_MISO,
  IOP_SPI_MOSI,  
  
  // SPI2
  IOP_SPI2_SCK,
  IOP_SPI2_MISO,
  IOP_SPI2_MOSI,
  
  // nRF24
  IOP_nRF_CSN,
  IOP_nRF_CE,
  
  // UART1-console
  IOP_U1RX,
  IOP_U1TX,
  
  // TIM2_CH4 pwm cikisi
  IOP_PWM_SERVO_2,
  
  // TIM2_CH3 pwm cikisi
  IOP_PWM_SERVO_1,
  
  // TIM4_CH2 pwm cikisi
  IOP_PWM_DC_2,
  
  // TIM4_CH1 pwm cikisi
  IOP_PWM_DC_1,  

  IOP_IN4,
  IOP_IN3,
  IOP_IN2,
  IOP_IN1
};

#ifdef _IOS_ // birkere tanýmlanmasý gereken

IO_PIN _ios[] = {
  {IO_PORT_C, 13},
  { IO_PORT_B, 11 },    // LED-BEYAZ
  { IO_PORT_B, 10 },    // LED-KIRMIZI
  
  // SPI1
  {IO_PORT_A, 5}, // SCK
  {IO_PORT_A, 6}, // MISO
  {IO_PORT_A, 7}, // MOSI  
  
  // SPI2
  { IO_PORT_B, 13 },    // SCK
  { IO_PORT_B, 14 },    // MISO
  { IO_PORT_B, 15 },    // MOSI  

  // nRF24
  {IO_PORT_A, 11}, // CSN
  {IO_PORT_A, 8}, // CE  
  
  // UART-1
  {IO_PORT_A, 10}, // RX
  {IO_PORT_A, 9},  // TX
  
  // TIM2_CH4 PWM-S-2 cikisi
  {IO_PORT_A, 3}, // servo 2

  // TIM2_CH3 PWM-S-1 cikisi
  {IO_PORT_A, 2}, // servo 1
  
  // TIM4_CH2 PWM-DC-2(ENA) cikisi
  {IO_PORT_B, 7}, // dc motor 2

  // TIM4_CH1 PWM-DC-1(ENB) cikisi
  {IO_PORT_B, 6}, // dc motor 1 

  {IO_PORT_B, 8}, // in4
  {IO_PORT_B, 5}, // in3
  {IO_PORT_B, 4}, // in2
  {IO_PORT_B, 3}, // in1
};

GPIO_TypeDef *_GPIO_Ports[] = {
  GPIOA,
  GPIOB,
  GPIOC,
  GPIOD,
  GPIOE,
  GPIOF,
  GPIOG,
};

#endif

void IO_Init(int idx, int mode);
void IO_Write(int idx, int val);
void IO_Toggle(int idx);
int IO_Read(int idx); // burda dönüþ deðerinin int veya bool kullanýlmasýnýn farký yok.32 bitte olsa 1 bitte olsa 1 cycle'da iþliyor iþlemci.

#endif