#ifndef _IO_H
#define _IO_H

#include "stm32f10x_gpio.h"

#define IO_OLED_RES
#define IO_OLED_CS

enum {
  IO_PORT_A,
  IO_PORT_B,
  IO_PORT_C,
  IO_PORT_D,
  IO_PORT_E,
  IO_PORT_F,
  IO_PORT_G,
};

enum {
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
  IOP_LED,              // Blue pill LED
  IOP_LED_RED,
  
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
  
  // Analog input
  IOP_JOY_VRX,
  IOP_JOY_VRY,
  
  // Joystick buton
  IOP_J_BUTTON,
  
  IOP_BUTTON_UP,
  IOP_BUTTON_DOWN,
  IOP_BUTTON_RIGHT,
  IOP_BUTTON_LEFT,
};

// Variable definitions
#ifdef _IOS_
IO_PIN _ios[] = {
  { IO_PORT_C, 13 },    // LED
  {IO_PORT_B, 0},       // LED-RED 
  
  // SPI
  { IO_PORT_A, 5 },    // SCK
  { IO_PORT_A, 6 },    // MISO
  { IO_PORT_A, 7 },    // MOSI

  // SPI2
  { IO_PORT_B, 13 },    // SCK
  { IO_PORT_B, 14 },    // MISO
  { IO_PORT_B, 15 },    // MOSI
  
  // nRF24
  {IO_PORT_A, 3}, // CSN
  {IO_PORT_A, 2}, // CE 

  // UART-1
  {IO_PORT_A, 10}, // RX
  {IO_PORT_A, 9},  // TX
  
  {IO_PORT_A, 1},    // Analog Input Ch0 - Vrx
  {IO_PORT_A, 0},    // Analog Input Ch1 - Vry
  
  {IO_PORT_A, 4}, // Joystick BUTTON
  
  {IO_PORT_B, 3}, // BTN_UP
  {IO_PORT_B, 4}, // BTN_DOWN
  {IO_PORT_B, 6}, // BTN_RIGHT
  {IO_PORT_B, 5}, // BTN_LEFT
};

GPIO_TypeDef    *_GPIO_Ports[] = {
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
int IO_Read(int idx);

#endif