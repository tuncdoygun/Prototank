#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "system.h"
#include "io.h"
#include "timer.h"

TIM_TypeDef     *TimTab[] = {
  TIM1, TIM2, TIM3, TIM4
};

uint32_t TimRccTab[] = {
  RCC_APB2Periph_TIM1,
  RCC_APB1Periph_TIM2,
  RCC_APB1Periph_TIM3,
  RCC_APB1Periph_TIM4,
};

IRQn_Type TimIrqTab[] = {
  TIM1_UP_IRQn,
  TIM2_IRQn,
  TIM3_IRQn,
  TIM4_IRQn,
};

void Timer_Init(int tmNo, unsigned prescale, unsigned period,
                unsigned repeat)
{
  TIM_TimeBaseInitTypeDef tmInit;
  
  // 1) Çevresel clock saðlýyoruz
  if (tmNo == TIMER_1)
    RCC_APB2PeriphClockCmd(TimRccTab[tmNo], ENABLE);
  else
    RCC_APB1PeriphClockCmd(TimRccTab[tmNo], ENABLE);
    
  // 2) Timer parametrelerini ayarlýyoruz
  tmInit.TIM_ClockDivision = TIM_CKD_DIV1;
  tmInit.TIM_CounterMode = TIM_CounterMode_Up;
  tmInit.TIM_Period = period - 1;
  tmInit.TIM_Prescaler = prescale - 1;
  tmInit.TIM_RepetitionCounter = repeat - 1;
  
  TIM_TimeBaseInit(TimTab[tmNo], &tmInit);
  
  Timer_Reset(tmNo);
}

void Timer_Start(int tmNo, int bStart)
{
  TIM_Cmd(TimTab[tmNo], bStart ? ENABLE : DISABLE);
}

void Timer_Reset(int tmNo)
{
  TIM_SetCounter(TimTab[tmNo], 0);
}

void Timer_IntConfig(int tmNo, int priority)
{
  // 1) Çevresel birim ayarlarý
  // a) False interrupt önlemi
  TIM_ClearITPendingBit(TimTab[tmNo], TIM_IT_Update);
  // b) Çevresel yakada kesme kaynaðýný aktive ediyoruz
  TIM_ITConfig(TimTab[tmNo], TIM_IT_Update, ENABLE);
  
  // 2) NVIC (çekirdek) ayarlarý
  // a) Öncelik ayarý
  NVIC_SetPriority(TimIrqTab[tmNo], priority);
  // b) interrupt enable
  NVIC_EnableIRQ(TimIrqTab[tmNo]);
}

////////////////////////////////////////////////////////////////////////////////

// TIM2 CH3 kullanýlacak
// freq: PWM frekansý
// duty: PWM duty cycle % olarak
int PWM_Init(int freq, int duty)
{
  //SystemCoreClock
  uint32_t period, prescale;
  
  // 1) Çýkýþ kanalýnýn I/O ayarlarý
  IO_Init(IOP_TIM2_CH3, IO_MODE_ALTERNATE);
  
  // 2) Timer ayarlarý
  // PWM frekansý belirlenecek
  // ftmr = fsysclk / CKD_DIVx
  // ftov = fpwm = ftmr / (period * prescale)
  // Tpwm = (period * prescale) / ftmr
  period =  SystemCoreClock / freq;
  if (period >= 65536) {
    // düzeltilmeli !!!
    prescale = period / 32768;
    period = 32768;
  }
  else
    prescale = 1;
    
  Timer_Init(TIMER_2, prescale, period, 1);
  
  // 3) PWM ayarlarý
  // Output compare
  TIM_OCInitTypeDef ocInit;
  
  ocInit.TIM_OCIdleState = TIM_OCIdleState_Reset;
  ocInit.TIM_OCMode = TIM_OCMode_PWM1;
  ocInit.TIM_OCPolarity = TIM_OCPolarity_High;
  ocInit.TIM_OutputState = TIM_OutputState_Enable;
  ocInit.TIM_Pulse = period * duty / 100;
  
  TIM_OC3Init(TIM2, &ocInit);
  
  // 4) Timer'ý çalýþtýrýyoruz
  Timer_Start(TIMER_2, 1);
  
  return period;
}

void PWM_Duty(int duty)
{
  TIM_SetCompare3(TIM2, duty);
}

////////////////////////////////////////////////////////////////////////////////

volatile unsigned long g_T1Count, g_T2Count;

void TIM1_UP_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) {
    ++g_T1Count;
    
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
  }
}

void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
    ++g_T2Count;
    
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
}

void TIM3_IRQHandler(void)
{
}

void TIM4_IRQHandler(void)
{
}



