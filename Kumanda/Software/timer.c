#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "system.h"
#include "io.h"
#include "timer.h"

unsigned int double_click_flag;

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

/*
  prescale : 16-bit prescale
  period :   AutoReload'a yazilacak deger.
  repeat :   sadece timer1 icin gecerli.Advanced timer icin.
*/
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
  TIM_ClearITPendingBit(TimTab[tmNo], TIM_IT_Update); // reload tastigi zaman olusur.
  // b) Çevresel yakada kesme kaynaðýný aktive ediyoruz
  TIM_ITConfig(TimTab[tmNo], TIM_IT_Update, ENABLE);
  
  // 2) NVIC (çekirdek) ayarlarý
  // a) Öncelik ayarý
  NVIC_SetPriority(TimIrqTab[tmNo], priority);
  // b) interrupt enable
  NVIC_EnableIRQ(TimIrqTab[tmNo]);
}

////////////////////////////////////////////////////////////////////////////////

void TIM1_UP_IRQHandler(void)
{
}

void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
    double_click_flag = 1;
    
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
}

void TIM3_IRQHandler(void)
{
}

void TIM4_IRQHandler(void)
{
}



