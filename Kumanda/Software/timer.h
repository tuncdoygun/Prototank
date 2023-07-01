#ifndef _TIMER_H
#define _TIMER_H

enum {
  TIMER_1,
  TIMER_2,
  TIMER_3,
  TIMER_4,
};


extern volatile unsigned long g_T1Count, g_T2Count;

void Timer_Init(int tmNo, unsigned prescale, unsigned period,
                unsigned repeat);
void Timer_Start(int tmNo, int bStart);
void Timer_Reset(int tmNo);
void Timer_IntConfig(int tmNo, int priority);

int PWM_Init(int freq, int duty);
void PWM_Duty(int duty);


#endif

