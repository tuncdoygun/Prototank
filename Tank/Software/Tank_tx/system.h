#ifndef _SYSTEM_H
#define _SYSTEM_H


#define CLOCKS_PER_SEC   1000 // 1 ms karþýlýk gelir.

typedef uint32_t clock_t;

clock_t clock(void);

void Sys_ClockInit(void);
void Sys_IoInit(void);
void Sys_ConsoleInit(void);

void __delay(uint32_t ncy); // number of cycles

// 29.07.2021
#define CLOCK_FREQ      72000000
#define INSTR_FREQ      ((CLOCK_FREQ) * 8 / 9) //instructor frequency
#define US_CYCLES       ((INSTR_FREQ) / 1000000)
#define MS_CYCLES       ((INSTR_FREQ) / 1000)

#define DelayUs(us)     __delay((us) * US_CYCLES) // timer ile gecikme yapýldýðýnda kesme,ayarlamalar vs zaman alabilir kritik zamanlarda.
#define DelayMs(ms)     __delay((ms) * MS_CYCLES)
//


#endif
