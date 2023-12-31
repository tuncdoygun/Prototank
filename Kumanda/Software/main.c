#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "system.h"
#include "io.h"
#include "adc.h"
#include "nRF24.h"
#include "button.h"
#include "timer.h"

#define SYS_CLOCK_FREQ  72000000
#define JOY_CH_HIZ        0
#define JOY_CH_YON        1

#define NSAMPLES        50

#define DEBUGss

uint8_t data[3] = {'B', 'T', 0};
int count, j_mod_count, double_click_counter;

enum {
  TIMER_STOP,
  TIMER_START
};

enum {
  NONE,
  GREEN_1,
  GREEN_2,
  YELLOW,
  RED
};

void init(void)
{
  // System Clock init
  Sys_ClockInit();
  
  // I/O portlar� ba�lang��
  Sys_IoInit();
  
  // LED ba�lang��
  IO_Write(IOP_LED, 1);
  IO_Init(IOP_LED, IO_MODE_OUTPUT);
  
  IO_Write(IOP_LED_RED, 1);
  IO_Init(IOP_LED_RED, IO_MODE_OUTPUT);
  
  // Joystick button baslangic
  BTN_InitButtons();
  
  // Console(oled) ba�lang��
  Sys_ConsoleInit();
  
  // init hardware pins
  nrf24_init();
    
  // Channel #2 , payload length: 6
  nrf24_config(2, 6);
  DelayMs(10);  
  
  // buton double click icin timer_2 baslangic
  Timer_Init(TIMER_2, SYS_CLOCK_FREQ / 2000, 2000, 1); // son parametre(repetition counter) timer 2 icin gecerli degil. prescale 36000.CNT'ye 0,5 ms'de bir pulse gidecek.autoreload 2000:yani 0,5 ms'yi 2000 kere sayip kesme olusturacak. 0,5ms*2000 = 1 sn
  Timer_IntConfig(TIMER_2, 3);
  
  // ADC baslangic
  IADC_IoInit(IOP_JOY_VRX);
  IADC_IoInit(IOP_JOY_VRY);
  IADC_Init(1, ADC_CONT_MODE_ON, ADC_SCAN_MODE_OFF);
}

void send_data_with_count(uint8_t *data, int count, int interval_in_us) {
  while(count--){
    nrf24_send(data);
    while(nrf24_isSending()); 
    nrf24_powerDown();
    DelayUs(interval_in_us);
  }
}

void Task_LED(void)
{
  static enum {
    I_LED_OFF,
    S_LED_OFF,
    I_LED_ON,
    S_LED_ON,
  } state = I_LED_OFF;
  
  static clock_t t0;    // Duruma ilk ge�i� saati
  clock_t t1;           // G�ncel saat de�eri
  
  
  t1 = clock();
  
  switch (state) {
  case I_LED_OFF:
      t0 = t1;      
      IO_Write(IOP_LED, 1);     // LED off
      state = S_LED_OFF;
      //break;    
  case S_LED_OFF:
    if (t1 >= t0 + 9 * CLOCKS_PER_SEC / 10) 
      state = I_LED_ON;
    break;
  
  case I_LED_ON:
    t0 = t1;
    IO_Write(IOP_LED, 0);     // LED On
    state = S_LED_ON;
    //break;    
  case S_LED_ON:
    if (t1 >= t0 + CLOCKS_PER_SEC / 10) 
      state = I_LED_OFF;
    break;
  }  
}

void Task_Joystick(void)
{
  int x, y, resultx, resulty;
  static int totalx , totaly, n;

  uint8_t data[6] = {'X', 'Y', 0, 0, 0, 0};
  
  y = IADC_Convert(JOY_CH_YON); // 10-bit 
  x = IADC_Convert(JOY_CH_HIZ); // 10-bit
 
  totalx += x;
  totaly += y;
  
  if(++n >= NSAMPLES) {
    n = 0;
    
    resultx = totalx / NSAMPLES;
    resulty = totaly / NSAMPLES;
    
    data[2] = (resultx >> 8) & 0x0F; // resultlar� gonderilecek veriye cevirme.
    data[3] = resultx & 0xFF; 
    data[4] = (resulty >> 8) & 0x0F;
    data[5] = resulty & 0xFF;
    
#ifdef DEBUG
    printf("x1=%x x2=%x y1=%x y2=%x x=%4d y=%4d\n\r", data[2], data[3], data[4], data[5], resultx, resulty);
#endif
    
    nrf24_send(data);
    while(nrf24_isSending());
    
    nrf24_powerDown();
    DelayMs(10);
    
    totalx = 0;
    totaly = 0;
  }
}
  
void Task_Button(void)
{ 
  if (g_Buttons[BTN_UP] == 1){ // basiliyken surekli gonderiyor.
#ifdef DEBUG
    printf("BTN_UP = HIGH\n");
#endif
    
    data[2] = 'U';
    nrf24_send(data);
    while(nrf24_isSending()); 
  
    data[2] = 0;
  } else if (g_Buttons[BTN_UP] == 2) { // butondan el cekildiginde 10 tane durdurma gonderip birakiyor.
#ifdef DEBUG
    printf("BTN_UP = LOW\n");
#endif
    
    /*
    data[2] = 'A'; 
    send_data_with_count(data, 10, 10);
    */
    
    data[2] = 0;
    g_Buttons[BTN_UP] = 0; // tanktaki kamerayi basili tutarak yonettigimden dolayi basili oldugunda surekli gonderilmeli,elimi biraktigimda daha gondermemeli.
                           // o sebepten buton semaphore'u burada sifirlaniyor.
  }

  if (g_Buttons[BTN_DOWN] == 1){
#ifdef DEBUG
    printf("BTN_DOWN = HIGH\n");
#endif
    
    data[2] = 'D';
    nrf24_send(data);
    while(nrf24_isSending()); 
  
    data[2] = 0;
  } else if (g_Buttons[BTN_DOWN] == 2){
#ifdef DEBUG
    printf("BTN_DOWN = LOW\n");
#endif

    /*
    data[2] = 'B'; 
    send_data_with_count(data, 10, 10);
    */
    
    g_Buttons[BTN_DOWN] = 0; 
    data[2] = 0;
  }
  
  if (g_Buttons[BTN_RIGHT] == 1){
#ifdef DEBUG
    printf("BTN_RIGHT = HIGH\n");
#endif
    
    data[2] = 'R';
    nrf24_send(data);
    while(nrf24_isSending()); 
  
    data[2] = 0;
  } else if (g_Buttons[BTN_RIGHT] == 2){
#ifdef DEBUG    
    printf("BTN_RIGHT = LOW\n");
#endif
    
    /*
    data[2] = 'C'; 
    send_data_with_count(data, 10, 10);
    */
    
    g_Buttons[BTN_RIGHT] = 0; 
    data[2] = 0;
  }
  
  if (g_Buttons[BTN_LEFT] == 1){
#ifdef DEBUG
    printf("BTN_LEFT = HIGH\n");
#endif
    
    data[2] = 'L';
    nrf24_send(data);
    while(nrf24_isSending()); 
  
    data[2] = 0;
  } else if (g_Buttons[BTN_LEFT] == 2){
#ifdef DEBUG
    printf("BTN_LEFT = LOW\n");
#endif
    
    /*
    data[2] = 'E'; 
    send_data_with_count(data, 10, 10);     
    }
    */
    
    g_Buttons[BTN_LEFT] = 0; 
    data[2] = 0;
  }
  
  if (g_Buttons[BTN_JOY] == 1){ // tank kamerasinin joystick butonuna cift tiklandiginda orta konuma gelmesi gerek.
#ifdef DEBUG
    printf("BTN_JOY = HIGH\n");
#endif
    // double buton click'in timer'i butona basildiginde baslamali.butona uzun basma suresini 1sn yapip buton click'inin timer tasma suresini de 1sn yaptigim zaman
    // ledler icin ilk uzun basistan sonra hemen ikinci basista cift tik aktif olmaz cunku cift click'in timer'i zaten dolmus olacak.o cakismayi
    // engellemek icin timer burada baslamali.
    
    Timer_Start(TIMER_2, TIMER_START);
    double_click_counter++;

    if (double_click_counter == 2) {
      data[2] = 'W';
      send_data_with_count(data, 10, 10);
      data[2] = 0;
    }
  
    g_Buttons[BTN_JOY] = 0; 
  }
  
#ifdef BTN_LONG_PRESS
  if (g_ButtonsL[BTN_JOY]){
#ifdef DEBUG
    printf("BTN_JOY\n");
#endif
    
    ++j_mod_count;
    
    if(j_mod_count % 2){
      data[2] = 'J';
      send_data_with_count(data, 10, 10);
      data[2] = 0;
    } else {
      data[2] = 'M';
      send_data_with_count(data, 10, 10);
      data[2] = 0;   
    }
    
    g_ButtonsL[BTN_JOY] = 0; //binary semaphore
  }
#endif

}

void Task_JButton_IRQ_Timeout(void) {
  if (double_click_flag == 1) {
    double_click_counter = 0;
    double_click_flag = 0; 
    Timer_Start(TIMER_2, TIMER_STOP);
  }
}

int main()
{
  uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
  uint8_t rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
  
  // Ba�lang�� yap�land�rmalar�
  init();
  
  printf("********** PROTOTANK KUMANDA **********\n");
  printf("Sistem baslatiliyor...\n");
  
  // Set the device addresses 
  nrf24_tx_address(tx_address);
  nrf24_rx_address(rx_address);
  printf("TX Address : E7E7E7E7E7\n");
  printf("RX Address : D7D7D7D7D7\n");
  
  // G�rev �evrimi (Task Loop)
  // Co-Operative Multitasking (Yard�mla�mal� �oklu g�rev) 
  while (1)
  {
    Task_LED();
    Task_Button();
    Task_JButton_IRQ_Timeout();
    Task_Joystick();
  }
}


