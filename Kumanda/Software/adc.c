#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "system.h"
#include "io.h"

volatile int g_bEOC;    // End of conversion flag

void IADC_IoInit(int idx)
{
  IO_Init(idx, IO_MODE_ANALOG);
}

// rank : kanal kacinci sirada olsun.1 - 16. Listeye ekleme yapmaya yarar.
void IADC_Channel(int ch, int rank)
{
  ADC_RegularChannelConfig(ADC1, ch, rank, ADC_SampleTime_239Cycles5);
}

// nConv: Toplam kanal sayýsý (listedeki).suanlik 1, daha fazlasi icin DMA kullanmak gerekli.
// cmode: TRUE/FALSE sürekli mod
// smode: TRUE/FALSE tarama modu
void IADC_Init(int nConv, int cmode, int smode)
{
  // 1) ADC clock setup
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_ADCCLKConfig(RCC_PCLK2_Div4); // 18 MHz clock uretilmis olacak. 
  
  ADC_InitTypeDef adcInit;
  
  //ADC_StructInit(&adcInit);// cok structure elemani oldugundan, default degerlerle baslatmak icin.
  
  // 2) Çevresel Structure deðerleri
  adcInit.ADC_ContinuousConvMode = cmode ? ENABLE : DISABLE;
  adcInit.ADC_DataAlign = ADC_DataAlign_Right;
  adcInit.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  adcInit.ADC_Mode = ADC_Mode_Independent;
  adcInit.ADC_NbrOfChannel = nConv;
  adcInit.ADC_ScanConvMode = smode ? ENABLE : DISABLE;
  
  ADC_Init(ADC1, &adcInit);
  
  // 3) Çevresel birimi baþlatýyoruz
  ADC_Cmd(ADC1, ENABLE);
  
  // 4) Vref ve sýcaklýk sensörü aktif olacak
  // AIN16: Sýcaklýk sensörü (jonksiyon)
  // AIN17: Internal voltaj regülatör gerilimi
  ADC_TempSensorVrefintCmd(ENABLE);
  
  // 5) ADC kalibrasyon
  // a) Kalibrasyon register reset
  ADC_ResetCalibration(ADC1);
  while (ADC_GetResetCalibrationStatus(ADC1)) ;
  // b) Kalibrasyonu baþlat
  ADC_StartCalibration(ADC1);
  while (ADC_GetCalibrationStatus(ADC1)) ;
}

// Software trigger
// ADC iþlemini yazýlýmsal olarak baþlatýr
void IADC_Start(void)
{
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

// ADC tamamlandýðýnda sonuç register'ý deðerine
// geri döner
int IADC_Result(void)
{
  return ADC_GetConversionValue(ADC1);
}

// 0 - 4095 arasi sayiya geri doner.
// Ya surekli cevrim yapmak icin IADC_Start cagirilacak ya da continious mod acik olacak.(IADC_Init fonksiyonunda).
// Surekli IADC_Start cagrilmazsa 1 kere cevrim yapar birakir.
int IADC_Convert(int ch)
{
  // 1) Kanalý listeye ekliyoruz
  IADC_Channel(ch, 1);
  
  // 2) AD dönüþümü baþlatýyoruz
  IADC_Start();
  
  // 3) Dönüþüm bitene dek bekliyoruz
  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) ;
  
  // 4) Sonucu okuyup geri dönüyoruz
  return ADC_GetConversionValue(ADC1);
}

// ADC EOC (End of Conversion) kesmesi yapýlandýrmasý
void IADC_IntConfig(void)
{
  // Çevresel bölge ayarlar
  ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
  ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
  
  // Çekirdek NVIC ayarlarý
  NVIC_SetPriority(ADC1_2_IRQn, 2);
  NVIC_EnableIRQ(ADC1_2_IRQn);
}

////////////////////////////////////////////////////

void ADC1_2_IRQHandler(void)
{
  if (ADC_GetITStatus(ADC1, ADC_IT_EOC) == SET) {
    g_bEOC = 1;
    
    ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
  }
}
