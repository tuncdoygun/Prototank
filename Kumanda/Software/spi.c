#include <stdio.h>
#include <stdint.h>

#include "stm32f10x_spi.h"
#include "system.h"
#include "io.h"
#include "spi.h"

struct SPI_CFG {
  int ioSck;
  int ioMosi;
  int ioMiso;
  
  uint32_t clkSPI; // clock SPI
  SPI_TypeDef *pSPI;
};

static struct SPI_CFG _uCfg[] = {
  {IOP_SPI_SCK, IOP_SPI_MOSI, IOP_SPI_MISO, RCC_APB2Periph_SPI1, SPI1},
  {IOP_SPI2_SCK, IOP_SPI2_MOSI, IOP_SPI2_MISO, RCC_APB1Periph_SPI2, SPI2},
};

// Donanýmsal SPI çev. birimini baþlatýr.
void SPI_Start(int idx) 
{
  SPI_InitTypeDef spiInit;
  
  // 1 - SPI Clock aktif edilir.
  if (idx == SPI_1)
    RCC_APB2PeriphClockCmd(_uCfg[idx].clkSPI, ENABLE);
  else
    RCC_APB1PeriphClockCmd(_uCfg[idx].clkSPI, ENABLE);
  
  // 2 - SPI pinleri yapýlandýrýlýr.   
  IO_Init(_uCfg[idx].ioSck, IO_MODE_ALTERNATE); // gpio port yerine spi'a devrediyoruz portu
  IO_Init(_uCfg[idx].ioMosi, IO_MODE_ALTERNATE);
  IO_Init(_uCfg[idx].ioMiso, IO_MODE_INPUT);    
    
  // 3 - SPI Init structure parametreleri yapýlandýrýlýr.
  spiInit.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // 18 MHz .spi çalýþtýrýrken,spi þu hýzda çalýþsýn demek yerine apb kaça bölünsün de hýzý belirlensin olarak hesaplanýr.
  spiInit.SPI_CPOL = SPI_CPOL_Low; // Mode - 0
  spiInit.SPI_CPHA = SPI_CPHA_1Edge;
  spiInit.SPI_DataSize = SPI_DataSize_8b;
  spiInit.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  spiInit.SPI_FirstBit = SPI_FirstBit_MSB;
  spiInit.SPI_Mode = SPI_Mode_Master;
  spiInit.SPI_NSS = SPI_NSS_Soft; // hardware olarak da kullanýlabiliyor.

  SPI_Init(_uCfg[idx].pSPI, &spiInit);
  
  // 4 - SPI çevresel birimi baþlatýlýr. 
  SPI_Cmd(_uCfg[idx].pSPI, ENABLE);    
}

// SPI üzerinden 8 bit veri gönderir ve alýr.
// sadece veri almak için dummy bir deðer gönderilebilir.
uint8_t SPI_Data(int idx, uint8_t val)
{  
  // SPI Transmit buffer boþ mu ?
  while(!SPI_I2S_GetFlagStatus(_uCfg[idx].pSPI, SPI_I2S_FLAG_TXE));
  
  SPI_I2S_SendData(_uCfg[idx].pSPI, val);
  
  while(!SPI_I2S_GetFlagStatus(_uCfg[idx].pSPI, SPI_I2S_FLAG_RXNE)); // normalde almadan çýkmaz gibi dursa da,tx ile rx ayný anda çalýþýyor.
  
  val = SPI_I2S_ReceiveData(_uCfg[idx].pSPI);

  return val;
}
