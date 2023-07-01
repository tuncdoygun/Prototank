#ifndef _SPI_H
#define _SPI_H

enum {
  SPI_1,
  SPI_2,
};

void SPI_Start(int idx);
uint8_t SPI_Data(int idx, uint8_t val);

#endif