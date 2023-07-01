#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "system.h"
#include "io.h"
#include "spi.h"

#include "oled.h"

#ifdef FNT_SMALL
#include "font_5x8.h"
#endif
#ifdef FNT_LARGE
#include "font_7x16.h"
#endif
#ifdef FNT_BIG
#include "font_15x32.h"
#endif

#define T_COLS  (NSEG / ((_szWidth) + 1)) // kac tane text sutun
#define T_ROWS  (NPGS / (_szHeight)) // kav tane text satir

static int      _szWidth;       // aktif font pixel geniþliði (bit)
static int      _szHeight;      // aktif font yüksekliði (page - byte)
static const unsigned char      *_pFont; // aktif font tablosu

static int      _row, _col;     // text satýr, sütun
static int      _font;

static int _page, _segment; // aktif page ve segmentin hangisi olacaðýný gösterecek.

uint8_t  _DspRam[NPGS * NSEG]; // ssd1306'daki ramin bir kopyasýný biz tutucaz

// SSD1306 OLED display'e komut gönderir
void OLED_Command(uint8_t cmd)
{
  IO_Write(IOP_OLED_DC, 0);  // komut
  
#ifdef IO_OLED_CS
  IO_Write(IOP_OLED_CS, 0); // CS = 0, aktif
#endif
  
  SPI_Data(cmd);
  
#ifdef IO_OLED_CS
  IO_Write(IOP_OLED_CS, 1); // CS = 1, pasif
#endif  
}

// OLED display'e pixel data gönderir.
// Her data byte 8 pixele karþýlýk gelir.
void OLED_Data(uint8_t data)
{
  IO_Write(IOP_OLED_DC, 1);  // data
  
#ifdef IO_OLED_CS
  IO_Write(IOP_OLED_CS, 0); // CS = 0, aktif
#endif
  
  SPI_Data(data); // display memory'nin aktif olmasý gerkiyor verinin yazýlacaðý yerin.Yani aktif page aktif segment
  
  _DspRam[_page * NSEG + _segment] = data;
  
  if(++_segment >= NSEG)
    _segment = 0;
  
#ifdef IO_OLED_CS
  IO_Write(IOP_OLED_CS, 1); // CS = 1, pasif
#endif 
}

void OLED_SetPage(uint8_t page)
{
  page &= 0x07; // 7'den büyükse büyük olan kýsýmlarý kýrpýlýyor.
  
  _page = page;
  OLED_Command(0xB0 | page); // page set etme komutu
}

void OLED_SetSegment(uint8_t segment)
{
  segment &= 0x7F; // 128'den büyük olmasýný engellemek.
  
  _segment = segment;
  
  OLED_Command(segment & 0x0F); // önce düþük anlamlý bit gönderilir,spi bu þekilde çalýþýyor
  OLED_Command(0x10 | (segment >> 4)); // segment set etme komutu
}

void OLED_FillPage(uint8_t page, uint8_t ch)
{
  int i;
  
  OLED_SetPage(page);
  OLED_SetSegment(0);
  
  for (i = 0; i < NSEG; ++i)
    OLED_Data(ch);
}

void OLED_FillDisplay(uint8_t ch)
{
  int page;
  
  for (page = 0; page < NPGS; ++page)
    OLED_FillPage(page, ch);
}

void OLED_ClearDisplay(void)
{
  OLED_FillDisplay(0);
}

void OLED_Rotate(int bRotate)
{
  unsigned char remap, scan;
  
  if (bRotate) {
    remap = 0xA0;
    scan = 0xC0;
  }
  else {
    remap = 0xA1;
    scan = 0xC8;
  }

  OLED_Command(remap); // Set Segment Re-Map
  OLED_Command(scan); // Set Com Output Scan Direction
}

void OLED_Start(int bRotate)
{
  SPI_Start();
  
#ifdef IO_OLED_GND
  IO_Write(IOP_OLED_GND, 0);
  IO_Init(IOP_OLED_GND, IO_MODE_OUTPUT);
#endif  

#ifdef IO_OLED_VDD
  IO_Write(IOP_OLED_VDD, 1);
  IO_Init(IOP_OLED_VDD, IO_MODE_OUTPUT);
#endif  
  
#ifdef IO_OLED_CS
  IO_Write(IOP_OLED_CS, 1);
  IO_Init(IOP_OLED_CS, IO_MODE_OUTPUT);
#endif  
  
  IO_Init(IOP_OLED_DC, IO_MODE_OUTPUT);
  
#ifdef IO_OLED_RES
  // SSD1306 reset pulse
  IO_Write(IOP_OLED_RES, 0);
  IO_Init(IOP_OLED_RES, IO_MODE_OUTPUT);
  DelayMs(10);
  IO_Write(IOP_OLED_RES, 1);
#endif  
  DelayMs(80);

  //////////////////////////////////////
  
  OLED_Command(0xAE); // Set display OFF		

  OLED_Command(0xD4); // Set Display Clock Divide Ratio / OSC Frequency
  OLED_Command(0x80); // Display Clock Divide Ratio / OSC Frequency 

  OLED_Command(0xA8); // Set Multiplex Ratio
  OLED_Command(0x3F); // Multiplex Ratio for 128x64 (64-1)

  OLED_Command(0xD3); // Set Display Offset
  OLED_Command(0x00); // Display Offset

  OLED_Command(0x40); // Set Display Start Line

  OLED_Command(0x8D); // Set Charge Pump
  OLED_Command(0x14); // Charge Pump (0x10 External, 0x14 Internal DC/DC)

  OLED_Rotate(bRotate);

  OLED_Command(0xDA); // Set COM Hardware Configuration
  OLED_Command(0x12); // COM Hardware Configuration

  OLED_Command(0x81); // Set Contrast
  OLED_Command(0x80); // Contrast

  OLED_Command(0xD9); // Set Pre-Charge Period
  OLED_Command(0xF1); // Set Pre-Charge Period (0x22 External, 0xF1 Internal)

  OLED_Command(0xDB); // Set VCOMH Deselect Level
  OLED_Command(0x40); // VCOMH Deselect Level

  OLED_Command(0xA4); // Enable display outputs according to the GDDRAM contents
  OLED_Command(0xA6); // Set display not inverted
    
  OLED_ClearDisplay();

  OLED_Command(0xAF); // Set display On
  
  //////////////////////////////////////
  // Set default font
  OLED_SetFont(FNT_SMALL);
}

void OLED_UpdateDisplay(void)
{
  int i, page, segment;
  
  for (i = page = 0; page < NPGS; ++page) {
    OLED_SetPage(page);
    OLED_SetSegment(0);
    
    for (segment = 0; segment < NSEG; ++segment)
      OLED_Data(_DspRam[i++]);
  }
}

void OLED_SetFont(int font)
{
  switch (font) {
#ifdef FNT_SMALL    
  case FNT_SMALL:
    _szWidth = 5;
    _szHeight = 1;
    _pFont = g_ChrTab;
    _font = FNT_SMALL;
    break;
#endif    
#ifdef FNT_LARGE
  case FNT_LARGE:
    _szWidth = 7;
    _szHeight = 2;
    _pFont = g_ChrTab2;
    _font = FNT_LARGE;
    break;
#endif    
#ifdef FNT_BIG
  case FNT_BIG:
    _szWidth = 15;
    _szHeight = 4;
    _pFont = g_ChrTab3;
    _font = FNT_BIG;
    break;
#endif    
  }
}

void OLED_Scroll(int nLines)
{
  int i, j;
  
  // i: Kopya hedefi
  
  // j: Kopya kaynaðý
  j = nLines * NSEG;
  
  for (i = 0; i < (NPGS - nLines) * NSEG; ++i) // Bu dongu kaydirma yapar.
    _DspRam[i] = _DspRam[j++];
  
  for ( ; i < NPGS * NSEG; ++i) // Bu dongu kaydirilan kadar satirin disinda alttakileri sifirlar
    _DspRam[i] = 0;
  
  OLED_UpdateDisplay();
}

int OLED_GetFont(void)
{
  return _font;
}



void OLED_SetCursor(int row, int col)
{
  _row = row;
  _col = col;
}

void OLED_GetCursor(int *pRow, int *pCol)
{
  *pRow = _row;
  *pCol = _col;
}

void OLED_DrawBitmap(const uint8_t *pBitmap)
{
  int i;
  
  for (i = 0; i < NPGS * NSEG; ++i)
    _DspRam[i] = pBitmap[i];
  
  OLED_UpdateDisplay();
}

void OLED_Return(void) // CR islevi
{
  _col = 0;
}

void OLED_NewLine(void) // \n islevi
{
  if (++_row >= T_ROWS) {
    _row = T_ROWS - 1;
    
    OLED_Scroll(_szHeight);
  }
}

void OLED_PutChar(char c)
{
  int i, j, k;
    
  if (_col >= T_COLS) {
    OLED_Return();
    OLED_NewLine();
  }
  
#ifdef FNT_BIG
  if (_font == FNT_BIG) {
    if (c < 32 || c > 127)
      c = 32;
    
    c -= 32;
  }
#endif    
  
  for (k = 0; k < _szHeight; ++k) {
    OLED_SetPage(_row * _szHeight + k);
    OLED_SetSegment(_col * (_szWidth + 1));
    
    i = _szWidth * _szHeight * c + k;
    
    for (j = 0; j < _szWidth; ++j) {
      OLED_Data(_pFont[i]);
      i += _szHeight;
    }
                
    OLED_Data(0);
  }
                
  ++_col;
}

////////////////////////////////////////////////////////////////////////////////

void OLED_PixelData(int x, int y, int c)
{
  uint8_t page, bitIdx, val;
  
  x &= 0x7F;
  y &= 0x3F;
  
  page = y >> 3;
  //page = y / 8;
  
  bitIdx = y & 7;
  //bitIdx = y % 8;
  
  val = _DspRam[page * NSEG + x];
  
  switch (c) {
  case OLED_SETPIXEL:
    val |= (1 << bitIdx);
    break;
    
  case OLED_CLRPIXEL:
    val &= ~(1 << bitIdx);
    break;
    
  case OLED_INVPIXEL:
    val ^= (1 << bitIdx);
    break;
  }
  
  _DspRam[page * NSEG + x] = val;
}

void OLED_SetPixel(int x, int y, int c)
{
  uint8_t page, bitIdx, val;
  
  x &= 0x7F;
  y &= 0x3F; // y en fazla 63
  
  page = y >> 3; // 8'e bölümünün sonucu. page = y / 8;
  bitIdx = y & 7; // 8'e bölümünden kalan. bitIdx = y % 8;
  
  val = _DspRam[page * NSEG + x]; // söz konusu pixelin hangi ram bölgesien karþýlýk geldiði
  
  switch (c) {
  case OLED_SETPIXEL:
    val |= (1 << bitIdx);
    break;
    
  case OLED_CLRPIXEL:
    val &= ~(1 << bitIdx);
    break;
    
  case OLED_INVPIXEL:
    val ^= (1 << bitIdx);
    break;
  }
  
  OLED_SetPage(page);
  OLED_SetSegment(x);
  OLED_Data(val);
}

int OLED_GetPixel(int x, int y)
{
  uint8_t page, bitIdx, val;
  
  x &= 0x7F;
  y &= 0x3F;
  
  page = y >> 3;
  bitIdx = y & 7;

  val = _DspRam[(unsigned)page * NSEG + x];
  
  return (val & (1 << bitIdx)) != 0;
}

#define abs(a)      (((a) > 0) ? (a) : -(a))

void OLED_Line(int x0, int y0, int x1, int y1, int c)
{
     int steep, t ;
     int deltax, deltay, error;
     int x, y;
     int ystep;

     steep = abs(y1 - y0) > abs(x1 - x0);

     if (steep)
     { // swap x and y
         t = x0; x0 = y0; y0 = t;
         t = x1; x1 = y1; y1 = t;
     }

     if (x0 > x1)
     {  // swap ends
         t = x0; x0 = x1; x1 = t;
         t = y0; y0 = y1; y1 = t;
     }

     deltax = x1 - x0;
     deltay = abs(y1 - y0);
     error = 0;
     y = y0;

     if (y0 < y1) 
         ystep = 1;
     else
         ystep = -1;

     for (x = x0; x < x1; x++)
     {
         if (steep)
            OLED_SetPixel(y, x, c);
         else
            OLED_SetPixel(x, y, c);

         error += deltay;
         if ((error << 1) >= deltax)
         {
             y += ystep;
             error -= deltax;
         } // if
     } // for
} // line

/*
void OLED_Circle(int x, int y, int r, int c)
{
    float step, t;
    int dx, dy;

    step = PI / 2 / 64;

    for (t = 0; t <= PI / 2; t += step) {
        dx = (int)(r * cos(t) + 0.5);
        dy = (int)(r * sin(t) + 0.5);

        if (x + dx < 128) {
            if (y + dy < 64)
                OLED_SetPixel(x + dx, y + dy, c);
            if (y - dy >= 0)
                OLED_SetPixel(x + dx, y - dy, c);
        }
        if (x - dx >= 0) {
            if (y + dy < 64)
                OLED_SetPixel(x - dx, y + dy, c);
            if (y - dy >= 0)
                OLED_SetPixel(x - dx, y - dy, c);
        }
    }
}
*/
/////////////////////////////////

void OLED_putch(char c) // printf ile kullanilabilmesi icin.
{
  switch (c) {
  case '\n':
    OLED_NewLine();
    
  case '\r':
    OLED_Return();
    break;
    
  case '\f':
    OLED_ClearDisplay();
    break;
    
  default:
    OLED_PutChar(c);
    break;
  }
}

