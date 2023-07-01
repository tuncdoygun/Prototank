#ifndef _OLED_H
#define _OLED_H


#define NPGS    8
#define NSEG    128

enum {
  OLED_CLRPIXEL,
  OLED_SETPIXEL,
  OLED_INVPIXEL
};

// Font sabitleri
#define FNT_SMALL       0       // 5x8
#define FNT_LARGE       1       // 7x16
#define FNT_BIG         2       // 15x32
  
extern uint8_t  _DspRam[];

void OLED_Data(uint8_t data);
void OLED_SetPage(uint8_t page);
void OLED_SetSegment(uint8_t segment);
void OLED_Rotate(int bRotate);
void OLED_Start(int bRotate);
void OLED_SetPixel(int x, int y, int c);
int OLED_GetPixel(int x, int y);
void OLED_UpdateDisplay(void);
void OLED_Scroll(int nLines);

void OLED_SetFont(int font);
void OLED_SetCursor(int row, int col);
void OLED_GetCursor(int *pRow, int *pCol);
void OLED_PutChar(char c);

void OLED_putch(char c);

void OLED_Line(int x0, int y0, int x1, int y1, int c);
void OLED_Circle(int x, int y, int r, int c);

void OLED_PixelData(int x, int y, int c);

#endif
