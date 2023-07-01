#ifndef _UART_H
#define	_UART_H

enum {
	UART_1,
	UART_2,
	UART_3
};

#define SZ_PRNBUF	256

void UART_Init(int idx, int baud);
void UART_Send(int idx, unsigned char val);
void UART_putch(unsigned char c);
int UART_puts(const char *str);
int UART_printf(const char *fmt, ...);

int UART_DataReady(int idx);
unsigned char UART_Recv(int idx);

extern int g_conUART; // console uart

#endif
