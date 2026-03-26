#ifndef UART1_H_
#define UART1_H_

#include <stdint.h>
#include <stdbool.h>

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initUart1();
void setUart1BaudRate(uint32_t baudRate, uint32_t fcyc);
void putcUart1(char c);
void putsUart1(char* str);
char getcUart1();
bool kbhitUart1();
void putintUart1(uint32_t num);
void putfloatUart1(float num, uint8_t decimals);
void puthex8Uart1(uint8_t value);
void puthexArrayUart1(const uint8_t *data, uint32_t length);
void putBytesUart1(uint8_t *data, uint32_t length);

#endif
