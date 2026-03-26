#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart1.h"


// PC4 - Uart 1 Rx
// PC5 - Uart 1 Tx
#define UART_TX_MASK 0x20   // PC5
#define UART_RX_MASK 0x10   // PC4

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize UART1
void  initUart1()
{
    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;
    _delay_cycles(3);

    // Configure UART1 pins
    GPIO_PORTC_DR2R_R |= UART_TX_MASK | UART_RX_MASK;             // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTC_DEN_R |= UART_TX_MASK | UART_RX_MASK;    // enable digital on UART1 pins
    GPIO_PORTC_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PA0, PA1
    GPIO_PORTC_PCTL_R &= ~(GPIO_PCTL_PC4_M | GPIO_PCTL_PC5_M); // clear bits 0-7
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;
                                                        // select UART1 to drive pins PC4 and PC5: default, added for clarity

    // Configure UART1 to 115200 baud, 8N1 format
    UART1_CTL_R = 0;                                    // turn-off UART1 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    UART1_IBRD_R = 21;                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART1_FBRD_R = 45;                                  // round(fract(r)*64)=45
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // enable TX, RX, and module
}

// Set baud rate as function of instruction cycle frequency
void setUart1BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
                                                        // where r = fcyc / 16 * baudRate
    divisorTimes128 += 1;                               // add 1/128 to allow rounding
    UART1_CTL_R = 0;                                    // turn-off UART1 to allow safe programming
    UART1_IBRD_R = divisorTimes128 >> 7;                // set integer value to floor(r)
    UART1_FBRD_R = ((divisorTimes128) >> 1) & 63;       // set fractional value to round(fract(r)*64)
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // turn-on UART1
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart1(char c)
{
    while (UART1_FR_R & UART_FR_TXFF);               // wait if uart1 tx fifo full
    UART1_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart1(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart1(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart1()
{
    while (UART1_FR_R & UART_FR_RXFE);               // wait if uart1 rx fifo empty
    return UART1_DR_R & 0xFF;                        // get character from fifo
}

// Returns the status of the receive buffer
bool kbhitUart1()
{
    return !(UART1_FR_R & UART_FR_RXFE);
}

// Send unsigned integer in decimal over UART
void putintUart1(uint32_t num)
{
    char buf[12];  // enough for 32-bit unsigned (max: 4294967295)
    int i = 0;

    // Special case for zero
    if (num == 0)
    {
        putcUart1('0');
        return;
    }

    // Convert number to decimal ASCII (in reverse order)
    while (num > 0)
    {
        buf[i++] = (num % 10) + '0';
        num /= 10;
    }

    // Send digits in correct order
    while (i > 0)
    {
        putcUart1(buf[--i]);
    }
}

// Send a floating-point number over UART with a given number of decimal places
void putfloatUart1(float num, uint8_t decimals)
{
    if (num < 0) {
        putcUart1('-');
        num = -num;
    }

    // Extract integer part
    uint32_t intPart = (uint32_t)num;

    // Extract fractional part
    float fracPart = num - (float)intPart;

    // Convert fractional part to integer based on requested decimals
    uint32_t fracInt = 0;
    uint8_t i;
    for (i = 0; i < decimals; i++) {
        fracPart *= 10.0f;
    }
    fracInt = (uint32_t)(fracPart + 0.5f); // round

    // Print integer part
    putintUart1(intPart);

    // Print decimal point and fractional part
    if (decimals > 0) {
        putcUart1('.');
        // Handle leading zeros in fractional part
        uint32_t divisor = 1;
        for (i = 1; i < decimals; i++) divisor *= 10;
        while (divisor > 0) {
            putcUart1((fracInt / divisor) % 10 + '0');
            divisor /= 10;
        }
    }
}

// Send a single byte as 2-digit hex
void puthex8Uart1(uint8_t value)
{
    char hexDigits[] = "0123456789ABCDEF";
    putcUart1(hexDigits[(value >> 4) & 0xF]);
    putcUart1(hexDigits[value & 0xF]);
}

// Send multiple hex values from an array
void puthexArrayUart1(const uint8_t *data, uint32_t length)
{
    uint32_t i;
    for (i = 0; i < length; i++)
    {
        puthex8Uart1(data[i]);
        putcUart1(' ');           // optional separator
    }
}

void putBytesUart1(uint8_t *data, uint32_t length)
{
    uint32_t i;
    for ( i = 0; i < length; i++)
        putcUart1(data[i]);   // send raw byte
}

