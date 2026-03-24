#include <clock.h>
#include <stdint.h>
#include <stdbool.h>
#include "wait.h"
#include "tm4c123gh6pm.h"

// Bitband alias
#define GREEN_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define CONTROL   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 0*4))) // PB7


// PortF masks
#define GREEN_LED_MASK 8
#define CONTROL_MASK 1

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5|SYSCTL_RCGCGPIO_R1;
    _delay_cycles(3);

    // Configure LED pin
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK;  // make bit an output
    GPIO_PORTF_DEN_R |= GREEN_LED_MASK;  // enable LED
    // configure control
    GPIO_PORTB_DIR_R |= CONTROL_MASK; //MAKE OUTPUT
    GPIO_PORTB_DEN_R |= CONTROL_MASK;

}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // Initialize hardware
    initHw();

    // Toggle green LED every second
    while(true)
    {
        CONTROL =1;
        waitMicrosecond(55000);
        CONTROL =0;
        waitMicrosecond(93000);
    }
}
