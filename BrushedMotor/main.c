#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "clock.h"
#include "wait.h"
#include "gpio.h"
#include "adc0.h"

#define ADC_IN PORTB, 4
#define PWM_MASK        8       // PF3
#define FREQ_IN_MASK    64      // PortC masks
#define NUMBER_OF_SLOTS 32
#define DEBOUNCE_US     200000  // 200 ms software debounce
#define VREF 3.3
#define ADC_MAX 4095

//-----------------------------------------------------------------------------
// Pin layout
//-----------------------------------------------------------------------------
// PF3 - PWM
// PC6 - CCP
// PB4 - ADC

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
uint32_t frequency = 0;
uint32_t time = 0;
uint32_t adcValue;
volatile uint32_t lastButtonTime = 0;  // For button debounce
volatile uint32_t rpm = 0;
volatile uint8_t dutyCycleIndex = 4;   // Start at 50%
volatile uint16_t pwmValue = 1000;
volatile uint16_t backRPM = 0;
float Vout;
float Vback;
uint32_t resistorDivider;

// Duty cycle table for PWM (0% -> 90%)
const uint16_t dutyCycleTable[10] =
{
    0,    // 0%
    200,  // 10%
    400,  // 20%
    600,  // 30%
    800,  // 40%
    1000, // 50%
    1200, // 60%
    1400, // 70%
    1600, // 80%
    1800, // 90%
};

//-----------------------------------------------------------------------------
// Pushbutton initialization (PF0 and PF4)
//-----------------------------------------------------------------------------
void initPB()
{
    enablePort(PORTF);

    // Configuring PF0 as push button
    setPinCommitControl(PORTF, 0);

    // Setting both PF0 and PF4 as digital inputs
    selectPinDigitalInput(PORTF, 0);
    selectPinDigitalInput(PORTF, 4);

    // Enabling pull-up resistors
    enablePinPullup(PORTF, 0);
    enablePinPullup(PORTF, 4);

    // Configure interrupt for both pins
    selectPinInterruptFallingEdge(PORTF, 0);
    selectPinInterruptFallingEdge(PORTF, 4);

    // Clear any prior interrupt
    clearPinInterrupt(PORTF, 0);
    clearPinInterrupt(PORTF, 4);

    // Enable interrupts
    enablePinInterrupt(PORTF, 0);
    enablePinInterrupt(PORTF, 4);

    // Enable GPIOF interrupt in NVIC (IRQ30)
    NVIC_EN0_R |= 1 << (INT_GPIOF - 16);
}

//-----------------------------------------------------------------------------
// GPIOF ISR for buttons
//-----------------------------------------------------------------------------
void GPIOF_Handler(void)
{
    uint32_t now = TIMER1_TAV_R;
    if (now - lastButtonTime < DEBOUNCE_US)
    {
        clearPinInterrupt(PORTF, 0);
        clearPinInterrupt(PORTF, 4);
        return;
    }
    lastButtonTime = now;

    if (getPinValue(PORTF, 0) == 0 && dutyCycleIndex < 9)
    {
        dutyCycleIndex++;
        //putsUart0("Index increased \r\n");
    }

    if (getPinValue(PORTF, 4) == 0 && dutyCycleIndex > 0)
    {
        dutyCycleIndex--;
        //putsUart0("Index decreased \r\n");
    }

    pwmValue = dutyCycleTable[dutyCycleIndex];
    PWM1_3_CMPB_R = pwmValue;

    clearPinInterrupt(PORTF, 0);
    clearPinInterrupt(PORTF, 4);
}

//-----------------------------------------------------------------------------
// PWM initialization (PF3)
//-----------------------------------------------------------------------------
void initPWM()
{
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    enablePort(PORTF);

    GPIO_PORTF_DEN_R |= PWM_MASK;
    GPIO_PORTF_AFSEL_R |= PWM_MASK;            // Switch to alternative function(PWM)
    GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF3_M);   // Clear old mux
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF3_M1PWM7; // map PF3 -> M1PWM7

    // Reset PWM1 safely
    SYSCTL_SRPWM_R |= SYSCTL_SRPWM_R1;
    SYSCTL_SRPWM_R &= ~SYSCTL_SRPWM_R1;

    // Configure PWM generator 3
    PWM1_3_CTL_R = 0;
    // Output LOW at LOAD, HIGH at CMPB
    PWM1_3_GENB_R = PWM_1_GENB_ACTLOAD_ZERO | PWM_1_GENB_ACTCMPBD_ONE;
    PWM1_3_LOAD_R = 2000;

    // Start disabled (50% duty)
    PWM1_3_CMPB_R = 1000;

    PWM1_3_CTL_R = PWM_1_CTL_ENABLE;
    PWM1_ENABLE_R = PWM_ENABLE_PWM7EN;
}

//-----------------------------------------------------------------------------
// CCP input for frequency measurement (PC6)
//-----------------------------------------------------------------------------
void initCCP()
{
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;
    _delay_cycles(3);

    GPIO_PORTC_AFSEL_R |= FREQ_IN_MASK;    // select alternative functions for SIGNAL_IN pin
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC6_M; // clear old mapping
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC6_WT1CCP0;
    GPIO_PORTC_DEN_R |= FREQ_IN_MASK;      // enable bit 6 for digital input
}

//-----------------------------------------------------------------------------
// Timer2A for 50 Hz periodic interrupt (ADC sampling)
//-----------------------------------------------------------------------------
void initTimer50Hz()
{
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2; // Timer2 clock
    _delay_cycles(3);

    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;        // Disable timer
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;  // 32-bit timer
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // Periodic mode (count down)
    TIMER2_TAILR_R = 800000 - 1;            // 50 Hz period: 40 MHz / 50 = 800,000 cycles
    TIMER2_IMR_R = TIMER_IMR_TATOIM;        // Enable timeout interrupt
    NVIC_EN0_R |= 1 << (INT_TIMER2A - 16);  // Enable Timer2A IRQ in NVIC
    TIMER2_CTL_R |= TIMER_CTL_TAEN;         // Enable timer

}

void Timer2A_Handler(void)
{
    uint16_t currentPWM = PWM1_3_CMPB_R;  // save current duty
    PWM1_3_CMPB_R = 0;                     // optional: 0% for ADC
    waitMicrosecond(350);

    // Find Vout and Vback
    adcValue = readAdc0Ss3();
    Vout = ((float)adcValue / 4095.0F) * VREF;
    Vback = 10.0f - (Vout*5.7f);

    PWM1_3_CMPB_R = currentPWM;            // restore duty cycle
    TIMER2_ICR_R = TIMER_ICR_TATOCINT;
    //putsUart0("In 50hz timer \r\n");
}

//-----------------------------------------------------------------------------
// Hardware initialization
//-----------------------------------------------------------------------------
void initHW()
{
    initSystemClockTo40Mhz();
    enablePort(PORTB);
    initUart0();
    setUart0BaudRate(115200, 40e6);

    initPB();
    initPWM();
    initCCP();
    initTimer50Hz();

    // ADC0 SS3 configuration for AIN10 (PB4)
    _delay_cycles(3);
    selectPinAnalogInput(ADC_IN);
    initAdc0Ss3();
    setAdc0Ss3Mux(10);
    setAdc0Ss3Log2AverageCount(2);
}

//-----------------------------------------------------------------------------
// Counter mode (Timer1 + Wide Timer1)
//-----------------------------------------------------------------------------
void enableCounterMode()
{
    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
    TIMER1_TAILR_R = 40000000; // 1 Hz
    TIMER1_IMR_R = TIMER_IMR_TATOIM;
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);

    // Configure Wide Timer 1 as counter of external events on CCP0 pin
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER1_CFG_R = 4; // 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
    WTIMER1_CTL_R = TIMER_CTL_TAEVENT_POS;
    WTIMER1_IMR_R = 0;
    WTIMER1_TAV_R = 0; // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;
}

void disableCounterMode()
{
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    NVIC_DIS0_R = 1 << (INT_TIMER1A-16);
}

//-----------------------------------------------------------------------------
// Read frequency from event counter
//-----------------------------------------------------------------------------
void timerFrequency()
{
    frequency = WTIMER1_TAV_R;
    WTIMER1_TAV_R = 0;         // reset counter for next period
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
    //putsUart0("In frequency timer\r\n");
}

void disableTimerMode()
{
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter
    NVIC_DIS3_R = 1 << (INT_WTIMER1A-16-96);         // turn-off interrupt 112 (WTIMER1A)
}

//-----------------------------------------------------------------------------
//  Calculate RPM from back emf
//-----------------------------------------------------------------------------
float calcBackRPM(float Vback){
    float Kv = 0.0031;              // Calculated empericaly y =0.0031 * RPM - 1.3268
    return (Vback + 1.3268)/ Kv;
}



int main()
{
    initHW();

    // Set up counter mode
    disableTimerMode();
    enableCounterMode();
    uint8_t dutyPercent = 0;

    while (1)
    {
        // Calculate RPM using the collector
        rpm = (frequency * 60) / NUMBER_OF_SLOTS;

        // Calculate RPM using back emf
        backRPM = (uint16_t)calcBackRPM(Vback);


        // Print latest measurement every second
        putsUart0("Duty Cycle: ");
        if (dutyCycleIndex != 0)
        {
            dutyPercent = 10 *(dutyCycleIndex + 1);
        }
        else
        {
            dutyPercent = dutyCycleIndex;
        }
        putintUart0(dutyPercent, false);
        putcUart0('%');
        putsUart0("\t");
        putsUart0("ADC: ");
        putintUart0(adcValue,false);
        putsUart0("\t");
        putsUart0("RPM: ");
        putintUart0(rpm, false);
        putsUart0("\t");
        putsUart0("Back RPM: ");
        putintUart0(backRPM, false);
        putsUart0("\t");
        putsUart0("Frequency: ");
        putintUart0(frequency, false);
        putsUart0("\t");
        putsUart0("Back EMF: ");
        putfloatUart0(Vback,4);
        putsUart0("\r\n\n\n\n");
        waitMicrosecond(500000);
    }
}
