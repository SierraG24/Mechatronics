#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "clock.h"
#include "wait.h"
#include "uart0.h"
#include "CTI.h"
#include "tm4c123gh6pm.h"

#define DEBUG 1 // Comment out or set to 0 to disable debug output

// Bit Band Alias
#define COIL_A_LEFT (*((volatile uint32_t *)(0x42000000 + (0x400053FC - 0x40000000) * 32 + 4 * 4)))  // PB4
#define COIL_A_RIGHT (*((volatile uint32_t *)(0x42000000 + (0x400053FC - 0x40000000) * 32 + 5 * 4))) // PB5
#define COIL_B_LEFT (*((volatile uint32_t *)(0x42000000 + (0x400053FC - 0x40000000) * 32 + 6 * 4)))  // PB6
#define COIL_B_RIGHT (*((volatile uint32_t *)(0x42000000 + (0x400053FC - 0x40000000) * 32 + 7 * 4))) // PB7
#define COLLECTOR (*((volatile uint32_t *)(0x42000000 + (0x400073FC - 0x40000000) * 32 + 6 * 4)))    // PD6

// Masks
#define COIL_A_LEFT_MASK 16
#define COIL_A_RIGHT_MASK 32
#define COIL_B_LEFT_MASK 64
#define COIL_B_RIGHT_MASK 128
#define A_ENABLE_MASK 4   // PF2
#define B_ENABLE_MASK 8   // PF3
#define COLLECTOR_MASK 64 // PD6

// Step delay in microseconds
// 16 = 50000
// 8 = 100000
#define STEPDELAY 500000

// Motor geometry
#define NP 50
#define FULL_STEP_DEG 1.8f
// Chance the microstepping mode here (4, 8, or 16)
#define MICROSTEPS 16

// Global Variables
volatile uint16_t sineTable[MICROSTEPS][2];     // Duty cycle for Coil A and B
volatile uint8_t directionTable[MICROSTEPS][4]; // 4 coils A+, A-, B+, B-
volatile uint32_t PhaseIndex = 0;               // current microstep 0..MICROSTEPS-1
volatile int32_t completeElecRotation = 0;      // signed so CCW decrements safely
volatile bool hitCollector = false;
volatile uint32_t balanceIndex; // Index where the beam is balanced

// Sine values were calculated by doing load value * sin(angle in radians)
// where load value = 2000 for 40MHz clock with PWM divisor of 2
void buildDutyPercentTable(uint8_t mode)
{
    uint8_t step;
    // Full steps
    if (mode == 4)
    {
        uint16_t table_full[4][2] = {
            {2000, 0}, // A=100%, B=0
            {0, 2000}, // A=0,    B=100%
            {2000, 0}, // A=100%, B=0
            {0, 2000}  // A=0,    B=100%
        };
        for (step = 0; step < 4; step++)
        {
            uint8_t j;
            for (j = 0; j < 2; j++)
                sineTable[step][j] = table_full[step][j];
        }
    }
    // Half Steps
    else if (mode == 8)
    {
        uint16_t table_half[8][2] = {
            {2000, 0},
            {1415, 1415},
            {0, 2000},
            {1415, 1415},
            {2000, 0},
            {1415, 1415},
            {0, 2000},
            {1415, 1415}};
        for (step = 0; step < 8; step++)
        {
            uint8_t j;
            for (j = 0; j < 2; j++)
                sineTable[step][j] = table_half[step][j];
        }
    }
    // Quarter Steps
    else if (mode == 16)
    {
        uint16_t table_quarter[16][2] = {
                    {2000,    0}, //   0.0
                    {1848,  765}, //  22.5
                    {1414, 1414}, //  45.0
                    { 765, 1848}, //  67.5
                    {   0, 2000}, //  90.0
                    { 765, 1848}, // 112.5
                    {1414, 1414}, // 135.0
                    {1848,  765}, // 157.5
                    {2000,    0}, // 180.0
                    {1848,  765}, // 202.5
                    {1414, 1414}, // 225.0
                    { 765, 1848}, // 247.5
                    {   0, 2000}, // 270.0
                    { 765, 1848}, // 292.5
                    {1414, 1414}, // 315.0
                    {1848,  765}  // 337.5
                };
        for (step = 0; step < 16; step++)
        {
            uint8_t j;
            for (j = 0; j < 2; j++)
                sineTable[step][j] = table_quarter[step][j];
        }
    }
}

// Calculate direction table for microstepping
// if (cos(angle) > 0) A+ else A-
// if (sin(angle) > 0) B+ else B-
void buildDirectionTable(uint8_t mode)
{
    uint8_t step = 0;
    // Full steps
    if (mode == 4)
    {
        uint8_t full[4][4] = {
            {1, 0, 0, 0}, // A+ B+
            {0, 0, 1, 0}, // A- B+
            {0, 1, 0, 0}, // A- B-
            {0, 0, 0, 1}  // A+ B-
        };
        for (step = 0; step < 4; step++)
        {
            uint8_t j;
            for (j = 0; j < 4; j++)
                directionTable[step][j] = full[step][j];
        }
    }
    // Half Steps
    else if (mode == 8)
    {
        uint8_t half[8][4] = {
            {1, 0, 0, 0}, // A+
            {1, 0, 1, 0}, // A+ B+
            {0, 0, 1, 0}, // B+
            {0, 1, 1, 0}, // A- B+
            {0, 1, 0, 0}, // A-
            {0, 1, 0, 1}, // A- B-
            {0, 0, 0, 1}, // B-
            {1, 0, 0, 1}  // A+ B-
        };
        for (step = 0; step < 8; step++)
        {
            uint8_t j;
            for (j = 0; j < 4; j++)
                directionTable[step][j] = half[step][j];
        }
    }
    // Quarter Steps
    else if (mode == 16)
    {
        uint8_t quarter[16][4] = {
            {1, 0, 1, 0}, // A+
            {1, 0, 1, 0}, // A+ B+
            {1, 0, 1, 0}, // B+
            {1, 0, 1, 0}, // A- B+
            {0, 1, 1, 0}, // A-
            {0, 1, 1, 0}, // A- B-
            {0, 1, 1, 0}, // B-
            {0, 1, 1, 0}, // A+ B-
            {0, 1, 0, 1}, // repeat A+
            {0, 1, 0, 1}, // repeat A+ B+
            {0, 1, 0, 1}, // repeat B+
            {0, 1, 0, 1}, // repeat A- B+
            {1, 0, 0, 1}, // repeat A-
            {1, 0, 0, 1}, // repeat A- B-
            {1, 0, 0, 1}, // repeat B-
            {1, 0, 0, 1}  // repeat A+ B-
        };
        for (step = 0; step < 16; step++)
        {
            uint8_t j;
            for (j = 0; j < 4; j++)
                directionTable[step][j] = quarter[step][j];
        }
    }
}

// Using pins PF2, and PF3 for PWM
void enablePWM()
{
    // Enable the clock to the PWM module 1
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    // Enable the clock to Port F
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    GPIO_PORTF_DEN_R |= A_ENABLE_MASK | B_ENABLE_MASK;
    GPIO_PORTF_AFSEL_R |= A_ENABLE_MASK | B_ENABLE_MASK;              // Switch to alternative function(PWM)
    GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF2_M | GPIO_PCTL_PF3_M);        // Clear old mux
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7; // map PF2 -> M1PWM6, PF3->M1PWM7

    // A_ENABLE  on M1PWM6 (PF2), M1PWM3a
    // B_ENABLE on M1PWM7 (PF3), M1PWM3b
    // Fix PWM enable macro and safe reset
    SYSCTL_SRPWM_R |= SYSCTL_SRPWM_R1;  // reset PWM1
    SYSCTL_SRPWM_R &= ~SYSCTL_SRPWM_R1; // release reset

    PWM1_3_CTL_R = 0;
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ZERO | PWM_1_GENA_ACTLOAD_ONE;
    PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;
    PWM1_3_LOAD_R = 2000; // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz

    // Start disabled (0% duty)
    PWM1_3_CMPA_R = PWM1_3_LOAD_R;
    PWM1_3_CMPB_R = PWM1_3_LOAD_R;

    PWM1_3_CTL_R = PWM_1_CTL_ENABLE;
    PWM1_ENABLE_R = PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
}

// Setting in a cmp value that changes the duty cycle percent
// Now expects scaled values 0–1023 directly
// This might be wrong
void setPWMDuty(uint16_t pwmA, uint16_t pwmB)
{
    PWM1_3_CMPA_R = pwmA;
    PWM1_3_CMPB_R = pwmB;
}

void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks for port B,D,F
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R3 | SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Set collector pin as input
    GPIO_PORTD_DIR_R &= ~COLLECTOR_MASK;

    // Setting Pins as outputs
    GPIO_PORTB_DIR_R |= COIL_A_LEFT_MASK;
    GPIO_PORTB_DIR_R |= COIL_A_RIGHT_MASK;
    GPIO_PORTB_DIR_R |= COIL_B_LEFT_MASK;
    GPIO_PORTB_DIR_R |= COIL_B_RIGHT_MASK;

    // Initialize the UART0
    // setUart0BaudRate(115200, 16);
    initUart0();

    // Set Digital Enabled
    GPIO_PORTB_DEN_R |= COIL_A_LEFT_MASK;
    GPIO_PORTB_DEN_R |= COIL_A_RIGHT_MASK;
    GPIO_PORTB_DEN_R |= COIL_B_LEFT_MASK;
    GPIO_PORTB_DEN_R |= COIL_B_RIGHT_MASK;
    GPIO_PORTD_DEN_R |= COLLECTOR_MASK;

    enablePWM();
}

// Set PWM + digital outputs for a microstep index
static void setPhase(uint32_t idx)
{
    idx %= MICROSTEPS;

    // blank PWM to avoid glitch
    setPWMDuty(0, 0);

    // Explicit 0/1 writes
    COIL_A_LEFT = (uint32_t)directionTable[idx][0];
    COIL_A_RIGHT = (uint32_t)directionTable[idx][1];
    COIL_B_LEFT = (uint32_t)directionTable[idx][2];
    COIL_B_RIGHT = (uint32_t)directionTable[idx][3];

    // restore duty
    setPWMDuty(sineTable[idx][0], sineTable[idx][1]);

    PhaseIndex = idx;
}

// CW: move backward through table (decrease PhaseIndex)
void stepCCW(uint32_t microsteps)
{
    uint32_t s;
    for (s = 0; s < microsteps; s++)
    {
        uint32_t next = (PhaseIndex + MICROSTEPS - 1) % MICROSTEPS;

        // If we were at index 0 and move backward, we completed an electrical rotation
        if (PhaseIndex == 0)
            completeElecRotation++;

        setPhase(next);
        waitMicrosecond(STEPDELAY);
    }
}

// CCW: move forward through table (increase PhaseIndex)
void stepCW(uint32_t microsteps)
{
    uint32_t s;
    for (s = 0; s < microsteps; s++)
    {
        uint32_t next = (PhaseIndex + 1) % MICROSTEPS;

        // If we wrapped forward back to 0, decrement rotation count
        if (next == 0)
            completeElecRotation--;

        setPhase(next);
        waitMicrosecond(STEPDELAY);
    }
}

// mechanical microsteps = degrees/360 * MICROSTEPS * polePairs
// Rotate a signed number of degrees (negative = CW, positive = CCW)
volatile uint32_t microstepsTarget;
void rotateDegrees(float degrees)
{
    // Determine direction
    bool ccw = (degrees > 0);
    float absDegrees = fabsf(degrees);

    // Compute microsteps
    float stepsPerRev = (float)NP * (float)MICROSTEPS;      // e.g., 50 * 16 = 800
    float microstepAngle = 360.0f / stepsPerRev;
    uint32_t microstepsTarget = (uint32_t)ceilf(absDegrees / microstepAngle);

    if (microstepsTarget == 0)
        microstepsTarget = 1; // ensure at least 1 step  

    // Rotate motor
    if (ccw)
        stepCCW(microstepsTarget);
    else
        stepCW(microstepsTarget);
}


// Calibrate motor and set origin at balanceIndex
void balanceBeam()
{
    hitCollector = false;

    // Spin CW until the collector sensor is triggered
    while (!hitCollector)
    {
        stepCW(1);  // step one microstep at a time CW
        if (GPIO_PORTD_DATA_R & COLLECTOR_MASK)
        {
            hitCollector = true;
            PhaseIndex = 0;
            completeElecRotation = 0;
            putsUart0("Hit collector\r\n");
        }
    }

    // Small delay
    waitMicrosecond(10000);

    // Move CCW to the balance index (origin)
    uint32_t i;
    if (MICROSTEPS != 4)
        balanceIndex = 45;
    else
        balanceIndex = 25;

    for (i = 0; i < balanceIndex; i++)
        stepCCW(1);

    putsUart0("Beam balanced\r\n");

    // After this, PhaseIndex is at balanceIndex (logical origin)
    // All future rotateDegrees() calls are relative to this position
}

// ------------------------------------------------------------------
// Main
// ------------------------------------------------------------------
int main(void)
{
    // Initialize hardware
    initHw();

    // Set tables
    buildDutyPercentTable(MICROSTEPS);
    buildDirectionTable(MICROSTEPS);
    setPhase(0); // ensure outputs start in a valid, known state
    PhaseIndex = 0;
    // Create struct data
    USER_DATA data;
    bool first = true;
    balanceBeam();

    // Loop to ask for angle using
    while (1)
    {
        if (first)
        {
            putsUart0("\n=== Stepper Motor Controller === \r\n");
            putsUart0("System Ready. Awaiting angle input...\r\n");
            putsUart0("Format: set 00.00 \r\n");
            putsUart0("Example: set 45.25  -> +45.25 degrees\r\n");
            putsUart0("         set -90 00  -> -90.00 degrees\r\n\n");
            first = false;
        }
        else
        {
            putsUart0("Input Command: \r\n");
        }
        getsUart0(&data);

// Echo back to the user of the TTY interface for testing
#ifdef DEBUG
        putsUart0(data.buffer);
        putsUart0("\r\n");
#endif

        parseFields(&data);

        // Command evaluation
        bool valid = false;

        if (isCommand(&data, "set", 1))
        {
            float angle = getFieldFloat(&data, 1);  // signed angle
            rotateDegrees(angle);                    // negative -> CW, positive -> CCW
            valid = true;
        }
        else if (isCommand(&data, "cal", 0))
        {
            balanceBeam();   // calibrate and set balanceIndex
            valid = true;
        }

        if (!valid)
        {
            putsUart0("Error: Invalid command format.\r\n");
            putsUart0("Expected: set [p|n] [whole] [fraction]\r\n\n");
        }
    }
}
