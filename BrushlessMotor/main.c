#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "clock.h"
#include "wait.h"
#include "CTI.h"
#include "gpio.h"
#include "uart0.h"

//-----------------------------------------------------------------------------
// Hardware configuration
//-----------------------------------------------------------------------------

// PWM:          PB5, PB0, PB1
// Coils:        PF1, PF2, PF3
// Hall sensors: PC4, PC5, PC6
// Button:       PF4
#define PWM_A_PIN 5
#define PWM_B_PIN 0
#define PWM_C_PIN 1
#define PWM_PORT  PORTB

#define COIL_A_PIN 1
#define COIL_B_PIN 2
#define COIL_C_PIN 3
#define COIL_PORT  PORTF

#define SENSE_A_PIN 4
#define SENSE_B_PIN 5
#define SENSE_C_PIN 6
#define SENSE_PORT  PORTC

#define SW1_PIN 4
#define SW1_PORT PORTF


// phaseDrive[state][A,B,C]
// +1 = high (drive +V)
//  0 = grounded (drive 0V)
// -1 = floating (high impedance)
static const int8_t phaseDrive[6][3] = {
    { +1,   0,  -1},  // Step 1: C high, B low, A float
    { -1,   0, +1 },  // Step 2: A high, B low, C float
    {  0,  -1, +1 },  // Step 3: A high, C float, B low
    {  0,  +1, -1 },  // Step 4: B high, C float, A low
    { -1,  +1,  0 },  // Step 5: B high, A float, C low
    { +1,  -1,  0 }   // Step 6: C high, A low, B float
};


// hallStates[state][A,B,C] Counterclockwise sequence
static const uint8_t hallStates[6][3] = {
	{1, 0, 0},
	{1, 0, 1},
	{0, 0, 1},
	{0, 1, 1},
	{0, 1, 0},
	{1, 1, 0}
};

//-----------------------------------------------------------------------------
// Initialization
//-----------------------------------------------------------------------------
void initHW(void)
{
    initSystemClockTo40Mhz();

    // Enable ports
    enablePort(COIL_PORT);
    enablePort(SENSE_PORT);
    enablePort(PWM_PORT);

    // Configure hall sensors as digital inputs
    enablePinPullup(SENSE_PORT, SENSE_A_PIN);
    enablePinPullup(SENSE_PORT, SENSE_B_PIN);
    enablePinPullup(SENSE_PORT, SENSE_C_PIN);
    selectPinDigitalInput(SENSE_PORT, SENSE_A_PIN);
    selectPinDigitalInput(SENSE_PORT, SENSE_B_PIN);
    selectPinDigitalInput(SENSE_PORT, SENSE_C_PIN);

    // Initially make coils floating (safe startup)
    selectPinDigitalInput(COIL_PORT, COIL_A_PIN);
    selectPinDigitalInput(COIL_PORT, COIL_B_PIN);
    selectPinDigitalInput(COIL_PORT, COIL_C_PIN);

    // Initialize UART 0
    initUart0();

        // Configure SW1 (PF4) as input with internal pull-up
    enablePort(SW1_PORT);
    selectPinDigitalInput(SW1_PORT, SW1_PIN);
    enablePinPullup(SW1_PORT, SW1_PIN);

}

//-----------------------------------------------------------------------------
// Apply commutation state (coils + PWM)
//-----------------------------------------------------------------------------
void applyCommutationState(uint8_t state)
{
    int8_t a = phaseDrive[state][0];
    int8_t b = phaseDrive[state][1];
    int8_t c = phaseDrive[state][2];

    // --- Coil A + PWM A ---
    if (a == +1)
    {
        selectPinPushPullOutput(COIL_PORT, COIL_A_PIN);
        setPinValue(COIL_PORT, COIL_A_PIN, 1);

        // Activate PWM A simultaneously
        selectPinPushPullOutput(PWM_PORT, PWM_A_PIN);
        setPinValue(PWM_PORT, PWM_A_PIN, 1);
    }
    else if (a == 0)
    {
        selectPinPushPullOutput(COIL_PORT, COIL_A_PIN);
        setPinValue(COIL_PORT, COIL_A_PIN, 1);

        // Optionally turn off PWM when coil is low
        selectPinPushPullOutput(PWM_PORT, PWM_A_PIN);
        setPinValue(PWM_PORT, PWM_A_PIN, 0);
    }
    else
    {
        selectPinDigitalInput(COIL_PORT, COIL_A_PIN);
        selectPinDigitalInput(PWM_PORT, PWM_A_PIN);

    }

    // --- Coil B + PWM B ---
    if (b == +1)
    {
        selectPinPushPullOutput(COIL_PORT, COIL_B_PIN);
        setPinValue(COIL_PORT, COIL_B_PIN, 1);

        selectPinPushPullOutput(PWM_PORT, PWM_B_PIN);
        setPinValue(PWM_PORT, PWM_B_PIN, 1);
    }
    else if (b == 0)
    {
        selectPinPushPullOutput(COIL_PORT, COIL_B_PIN);
        setPinValue(COIL_PORT, COIL_B_PIN, 1);

        selectPinPushPullOutput(PWM_PORT, PWM_B_PIN);
        setPinValue(PWM_PORT, PWM_B_PIN, 0);
    }
    else
    {
        selectPinDigitalInput(COIL_PORT, COIL_B_PIN);
        selectPinDigitalInput(PWM_PORT, PWM_B_PIN);
    }

    // --- Coil C + PWM C ---
    if (c == +1)
    {
        selectPinPushPullOutput(COIL_PORT, COIL_C_PIN);
        setPinValue(COIL_PORT, COIL_C_PIN, 1);

        selectPinPushPullOutput(PWM_PORT, PWM_C_PIN);
        setPinValue(PWM_PORT, PWM_C_PIN, 1);
    }
    else if (c == 0)
    {
        selectPinPushPullOutput(COIL_PORT, COIL_C_PIN);
        setPinValue(COIL_PORT, COIL_C_PIN, 1);

        selectPinPushPullOutput(PWM_PORT, PWM_C_PIN);
        setPinValue(PWM_PORT, PWM_C_PIN, 0);
    }
    else
    {
        selectPinDigitalInput(COIL_PORT, COIL_C_PIN);
        selectPinDigitalInput(PWM_PORT, PWM_C_PIN);
    }
}


//-----------------------------------------------------------------------------
// Drive Motor with no sensors (open-loop)
//-----------------------------------------------------------------------------
void testCommutationSpeed(void)
{
    putsUart0("Starting motor slip test. Press SW1 to speed up motor...\r\n");

    uint32_t delayDec = 200;        // Start fast (small delay)
    const uint32_t minDelay = 200;
    uint8_t state = 0;
    bool prevSw1 = true;          // True = not pressed (pull-up active)
    uint32_t currentDelay =  11000;

    putsUart0("Starting Delay: ");
    putintUart0(currentDelay, false);
    putsUart0("\r\n");

    while (true)
    {
        // Perform one full commutation cycle
        uint8_t step;
        for (step = 0; step < 6; step++)
        {
            applyCommutationState(state);
            state = (state + 1) % 6;
            waitMicrosecond(currentDelay);
        }

        // --- Read SW1 (PF4 active low) ---
        bool sw1 = getPinValue(PORTF, 4);

        // Detect new press (button goes from high -> low)
        if (!sw1 && prevSw1)
        {
            currentDelay -= delayDec;  // Increase delay by 100 us
            if (currentDelay < minDelay)
                currentDelay = minDelay;

            putsUart0("Delay decreased to: ");
            putintUart0(currentDelay, false);
            putsUart0(" us\r\n");

            // Debounce delay (avoid multiple triggers)
            waitMicrosecond(200000);  // 200 ms
        }

        prevSw1 = sw1;  // Save state for next loop
    }
}


//-----------------------------------------------------------------------------
// Polling till valid hall sensor state is read
//-----------------------------------------------------------------------------
void waitForValidHallState(uint8_t state)
{
    // Expected hall pattern for the *next* commutation state
    uint8_t nextState = (state + 1) % 6;
    uint8_t hallA = hallStates[nextState][0];
    uint8_t hallB = hallStates[nextState][1];
    uint8_t hallC = hallStates[nextState][2];

    // Wait until sensors read that pattern
    while (true)
    {
        bool a = getPinValue(SENSE_PORT, SENSE_A_PIN);
        bool b = getPinValue(SENSE_PORT, SENSE_B_PIN);
        bool c = getPinValue(SENSE_PORT, SENSE_C_PIN);
        //applyCommutationState(nextState); // Keep applying the next state to keep the motor energized
        if (a == hallA && b == hallB && c == hallC)
            break;
    }

    // Small settling delay
    waitMicrosecond(50);
}

//-----------------------------------------------------------------------------
// Speed up motor
//-----------------------------------------------------------------------------
uint8_t setUpMotorSpeed(void)
{
    putsUart0("Speeding up motor to operating speed...\r\n");

    uint32_t currentDelay =  7000;
    uint8_t state = 0;

    putsUart0("Delay: ");
    putintUart0(currentDelay, false);
    putsUart0("\r\n");

    uint32_t count = 0;

    while (count < 35)
    {
        // Perform one full commutation cycle
        uint8_t step;
        for (step = 0; step < 6; step++)
        {
            //applyCommutationState(state);
            state = (state + 1) % 6;
            waitMicrosecond(currentDelay);
        }
        count++;
    }
    putsUart0("Leaving setup speed function \n\r");
    return state;
}

//-----------------------------------------------------------------------------
// Drive Motor with no sensors (open-loop)
//-----------------------------------------------------------------------------
void testSensorCommutation(void)
{
    /// Start motor in open-loop to get it moving
    uint8_t state = setUpMotorSpeed();

    putsUart0("Switching to sensor-based commutation...\r\n");

    while (true)
    {
        applyCommutationState(state);
        waitForValidHallState(state);
        state = (state + 1) % 6;
        waitMicrosecond(7000);
    }
}

/**
 * main.c
 */

int main(void)
{
    initHW();

    //testCommutationSpeed();
    testSensorCommutation();

    return 0;
}
