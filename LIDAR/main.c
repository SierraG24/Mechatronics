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
#include "uart1.h"
#include "lidar.h"
#include "math.h"

#define PWM_MASK 8 // PF3
#define PWM_PIN 3
#define SW1_PIN 4
#define SW PORTF, 4
#define MAX_POINTS 1000

//-----------------------------------------------------------------------------
// Pin layout
//---------------------------------------------------------------------------
// PF3 - PWM
// PF4 - SW1
// PC4 - Uart 1 Rx  Yellow Wire
// PC5 - Uart 1 Tx  Green Wire

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
static float distance[MAX_POINTS];
static float angle[MAX_POINTS];

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
// Initize the PWM to 25% percent duty cycle
// Period of 20Khz
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
    PWM1_3_LOAD_R = 1999;

    // Start disabled (0% duty)
    PWM1_3_CMPB_R = 0;

    PWM1_3_CTL_R = PWM_1_CTL_ENABLE;
    PWM1_ENABLE_R = PWM_ENABLE_PWM7EN;
}

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
}

void initHW()
{
    initSystemClockTo40Mhz();
    initUart0();
    initUart1();
    initPWM();
    initPB();
}

int main(void)
{
    initHW();
    putsUart0("\nLIDAR LAB Initialized\n\r");

    uint16_t tempAngle;
    uint16_t tempDist;
    uint8_t index;
    bool responseCaptured;
    while (true)
    {
        // Active LOW button
        if (!getPinValue(PORTF, 4))
        {
            putsUart0("\n\nScanning\n\r");

            flushUart1();
            waitMicrosecond(20000);
            startScan();
            responseCaptured = getResponse();

            if (!responseCaptured)
            {
                putsUart0("No valid response from LIDAR\n\r");
                sendStop();
            }
            else
            {

                putsUart0("Valid response from LIDAR\n\r");

                // Wait for first point of a new scan
                putsUart0("\n\rWaiting for new scan");
                bool newScanDetected = false;
                uint8_t buffer[5];

                while (!newScanDetected)
                {
                    uint8_t index = 0;
                    while (index < 5)
                    {
                        if (kbhitUart1())
                        {
                            buffer[index] = getcUart1();
                            index++;
                        }
                    }
                    // Check the new scan bit (LSB of quality byte)
                    if (buffer[0] & 0x01)
                    {
                        newScanDetected = true;
                    }
                }

                // New scan detected, read MAX_POINTS points
                putsUart0("\n\rNew Scan detected\n\r");
                uint16_t i;
                tempAngle = 0;
                tempDist = 0;
                for (i = 0; i < MAX_POINTS; i++)
                {

                        uint8_t buffer[5];
                        index = 0;
                        while (index < 5)
                        {
                            if (kbhitUart1())
                            {
                                buffer[index] = getcUart1();

                                // print each byte
                                // puthex8Uart0(bytes[index]);
                                // putcUart0(' ');
                                index++;
                            }
                        }

                        // Extract distance
                        tempDist = ((uint16_t)buffer[4] << 8) | (uint16_t)buffer[3];
                        distance[i] = (float)tempDist / 4.0f;

                        // Extract angle
                        uint16_t raw = ((uint16_t)buffer[2] << 8) | (uint16_t)buffer[1];
                        tempAngle = raw >> 1; // drop the checkbit
                        angle[i] = (float)tempAngle / 64.0f;



                }

                sendStop();
                // putsUart0("Stopped Scanning \n\n\r");
                //  Print all captured points
                for (i = 0; i < MAX_POINTS; i++)
                {
                    if (i == 0)
                    {
                        putsUart0("\n\n\r");
                        putfloatUart0(distance[i] / 1000.0f, 2);
                        putsUart0(",");
                        putfloatUart0(angle[i], 2);
                        putsUart0(";\n\r");
                    }
                    else
                    {
                        if (distance[i] != 0)
                        {
                            putfloatUart0(distance[i] / 1000.0f, 2);
                            putsUart0(",");
                            putfloatUart0(angle[i], 2);
                            putsUart0(";\n\r");
                        }
                    }
                }

                float area = 0.0f;
                const float DEG_TO_RAD = 3.14159265f / 180.0f;

                for (i = 0; i < MAX_POINTS; i++)
                {
                    float rad1 = angle[i] * DEG_TO_RAD;
                    float rad2 = angle[(i + 1) % MAX_POINTS] * DEG_TO_RAD;

                    float deltaDist = distance[i] * distance[(i + 1) % MAX_POINTS];
                    float deltaAngle = sinf(rad2 - rad1);

                    /* Accumulate the sector contribution (1/2 * r_i * r_{i+1} * sin(delta theta)) */
                    area += 0.5f * deltaDist * deltaAngle;
                }

                /* Area can be negative depending on vertex ordering; take absolute value */
                area = fabsf(area);

                /* Convert mm^2 to m^2 (1 m^2 = 1e6 mm^2) */
                putsUart0("\n\rArea: ");
                putfloatUart0(area / 1000000.0f, 2);
                putsUart0(" m^2\t");
                putfloatUart0((area / 1000000.0f) * 10.7639, 2);
                putsUart0(" ft^2\t");
                putfloatUart0(((area / 1000000.0f) * 10.7639 * 144), 2);
                putsUart0(" in^2\n\r");
            }

            waitMicrosecond(100000);
        }
    }
}
