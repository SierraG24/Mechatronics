
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "gpio.h"
#include <inttypes.h>
#include "wait.h"
#include "clock.h"
#include "uart0.h"
#include "math.h"

#define HX711_PORT PORTB
#define HX711_CLK 1
#define HX711_DATA 0

#define SW1_PIN 4
#define SW PORTF, 4
#define DEBOUNCE_DELAY_US 250000 // 250 ms

// Creating a sliding window buffer
#define WINDOW_SIZE 25
#define COLLECTION 3
long offset[WINDOW_SIZE];
long raw[WINDOW_SIZE];
long result[COLLECTION];

void slideBuffer(long *buffer, long newValue)
{
    // Shift everything left by 1
    int i;
    for (i = 1; i < WINDOW_SIZE; i++)
    {
        buffer[i - 1] = buffer[i];
    }

    // Put new value at the end
    buffer[WINDOW_SIZE - 1] = newValue;
}
void putSignedIntUart0(int32_t value)
{
    if (value < 0)
    {
        putcUart0('-');
        putintUart0((uint32_t)(-value), false);
    }
    else
    {
        putintUart0((uint32_t)value, false);
    }
}

long calculateAvg(long *sample)
{
    uint64_t sum = 0;
    int i;
    for ( i = 0; i < WINDOW_SIZE; i++)
        sum += sample[i];

    return (long)(sum / WINDOW_SIZE);
}

//-----------------------------------------------------------------------------
// Pin layout
//-----------------------------------------------------------------------------
//
// PF4 - SW0
// PB1 - CLK
// PB0 - Data

void initHx711()
{
    enablePort(HX711_PORT); // enable port B

    selectPinPushPullOutput(HX711_PORT, HX711_CLK); // set output
    setPinValue(HX711_PORT, HX711_CLK, 0);          // start low

    selectPinDigitalInput(HX711_PORT, HX711_DATA); // set digital input
    waitMicrosecond(10000);
}

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
}

int32_t readHx711(void)
{

    long value = 0;
    //setPinValue(HX711_PORT, HX711_DATA, 1);
    setPinValue(HX711_PORT, HX711_CLK, 0);

    while (getPinValue(HX711_PORT, HX711_DATA))
    {
    }

    int i = 0;

    // 2) Clock out 24 bits (MSB first)
    for (i = 0; i < 24; i++)
    {
        // clock high (< 50 us)
        setPinValue(HX711_PORT, HX711_CLK, 1);
        waitMicrosecond(5); // 1 us is comfortably < 50 us

        // read DATA while clock is high
        value = value << 1;

        if (getPinValue(HX711_PORT, HX711_DATA))
            value |= 1;

        // clock low (> 200 ns)
        setPinValue(HX711_PORT, HX711_CLK, 0);
        waitMicrosecond(5); // >> 200 ns, so it's fine
    }

    // 3) 25th clock pulse: select Channel A, gain 128
    setPinValue(HX711_PORT, HX711_CLK, 1);
    waitMicrosecond(5);

    // 4) Sign-extend 24-bit twos complement to 32 bits
    if (value & 0x800000)    // if sign bit (bit 23) is set
        value |= 0xFF000000; // fill upper bits with 1s

    setPinValue(HX711_PORT, HX711_CLK, 0);
    waitMicrosecond(15);

    return (int32_t)value;
}

void initHW()
{
    initSystemClockTo40Mhz();
    initHx711();
    initUart0();
    initPB();

    // Initialize sliding window buffer  for the raw and offset to 0
    int i;
    for (i = 0; i < WINDOW_SIZE; i++)
        raw[i] = 0;
    for (i = 0; i < WINDOW_SIZE; i++)
        offset[i] = 0;
    for (i = 0; i < COLLECTION; i++)
        result[i] = 0;
}
// Polling function to derive adc value to force
// Maybe do offset - raw
long calculateWeight()
{
    uint32_t j, i;
    long tempOffset;
    long tempRaw;
    long avgOffset[COLLECTION];
    long avgRaw[COLLECTION];

    //----------------------------------------------------------------------
    // 1. WAIT FOR BUTTON PRESS TO CAPTURE OFFSET BASELINE
    //----------------------------------------------------------------------
    putsUart0(">>> Press button to TAR (zero)\r\n");
    while (getPinValue(PORTF, 4))
        ; // wait until button PRESSED
    while (!getPinValue(PORTF, 4))
        ; // wait until RELEASED

    // Collect offset baseline
    for (j = 0; j < COLLECTION; j++)
    {
        for (i = 0; i < WINDOW_SIZE; i++)
        {
            tempOffset = (long)readHx711();
            slideBuffer(offset, tempOffset);
            waitMicrosecond(20);
        }

        avgOffset[j] = calculateAvg(offset);
        //putsUart0("Offset Avg: ");
        //putSignedIntUart0(avgOffset[j]);
        //putsUart0("\r\n");
    }

    //----------------------------------------------------------------------
    // 2. WAIT FOR SECOND BUTTON PRESS TO CAPTURE RAW
    //----------------------------------------------------------------------
    putsUart0(">>> Place weight & press button\r\n");
    while (getPinValue(PORTF, 4))
        ; // wait until press
    while (!getPinValue(PORTF, 4))
        ; // release

    // Collect raw readings
    for (j = 0; j < COLLECTION; j++)
    {
        for (i = 0; i < WINDOW_SIZE; i++)
        {
            tempRaw = (long)readHx711();
            slideBuffer(raw, tempRaw);
            waitMicrosecond(20);
        }

        avgRaw[j] = calculateAvg(raw);
        //putsUart0("Raw Avg: ");
        //putSignedIntUart0(avgRaw[j]);
        //putsUart0("\r\n");
    }

    //----------------------------------------------------------------------
    // 3. Compute difference (raw - offset) Might need to flip
    //----------------------------------------------------------------------
    for (i = 0; i < COLLECTION; i++)
    {
        result[i] = avgOffset[i] - avgRaw[i];
        //putsUart0("Result: ");
        //putSignedIntUart0(result[i]);
        //putsUart0("\r\n");
    }

    //----------------------------------------------------------------------
    // 4. Calculate average result
    //----------------------------------------------------------------------
    long avgResult = 0;
    for (i = 0; i < COLLECTION; i++)
        avgResult += result[i];

    avgResult /= COLLECTION;

    putsUart0("Final ADC Value: ");
    putSignedIntUart0(avgResult);
    putsUart0("\r\n\n");

    return avgResult;
}

float convertToMass(int32_t adcValue)
{
    float scale_g = 0.0262f; // Derive this value
    float k = -6.95f;

    float result = (scale_g * (float)abs(adcValue)) + k;
    return result;
}

float convertToForce(float mass_g)
{
    float mass_kg = mass_g / 1000.0f;  // Covert g to kg
    float result = mass_kg * 9.80665f; // F = m * g
    return result;
}

int main(void)
{
    initHW();
    // putsUart0("HX711 test\r\n");

    while (true)
    {
        putsUart0("Starting Measurement\n\r");
        long adcValue = calculateWeight();
        float mass_g = convertToMass(adcValue);
        float force = convertToForce(mass_g);
        putsUart0("Mass_g: ");
        putfloatUart0(mass_g, 2);
        putsUart0("\tForce: ");
        putfloatUart0(force, 2);
        putsUart0("\n\n\r");
        waitMicrosecond(1000);
    }
}
