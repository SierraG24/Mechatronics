#include <inttypes.h>
#include "tempSensor.h"
#include "uart0.h"

float tempToVoltage(float temp)
{
    if (temp < MIN_K_TYPE_TEMP_C || temp > MAX_K_TYPE_TEMP_C)
    {
        putsUart0("Temperature out of range for K-type thermocouple.\n\r");
        return 0.0; // Out of range
    }

    float result;

    // Measure distace from the minimum temp
    float distance = temp - MIN_K_TYPE_TEMP_C;
    uint32_t index = (uint32_t)(distance / STEP_SIZE_C);
    float fraction = (distance - (index * STEP_SIZE_C)) / STEP_SIZE_C;

    // Grab the bounds
    float lowerbound = typeK_mv[index];
    float upperbound = typeK_mv[index + 1];

    // Calculate result: voltage = Vlow + (delta V * frac)
    result = lowerbound + (upperbound - lowerbound) * fraction;

    return result;
}


float voltageToTemp(float mv)
{
    // Check for out of range
    if (mv < typeK_mv[0] || mv > typeK_mv[TABLE_SIZE - 1])
    {
        putsUart0("Voltage out of range for K-type thermocouple.\n\r");
        return 0.0; // Out of range
    }

    // Find the two points to interpolate between
    uint32_t i;
    for (i = 0; i < TABLE_SIZE - 1; i++)
    {
        // Set bounds
        float lowerbound = typeK_mv[i];
        float upperbound = typeK_mv[i + 1];

        if (mv >= lowerbound && mv <= upperbound + 1.e-6f)
        {
            float fraction = (mv - lowerbound) / (upperbound - lowerbound);
            // Calculate the temp at the index 
            float temp = MIN_K_TYPE_TEMP_C + (i * STEP_SIZE_C);
            // Calculate the temp at mV
            float result = temp + (fraction * STEP_SIZE_C);
            return result;
        }
    }

    // Return 0 if no bracket is found
    return 0;
}
