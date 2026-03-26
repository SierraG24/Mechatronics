#ifndef TEMPSENSOR_H_
#define TEMPSENSOR_H_

// Cold based junction referenced at 0C
// Range from -260 to 320C
// Cold based junction referenced at 0C
// Range from -260 to 320C
#define MIN_K_TYPE_TEMP_C -260
#define MAX_K_TYPE_TEMP_C 320
#define STEP_SIZE_C (10.0f)
#define TABLE_SIZE (sizeof(typeK_mv) / sizeof(typeK_mv[0]))

// Each index represents a 10 degree C step starting from -260C to 310C in mV
// Each index is a 10C step 
// Each index represents a 10C step starting from -260C
static const float typeK_mv[] =
{
    // -260C to -210C
    -6.441, -6.404, -6.344, -6.262, -6.158, -6.035,

    // -200C to -150C
    -5.891, -5.730, -5.550, -5.354, -5.141, -4.913,

    // -140C to -90C
    -4.669, -4.411, -4.138, -3.852, -3.554, -3.243,

    // -80C to -30C
    -2.920, -2.587, -2.243, -1.889, -1.527, -1.156,

    // -20C to +30C
    -0.778, -0.392, 0.000, 0.397, 0.798, 1.203,

    // +40C to +90C
    1.612, 2.023, 2.436, 2.851, 3.267, 3.682,

    // +100C to +150C
    4.096, 4.509, 4.920, 5.328, 5.735, 6.138,

    // +160C to +210C
    6.540, 6.941, 7.340, 7.739, 8.138, 8.539,

    // +220C to +270C
    8.940, 9.343, 9.747, 10.153, 10.561, 10.971,

    // +280C to +320C
    11.382, 11.795, 12.209, 12.624, 13.040
};



//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

float tempToVoltage(float temp);
float voltageToTemp(float mv);

#endif
