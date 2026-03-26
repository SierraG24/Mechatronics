#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "gpio.h"
#include "wait.h"
#include "i2c0.h"
#include "uart0.h"
#include "clock.h"
#include "ads1115.h"
#include "tempSensor.h"

//-----------------------------------------
// Global variables
//----------------------------------------
int16_t ch0, ch1;
float Tcj, Ttc;
float Vcj, Vtc, Vf;
bool flag;

void initHw()
{
    initSystemClockTo40Mhz();
    initUart0();
    initI2c0();
}

void readTMP36()
{
    // TMP36 on AIN2
    configADS1115(ADS1115_GND_ADDRESS, ADS1115_MUX_6,
                  ADS1115_PGA_2_048V, ADS1115_MODE_SINGLESHOT,
                  ADS1115_DR_8SPS, ADS1115_COMP_MODE_TRADITIONAL,
                  ADS1115_COMP_POL_ACTIVE_LOW, ADS1115_COMP_LAT_NONLATCHING,
                  ADS1115_COMP_QUE_NONE);
    // pollI2c0Address(ADS1115_GND_ADDRESS);
    waitMicrosecond(130000);
    ch0 = readADS1115(ADS1115_GND_ADDRESS);
    Tcj = adcToVoltage(ch0, ADS1115_PGA_2_048V);
    Tcj = ((float)Tcj - 0.5f) / 0.01f;
    Vcj = tempToVoltage(Tcj);
}

void readTypeK()
{
    // Thermocouple on AIN0-AIN1 differential
    configADS1115(ADS1115_GND_ADDRESS, ADS1115_MUX_0,
                  ADS1115_PGA_0_256V, ADS1115_MODE_SINGLESHOT,
                  ADS1115_DR_8SPS, ADS1115_COMP_MODE_TRADITIONAL,
                  ADS1115_COMP_POL_ACTIVE_LOW, ADS1115_COMP_LAT_NONLATCHING,
                  ADS1115_COMP_QUE_NONE);
    // pollI2c0Address(ADS1115_GND_ADDRESS);
    waitMicrosecond(130000);
    ch1 = readADS1115(ADS1115_GND_ADDRESS);
    if (ch1 < 0)
        Vtc = adcToVoltage(ch1, ADS1115_PGA_0_256V) * 1000;
    else
        Vtc = adcToVoltage(ch1, ADS1115_PGA_0_256V) * 1500;
    Vf = Vtc + Vcj;
    Ttc = voltageToTemp(Vf);
}

bool isThermocoupleOC()
{
    // Set differential channels
    configADS1115(ADS1115_GND_ADDRESS, ADS1115_MUX_0,
                  ADS1115_PGA_0_256V, ADS1115_MODE_SINGLESHOT,
                  ADS1115_DR_8SPS, ADS1115_COMP_MODE_TRADITIONAL,
                  ADS1115_COMP_POL_ACTIVE_LOW, ADS1115_COMP_LAT_NONLATCHING,
                  ADS1115_COMP_QUE_NONE);

    // pollI2c0Address(ADS1115_GND_ADDRESS);
    waitMicrosecond(130000);
    int16_t data = readADS1115(ADS1115_GND_ADDRESS);
    float voltage = adcToVoltage(data, ADS1115_PGA_0_256V);

    // The pins if open circuit, will float near vref
    if (fabsf(voltage) > 0.20f)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void printInfo()
{
    // Print ADC values
    putsUart0("TMP36 ADC: ");
    putintUart0(ch0, false);
    putsUart0("\tTypeK ADC: ");
    putintUart0((uint32_t) ch1, false);
    putsUart0("\n\r");

    // Print voltages
    putsUart0("Vcj: "); //cold junction equivalent thermocouple voltage in mV
    putfloatUart0(Vcj,3);
    putsUart0("\tVtc: ");// thermocouple voltage from a0-a1 difference
    putfloatUart0(Vtc,3);
    putsUart0("\tVf: ");
    putfloatUart0(Vf,3);
    putsUart0("\n\r");

    // Print temperatures
    putsUart0("Tcj: "); // Cold junction temp tmp36
    putfloatUart0(Tcj, 2);
    putsUart0("\tTtc: "); // Thermocouple temp
    putfloatUart0(Ttc, 2);
    putsUart0("\n\n\r");
}

int main(void)
{
    initHw();

    // putsUart0("Scanning I2C bus...\r\n");
    //    uint8_t addr;
    //    for (addr = 0x48; addr <= 0x4B; addr++)
    //    {
    //        if (pollI2c0Address(addr))
    //        {
    //            putsUart0("Found device at address 0x");
    //            putintUart0(addr, true);
    //            putsUart0("\r\n");
    //        }
    //    }

    while (1)
    {
        readTMP36();
        readTypeK();
        printInfo();
        waitMicrosecond(100000);
                flag = isThermocoupleOC();

                if (flag)
                {
                    putsUart0("Thermocouple disconnected\n");
                }
                else
                {
                    putsUart0("Thermocouple connected.\r\n");
                }

        waitMicrosecond(150000);
    }
}
