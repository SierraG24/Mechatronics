#include <i2c0.h>
#include "ads1115.h"

void configADS1115(uint8_t address, uint8_t mux, uint8_t pga,
                   uint8_t mode, uint8_t dataRate,
                   uint8_t compMode, uint8_t compPol,
                   uint8_t compLat, uint8_t compQue)
{
    uint16_t config = 0;

    // Bit layout:
    // 15      14-12     11-9     8      7-5      4      3      2      1-0
    // OS |    MUX   |   PGA   | MODE |   DR   | COMP_MODE | POL | LAT | QUE

    config |= (1 << 15);                 // OS bit: Start single conversion
    config |= (mux & 0x07) << 12;        // MUX selection
    config |= (pga & 0x07) << 9;         // Gain
    config |= (mode & 0x01) << 8;        // Mode
    config |= (dataRate & 0x07) << 5;    // Data rate
    config |= (compMode & 0x01) << 4;    // Comparator mode
    config |= (compPol & 0x01) << 3;     // Comparator polarity
    config |= (compLat & 0x01) << 2;     // Latching comparator
    config |= (compQue & 0x03);          // Comparator queue (2 bits)

    // Split into MSB and LSB for I²C transfer
    uint8_t data[2];
    data[0] = (config >> 8) & 0xFF;      // MSB
    data[1] = config & 0xFF;             // LSB

    // Write to the ADS1115 configuration register 
    writeI2c0Registers(address, ADS1115_REG_CONFIG, data, 2);
}

int16_t readADS1115(uint8_t address)
{
    uint8_t data[2];

    // Read conversion result (2 bytes)
    readI2c0Registers(address, ADS1115_REG_CONVERSION, data, 2);

    // Combine MSB + LSB
    return (int16_t)((data[0] << 8) | data[1]);
}

float adcToVoltage(int16_t raw, uint8_t pga)
{
    float fullScale = 0.0f;

    // Determine full-scale voltage based on PGA setting
    switch (pga)
    {
        case ADS1115_PGA_6_144V: fullScale = 6.144f; break;
        case ADS1115_PGA_4_096V: fullScale = 4.096f; break;
        case ADS1115_PGA_2_048V: fullScale = 2.048f; break;
        case ADS1115_PGA_1_024V: fullScale = 1.024f; break;
        case ADS1115_PGA_0_512V: fullScale = 0.512f; break;
        case ADS1115_PGA_0_256V: fullScale = 0.256f; break;
        default: fullScale = 2.048f; break; // default ±2.048V
    }

    // ADS1115 is a 16-bit ADC -32768 to 32767
    return (raw * fullScale) / 32768.0f;
}
