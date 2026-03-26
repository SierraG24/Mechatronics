#ifndef ADS1115_H_
#define ADS1115_H_

// ADS1115 Defines
// ADS1115 I2C address and registers
#define ADS1115_GND_ADDRESS 0x48
#define ADS1115_VDD_ADDRESS 0x49
#define ADS1115_SDA_ADDRESS 0x4A
#define ADS1115_SCL_ADDRESS 0x4B

// ADS1115 PGA settings
#define ADS1115_PGA_6_144V 0b000 // ±6.144V
#define ADS1115_PGA_4_096V 0b001 // ±4.096V
#define ADS1115_PGA_2_048V 0b010 // ±2.048V (default)
#define ADS1115_PGA_1_024V 0b011 // ±1.024V
#define ADS1115_PGA_0_512V 0b100 // ±0.512V
#define ADS1115_PGA_0_256V 0b101 // ± 0.256

// Setting Channel MUX
#define ADS1115_MUX_0  0b000  // AINp = AIN0 & AINn = AIN1 default,
#define ADS1115_MUX_1  0b001  // AINp = AIN0 & AINn = AIN3
#define ADS1115_MUX_2  0b010  // AINp = AIN1 & AINn = AIN3
#define ADS1115_MUX_3  0b011  // AINp = AIN2 & AINn = AIN3
#define ADS1115_MUX_4  0b100  // AINp = AIN0 & AINn = GND
#define ADS1115_MUX_5  0b101  // AINp = AIN1 & AINn = GND
#define ADS1115_MUX_6  0b110  // AINp = AIN2 & AINn = GND
#define ADS1115_MUX_7  0b111  // AINp = AIN3 & AINn = GND

// Mode
#define ADS1115_MODE_CONTINUOUS 0
#define ADS1115_MODE_SINGLESHOT 1

// ADS1115 Data Rate
#define ADS1115_DR_8SPS     0b000
#define ADS1115_DR_16SPS    0b001
#define ADS1115_DR_32SPS    0b010
#define ADS1115_DR_64SPS    0b011
#define ADS1115_DR_128SPS   0b100
#define ADS1115_DR_250SPS   0b101
#define ADS1115_DR_475SPS   0b110
#define ADS1115_DR_860SPS   0b111

// ADS1115 Compare Mode
#define ADS1115_COMP_MODE_TRADITIONAL 0
#define ADS1115_COMP_MODE_WINDOW      1

// ADS1115 Compare Polarity
#define ADS1115_COMP_POL_ACTIVE_LOW 0
#define ADS1115_COMP_POL_ACTIVE_HIGH 1

// ADS1115 Latching Comparator
#define ADS1115_COMP_LAT_NONLATCHING 0
#define ADS1115_COMP_LAT_LATCHING    1

// ADS1115 Comparator Queue
#define ADS1115_COMP_QUE_1    0b00  // Assert after 1 conversion
#define ADS1115_COMP_QUE_2    0b01  // Assert after 2 conversions
#define ADS1115_COMP_QUE_4    0b10  // Assert after 4 conversions
#define ADS1115_COMP_QUE_NONE 0b11  // Disable comparator and set ALERT/RDY pin to high impedance

// ADS1115 Registers
#define ADS1115_REG_CONVERSION  0x00
#define ADS1115_REG_CONFIG      0x01
#define ADS1115_REG_LO_THRESH   0x02
#define ADS1115_REG_HI_THRESH   0x03


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
void configADS1115(uint8_t address, uint8_t mux, uint8_t pga,
                   uint8_t mode, uint8_t dataRate,
                   uint8_t compMode, uint8_t compPol,
                   uint8_t compLat, uint8_t compQue);

int16_t readADS1115(uint8_t address);

float adcToVoltage(int16_t raw, uint8_t pga);

#endif
