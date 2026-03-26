#include "lidar.h"
#include "wait.h"

//----------------------------------------------
// STOP (0x25)
//----------------------------------------------
void sendStop()
{
    uint8_t packet[2] = {START_FLAG, STOP};
    putBytesUart1(packet, 2);
    PWM1_3_CMPB_R = 0; // stop motor
    waitMicrosecond(200000);
}

//----------------------------------------------
// RESET (0x40)
//----------------------------------------------
void sendReset()
{
    uint8_t packet[2] = {START_FLAG, RESET};
    putBytesUart1(packet, 2);
}

//----------------------------------------------
// GET_SAMPLERATE (0x59)
//----------------------------------------------
void getSampleRate()
{
    uint8_t packet[2] = {START_FLAG, GET_SAMPLERATE};
    putBytesUart1(packet, 2);
}

//----------------------------------------------
// START SCAN (0x20)
//----------------------------------------------
void startScan()
{
    PWM1_3_CMPB_R = 1200; // turn motor on
    waitMicrosecond(200000); // Let the motor speed up
    uint8_t packet[2] = {START_FLAG, SCAN};
    putBytesUart1(packet, 2); // send RAW bytes
}

//----------------------------------------------
// Get Response Bit
//----------------------------------------------
bool getResponse()
{
    uint8_t bytes[7] = {0};
    uint8_t index = 0;

    //putsUart0("Response Bytes: ");

    // Read EXACTLY 7 bytes from Lidar
    while (index < 7)
    {
        if (kbhitUart1())
        {
            bytes[index] = getcUart1();

            // print each byte
//            puthex8Uart0(bytes[index]);
//            putcUart0(' ');

            index++;
        }
    }

    //putsUart0("\n\r");

    // Expected 7-byte response
    const uint8_t expected[7] = {0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};

    // Compare
    uint8_t i;
    for (i = 0; i < 7; i++)
    {
        if (bytes[i] != expected[i])
            return false;
    }

    return true;
}

void flushUart1()
{
    while (kbhitUart1())
            getcUart1();   // throw away garbage bytes
}
