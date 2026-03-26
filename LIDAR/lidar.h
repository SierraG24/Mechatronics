#ifndef LIDAR_H
#define LIDAR_H

#define START_FLAG          0xA5
#define SINGLE_RESPONSE     0x00
#define MULTIPLE_RESPONSE   0x01

// All of the Request Values
#define STOP                0x25
#define RESET               0x40
#define SCAN                0x20    
#define EXPRESS_SCAN        0x82    // Scan and work at the highest speed
#define FORCE_SCAN          0x21    // Scan and force data output without checking rotation speed
#define GET_SAMPLERATE      0x59
#define GET_LIDAR_CONF      0x84

// Libraries
#include "uart1.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"
#include <stdbool.h>

//typedef struct {
//    float angle;      // degrees
//    float distance;   // mm
//    uint8_t quality;  // Check scan bit
//} LidarPoint;


void sendStop(void);
void sendReset(void);
void getSampleRate(void);
void startScan(void);
bool getResponse();
void flushUart1();

//bool parseLidarPoint(LidarPoint* pt);

#endif
