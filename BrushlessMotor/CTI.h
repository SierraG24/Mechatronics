/*
 * CTI.h
 *
 *  Created on: Mar 25, 2025
 *      Author: jrsie
 */

#ifndef CTI_H_
#define CTI_H_

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"

#define MAX_CHARS 80
#define MAX_FIELDS 5

typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

//Function Prototypes
void getsUart0(USER_DATA *data);
void parseFields(USER_DATA *data);
char* getFieldString(USER_DATA *data, uint8_t fieldNumber);
int32_t getFieldInteger(USER_DATA *data, uint8_t fieldNumber);
bool isCommand(USER_DATA *data, const char strCommand[], uint8_t minArguments);

#endif /* CTI_H_ */
