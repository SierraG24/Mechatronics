/*
 * CTI.c
 *
 *  Created on: Mar 25, 2025
 *      Author: jrsie
 */

#include "CTI.h"
#include <stdlib.h>
#include <string.h>

void getsUart0(USER_DATA *data)
{
    uint8_t index = 0;
    char symbol;
    uint8_t flag = 0;

    while (flag != 1)
    {
        symbol = getcUart0();
        if (symbol == 8 || symbol == 127) // backspace
        {
            if (index > 0)
            {
                index--;
            }
        }
        else if (symbol == 10 || symbol == 13) // return
        {
            data->buffer[index] = '\0';
            flag = 1;
        }
        else if (symbol >= 32) // printable
        {
            data->buffer[index] = symbol;
            index++;
            if (index == MAX_CHARS)
            {
                flag = 1;
            }
        }
        data->buffer[index] = '\0';
    }
}

void parseFields(USER_DATA *data)
{
    uint8_t i, fieldNum = 0;
    bool newField = true;
    char symbol;

    for (i = 0; data->buffer[i] != 0; i++)
    {
        symbol = data->buffer[i];

        if((symbol >= 'a' && symbol <= 'z') || (symbol >= 'A' && symbol <= 'Z'))
        {
            if (newField && fieldNum < MAX_FIELDS)
            {
                data->fieldType[fieldNum] = 'a';
                data->fieldPosition[fieldNum] = i;
                fieldNum++;
                newField = false;
            }
        }
        // allow numbers, minus sign, and decimal point as part of numeric field
        else if ((symbol >= '0' && symbol <= '9') || symbol == '-' || symbol == '.')
        {
            if (newField && fieldNum < MAX_FIELDS)
            {
                data->fieldType[fieldNum] = 'n'; // treat as numeric (could be float)
                data->fieldPosition[fieldNum] = i;
                fieldNum++;
                newField = false;
            }
        }
        else
        {
            newField = true;
            data->buffer[i] = '\0';
        }
    }
    data->fieldCount =  fieldNum;
}

char* getFieldString(USER_DATA *data, uint8_t fieldNumber)
{
    if(fieldNumber >= MAX_FIELDS || fieldNumber > data->fieldCount)
    {
        return "\0";
    }
    else
    {
        return &data->buffer[data->fieldPosition[fieldNumber]];
    }
}

int32_t getFieldInteger(USER_DATA *data, uint8_t fieldNumber)
{
   if(fieldNumber >= MAX_FIELDS || fieldNumber > data->fieldCount
      || data->fieldType[fieldNumber] != 'n')
   {
       return 0;
   }
   else
   {
       char *str = &data->buffer[data->fieldPosition[fieldNumber]];
       return atoi(str);
   }
}

//function to extract float directly
float getFieldFloat(USER_DATA *data, uint8_t fieldNumber)
{
   if(fieldNumber >= MAX_FIELDS || fieldNumber > data->fieldCount
      || data->fieldType[fieldNumber] != 'n')
   {
       return 0.0f;
   }
   else
   {
       char *str = &data->buffer[data->fieldPosition[fieldNumber]];
       return atof(str); // handles negative numbers and decimals
   }
}

bool isCommand(USER_DATA *data, const char strCommand[], uint8_t minArguments)
{
    if ((data->fieldCount - 1) < minArguments)
    {
        return false;
    }

    char *cmd = &data->buffer[data->fieldPosition[0]];
    return (strcmp(cmd, strCommand) == 0);
}
