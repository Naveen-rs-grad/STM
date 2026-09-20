#ifndef DHT11_H
#define DHT11_H

#include "main.h"

typedef struct
{
    uint8_t humidity;
    uint8_t temperature;
} DHT11_Data_t;

uint8_t DHT11_Read(DHT11_Data_t *data);

#endif
