/*
Copyright © 2017 Silvair Sp. z o.o. All Rights Reserved.
 
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:
 
The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.
 
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef SENSOR_H
#define SENSOR_H

typedef union
{
    uint32_t als;
    uint8_t  pir;
    uint32_t power;
    uint16_t current;
    uint16_t voltage;
    uint32_t energy;
    uint32_t precise_energy;
} SensorValue_T;

typedef enum
{
    PRESENCE_DETECTED               = 0x004D,
    PRESENT_AMBIENT_LIGHT_LEVEL     = 0x004E,
    PRESENT_DEVICE_INPUT_POWER      = 0x0052,
    PRESENT_INPUT_CURRENT           = 0x0057,
    PRESENT_INPUT_VOLTAGE           = 0x0059,
    TOTAL_DEVICE_ENERGY_USE         = 0x006A,
    PRECISE_TOTAL_DEVICE_ENERGY_USE = 0x0072
} SensorProperty_T;

#endif
