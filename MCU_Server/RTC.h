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

#ifndef RTC_H_
#define RTC_H_

#include <stddef.h>
#include <stdint.h>

#define RTC_WITH_BATTERY_ATTACHED 0x03
#define RTC_WITHOUT_BATTERY_ATTACHED 0x01
#define RTC_NOT_ATTACHED 0x00

struct __attribute__((packed)) TimeDate
{
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  seconds;
    uint16_t milliseconds;
};

typedef void (*SendTimeSourceGetRespCallback)(uint8_t, TimeDate *);
typedef void (*SendTimeSourceSetRespCallback)(uint8_t);

bool RTC_Init(SendTimeSourceGetRespCallback get_resp_callback, SendTimeSourceSetRespCallback set_resp_callback);

void RTC_SetTime(TimeDate *time);

void RTC_GetTime(void);

bool RTC_IsBatteryDetected(void);

void SetTimeServerInstanceIdx(uint8_t instance_index);

uint8_t GetTimeServerInstanceIdx(void);

void LoopRTC(void);

#endif    // RTC_H_
