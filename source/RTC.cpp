/*
Copyright © 2021 Silvair Sp. z o.o. All Rights Reserved.

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

#include "RTC.h"

#include "Log.h"
#include "MCU_Health.h"
#include "MeshGenericBattery.h"
#include "TAILocalTimeConverter.h"
#include "Timestamp.h"
#include "UARTProtocol.h"
#include "Utils.h"

#define BATTERY_MEASUREMENT_PERIOD_MS 60000
#define BATTERY_CURVE_STEP_PERCENT 10
#define VOLTAGE_DIVIDER_COEFFICIENT 2
#define ANALOG_MAX_READOUT 1023
#define ANALOG_REFERENCE_VOLTAGE_MV 3300


#define PCF8523_CURRENT_CONSUMPTION_NA 1200
#define CR1220_BATTER_CAPACITANCE_MAH 37
#define BATTERY_DISCHARGE_TIME_PER_PERCENT_IN_MINUTES ((((CR1220_BATTER_CAPACITANCE_MAH * 1000000) / PCF8523_CURRENT_CONSUMPTION_NA) * 60) / 100)
#define BATTERY_LEVEL_LOW_PERCENT 30
#define BATTERY_LEVEL_CRITICAL_LOW_PERCENT 10
#define BATTERY_NOT_DETECTED_THRESHOLD_PERCENT 0

#define HEALTH_FAULT_ID_BATTERY_LOW_WARNING 0x01
#define HEALTH_FAULT_ID_BATTERY_LOW_ERROR 0x02
#define HEALTH_FAULT_ID_RTC_ERROR 0xA1

volatile static bool                 ReceivedTimeGet = false;
static SendTimeSourceGetRespCallback TimeSourceGetRespCallback;
static SendTimeSourceSetRespCallback TimeSourceSetRespCallback;
static uint8_t                       LastBatteryLevelPercent    = 0;
static bool                          IsBatteryDetected          = false;
static bool                          IsBatteryLevelEverMeasured = false;
static bool                          IsTimeValid                = false;

static const uint16_t cr1220_battery_curve_mv[] = {
    0,    /* 0 % of battery capacity */
    2600, /* 10 % of battery capacity */
    2750, /* 20 % of battery capacity */
    2810, /* 30 % of battery capacity */
    2860, /* 40 % of battery capacity */
    2900, /* 50 % of battery capacity */
    2900, /* 60 % of battery capacity */
    2900, /* 70 % of battery capacity */
    2900, /* 80 % of battery capacity */
    2900, /* 90 % of battery capacity */
    2900, /* 100 % of battery capacity */
};

static uint8_t TimeServerInstanceIdx = INSTANCE_INDEX_UNKNOWN;

static struct
{
    uint32_t        end_time;
    struct TimeDate set_time;
    bool            set_time_valid;
} TimeSetParams;

static void OnSecondElapsed(void);
static void MeasureBatteryLevel(void);
static void UpdateBatteryStatus(void);
static void UpdateHealthFaultStatus(void);
static void SendGetRespTimeUnknown(void);
static void CheckRtcHasValidTime(void);

bool RTC_Init(SendTimeSourceGetRespCallback get_resp_callback, SendTimeSourceSetRespCallback set_resp_callback)
{
    if (PCF8523_Drv_IsInitialized())
    {
        return true;
    }

    if (!Pcf8523_Drv_Init())
    {
        return false;
    }

    PCF8523_Drv_RtcStart();
    CheckRtcHasValidTime();
    PCF8523_Drv_ConfigureIntEverySecond();
    PCF8523_Drv_ConfigureBatterySwitchOver();
    PCF8523_Drv_ConfigureInternalCapacitors();
    TimeSourceGetRespCallback = get_resp_callback;
    TimeSourceSetRespCallback = set_resp_callback;
    pinMode(PIN_RTC_INT1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_RTC_INT1), OnSecondElapsed, FALLING);

    return PCF8523_Drv_IsInitialized();
}

void RTC_SetTime(TimeDate *p_time)
{
    if ((TimeServerInstanceIdx == INSTANCE_INDEX_UNKNOWN) || (PCF8523_Drv_IsInitialized() == false))
    {
        return;
    }

    if (p_time->milliseconds != 0)
    {
        TimeSetParams.end_time = Timestamp_GetDelayed(Timestamp_GetCurrent(), (1000 - p_time->milliseconds));

        struct LocalTime local_time;
        local_time.year    = p_time->year;
        local_time.month   = (enum Month)(p_time->month + 1);
        local_time.day     = p_time->day;
        local_time.hour    = p_time->hour;
        local_time.minutes = p_time->minute;
        local_time.seconds = p_time->seconds;

        uint64_t time = TAILocalTimeConverter_LocalTimeToTAI(&local_time, 0, 0);
        time++;
        local_time = TAILocalTimeConverter_TAIToLocalTime(time, 0, 0);

        TimeSetParams.set_time.year         = local_time.year;
        TimeSetParams.set_time.month        = local_time.month - 1;
        TimeSetParams.set_time.day          = local_time.day;
        TimeSetParams.set_time.hour         = local_time.hour;
        TimeSetParams.set_time.minute       = local_time.minutes;
        TimeSetParams.set_time.seconds      = local_time.seconds;
        TimeSetParams.set_time.milliseconds = 0;

        TimeSetParams.set_time_valid = true;
    }
    else
    {
        IsTimeValid = true;
        PCF8523_Drv_SetTime(p_time);
        TimeSourceSetRespCallback(TimeServerInstanceIdx);
    }
}

void RTC_GetTime(void)
{
    if ((TimeServerInstanceIdx == INSTANCE_INDEX_UNKNOWN) || (PCF8523_Drv_IsInitialized() == false))
    {
        return;
    }

    if (IsTimeValid == false)
    {
        SendGetRespTimeUnknown();
        return;
    }

    ReceivedTimeGet = true;
}

bool RTC_IsBatteryDetected(void)
{
    MeasureBatteryLevel();

    if (IsBatteryDetected)
    {
        LOG_INFO("Battery detected");
    }
    else
    {
        LOG_INFO("Battery not detected");
    }

    return IsBatteryDetected;
}

void SetTimeServerInstanceIdx(uint8_t instance_index)
{
    TimeServerInstanceIdx = instance_index;
}

uint8_t GetTimeServerInstanceIdx(void)
{
    return TimeServerInstanceIdx;
}

void LoopRTC(void)
{
    if ((TimeServerInstanceIdx == INSTANCE_INDEX_UNKNOWN) || (PCF8523_Drv_IsInitialized() == false))
    {
        return;
    }

    MeasureBatteryLevel();

    if (TimeSetParams.set_time_valid == false)
    {
        return;
    }

    if (Timestamp_Compare(TimeSetParams.end_time, Timestamp_GetCurrent()))
    {
        IsTimeValid = true;
        PCF8523_Drv_SetTime(&TimeSetParams.set_time);
        TimeSourceSetRespCallback(TimeServerInstanceIdx);

        TimeSetParams.set_time_valid = false;
    }
}

static void OnSecondElapsed(void)
{
    if ((TimeServerInstanceIdx == INSTANCE_INDEX_UNKNOWN) || (PCF8523_Drv_IsInitialized() == false))
    {
        return;
    }

    if (ReceivedTimeGet)
    {
        struct TimeDate date;
        PCF8523_Drv_GetTime(&date);

        if (date.month > 12)
        {
            // In case of connection error with RTC the library returns the month equal to 165.
            // All the other data is also invalid
            LOG_INFO("RTC connection error");
            MCU_Health_SendSetFaultRequest(SILVAIR_ID, HEALTH_FAULT_ID_RTC_ERROR, TimeServerInstanceIdx);
            return;
        }

        MCU_Health_SendClearFaultRequest(SILVAIR_ID, HEALTH_FAULT_ID_RTC_ERROR, TimeServerInstanceIdx);
        TimeSourceGetRespCallback(TimeServerInstanceIdx, &date);
        ReceivedTimeGet = false;
    }
}

static void MeasureBatteryLevel(void)
{
    static unsigned long last_measurement_timestamp = 0;

    if ((IsBatteryLevelEverMeasured == true) && (IsBatteryDetected == false))
    {
        return;
    }

    if ((Timestamp_GetTimeElapsed(last_measurement_timestamp, Timestamp_GetCurrent()) > BATTERY_MEASUREMENT_PERIOD_MS) || (last_measurement_timestamp == 0))
    {
        uint16_t adc_readout        = analogRead(PIN_RTC_BATTERY);
        uint16_t battery_voltage_mv = (adc_readout * VOLTAGE_DIVIDER_COEFFICIENT * ANALOG_REFERENCE_VOLTAGE_MV) / ANALOG_MAX_READOUT;

        LastBatteryLevelPercent = 100;
        for (size_t i = 0; i < ARRAY_SIZE(cr1220_battery_curve_mv) - 1; i++)
        {
            if (battery_voltage_mv < cr1220_battery_curve_mv[i])
            {
                LastBatteryLevelPercent = (i - 1) * BATTERY_CURVE_STEP_PERCENT;
                break;
            }
        }

        if (IsBatteryDetected)
        {
            UpdateBatteryStatus();
            UpdateHealthFaultStatus();
        }
        LOG_INFO("RTC battery voltage: %d mV (%d%%)", battery_voltage_mv, LastBatteryLevelPercent);

        last_measurement_timestamp = millis();

        if (!IsBatteryLevelEverMeasured && (LastBatteryLevelPercent > BATTERY_NOT_DETECTED_THRESHOLD_PERCENT))
        {
            IsBatteryDetected = true;
        }
        IsBatteryLevelEverMeasured = true;
    }
}

static void UpdateBatteryStatus(void)
{
    uint32_t time_to_discharge_minutes = LastBatteryLevelPercent * BATTERY_DISCHARGE_TIME_PER_PERCENT_IN_MINUTES;

    uint8_t battery_flags = BATTERY_FLAGS_PRESENCE_PRESENT_AND_REMOVABLE | BATTERY_FLAGS_CHARGING_IS_NOT_CHARGEABLE;
    if (LastBatteryLevelPercent <= BATTERY_LEVEL_CRITICAL_LOW_PERCENT)
    {
        battery_flags |= BATTERY_FLAGS_INDICATOR_CRITICALLY_LOW_LEVEL;
        battery_flags |= BATTERY_FLAGS_SERVICEABILITY_BATTERY_REQUIRES_SERVICE;
    }
    else if (LastBatteryLevelPercent <= BATTERY_LEVEL_LOW_PERCENT)
    {
        battery_flags |= BATTERY_FLAGS_INDICATOR_LOW_LEVEL;
        battery_flags |= BATTERY_FLAGS_SERVICEABILITY_BATTERY_REQUIRES_SERVICE;
    }
    else
    {
        battery_flags |= BATTERY_FLAGS_INDICATOR_GOOD_LEVEL;
        battery_flags |= BATTERY_FLAGS_SERVICEABILITY_BATTERY_DOES_NOT_REQUIRE_SERVICE;
    }

    uint8_t payload[] = {
        TimeServerInstanceIdx,
        LastBatteryLevelPercent,
        ((uint8_t)(time_to_discharge_minutes & 0xFF)),
        ((uint8_t)((time_to_discharge_minutes >> 8) & 0xFF)),
        ((uint8_t)((time_to_discharge_minutes >> 16) & 0xFF)),
        (BATTERY_TIME_TO_CHARGE_UNKNOWN & 0xFF),
        ((BATTERY_TIME_TO_CHARGE_UNKNOWN >> 8) & 0xFF),
        ((BATTERY_TIME_TO_CHARGE_UNKNOWN >> 16) & 0xFF),
        battery_flags,
    };

    UART_SendBatteryStatusSetRequest(payload, sizeof(payload));
}

static void UpdateHealthFaultStatus(void)
{
    if (LastBatteryLevelPercent <= BATTERY_LEVEL_CRITICAL_LOW_PERCENT)
    {
        MCU_Health_SendSetFaultRequest(SILVAIR_ID, HEALTH_FAULT_ID_BATTERY_LOW_WARNING, TimeServerInstanceIdx);
        MCU_Health_SendSetFaultRequest(SILVAIR_ID, HEALTH_FAULT_ID_BATTERY_LOW_ERROR, TimeServerInstanceIdx);
    }
    else if (LastBatteryLevelPercent <= BATTERY_LEVEL_LOW_PERCENT)
    {
        MCU_Health_SendSetFaultRequest(SILVAIR_ID, HEALTH_FAULT_ID_BATTERY_LOW_WARNING, TimeServerInstanceIdx);
        MCU_Health_SendClearFaultRequest(SILVAIR_ID, HEALTH_FAULT_ID_BATTERY_LOW_ERROR, TimeServerInstanceIdx);
    }
    else
    {
        MCU_Health_SendClearFaultRequest(SILVAIR_ID, HEALTH_FAULT_ID_BATTERY_LOW_WARNING, TimeServerInstanceIdx);
        MCU_Health_SendClearFaultRequest(SILVAIR_ID, HEALTH_FAULT_ID_BATTERY_LOW_ERROR, TimeServerInstanceIdx);
    }
}

static void SendGetRespTimeUnknown(void)
{
    if (PCF8523_Drv_IsInitialized() == false)
    {
        return;
    }

    struct TimeDate date = {0};
    TimeSourceGetRespCallback(TimeServerInstanceIdx, &date);
}

static void CheckRtcHasValidTime(void)
{
    if (PCF8523_Drv_IsInitialized() == false)
    {
        return;
    }

    // Check if RTC Control 3 register has a default value - it means that RTC was reset.
    // This can happen, when battery is removed and inserted during power off state.
    if (PCF8523_Drv_IsControl3Default())
    {
        IsTimeValid = false;
        LOG_DEBUG("Time is not valid in RTC after reset");
    }
    else
    {
        IsTimeValid = true;
        LOG_DEBUG("Time is valid in RTC after reset");
    }
}
