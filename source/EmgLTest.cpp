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

#include "EmgLTest.h"

#include "Log.h"
#include "MeshGenericBattery.h"
#include "Timestamp.h"
#include "UARTProtocol.h"

#define ELT_FUNCTIONAL_TEST_TIME_MS 1000
#define ELT_DURATION_TEST_TIME_MS (60 * 1000)

#define EMG_LIGHTING_OPCODE 0x00EA3601
#define EMG_LIGHTING_TEST2_OPCODE 0x00E93601

#define EMG_BATTERY_MEASUREMENT_PERIOD_MS (60 * 1000)
#define EMG_BATTERY_LEVEL_LOW_PERCENT 30
#define EMG_BATTERY_LEVEL_CRITICAL_LOW_PERCENT 10
#define EMG_BATTERY_NOT_DETECTED_THRESHOLD_PERCENT 0

#define EMG_ANALOG_MAX_READOUT 1023
#define EMG_ANALOG_DEAD_RANGE_VALUE 10

#define EMG_DURATION_RESULT_TEST_LENGTH_UNKNOWN 0xFFFF

static void MeshMessageRequest1Send(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t subopcode, uint8_t *p_payload, size_t len);

//EL
static void ElInhibitEnter(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len);
static void ElInhibitExit(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len);
static void ElStateGet(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len);
static void ElPropertyStatus(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len);
static void ElLampOperationTimeGet(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len);
static void ElLampOperationTimeClear(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len);
static void ElRestEnter(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len);
static void ElRestExit(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len);

//ELT
static void EltFunctionalTestGet(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len);
static void EltFunctionalTestStart(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len);
static void EltFunctionalTestStop(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len);
static void EltDurationTestGet(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len);
static void EltDurationTestStart(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len);
static void EltDurationTestStop(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len);

static void    UpdateBatteryStatus(uint8_t battery_level_percent);
static void    SimulateEltTest(void);
static uint8_t SimulateBatteryLevel(void);

static uint8_t            InstanceIndex = INSTANCE_INDEX_UNKNOWN;
static EmgLightingState_T ElState       = EMG_LIGHTING_STATE_NORMAL;

static uint32_t ElTotalOperationTimeOffsetMs;
static uint32_t ElEmergencyTimeMs = 10 * 1000;

static uint32_t ELTFunctionalTestTimeStartMs;
static uint32_t ELTDurationTestTimeStartMs;

static EmgLightingTestTestExecutionStatus ELTFunctionalTestStatus = EMG_LIGHTING_TEST_STATUS_UNKNOWN;
static EmgLightingTestTestExecutionStatus ELTDurationTestStatus   = EMG_LIGHTING_TEST_STATUS_UNKNOWN;

void EmgLTest_LightElSrvProcessMessage(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len)
{
    if ((p_payload == NULL) || (p_header == NULL) || (len < 1))
    {
        return;
    }

    EmgLightingSubOpcode subopcode = (EmgLightingSubOpcode)p_payload[0];

    LOG_INFO("LightElSrv subopcode: 0x%02X", subopcode);

    switch (subopcode)
    {
        case EMG_LIGHTING_SUBOPCODE_INHIBIT_ENTER:
            ElInhibitEnter(p_header, p_payload + 1, len - 1);
            break;

        case EMG_LIGHTING_SUBOPCODE_INHIBIT_EXIT:
            ElInhibitExit(p_header, p_payload + 1, len - 1);
            break;

        case EMG_LIGHTING_SUBOPCODE_STATE_GET:
            ElStateGet(p_header, p_payload + 1, len - 1);
            break;

        case EMG_LIGHTING_SUBOPCODE_PROPERTY_STATUS:
            ElPropertyStatus(p_header, p_payload + 1, len - 1);
            break;

        case EMG_LIGHTING_SUBOPCODE_LAMP_OPERATION_TIME_GET:
            ElLampOperationTimeGet(p_header, p_payload + 1, len - 1);
            break;

        case EMG_LIGHTING_SUBOPCODE_LAMP_OPERATION_TIME_CLEAR:
            ElLampOperationTimeClear(p_header, p_payload + 1, len - 1);
            break;

        case EMG_LIGHTING_SUBOPCODE_REST_ENTER:
            ElRestEnter(p_header, p_payload + 1, len - 1);
            break;

        case EMG_LIGHTING_SUBOPCODE_REST_EXIT:
            ElRestExit(p_header, p_payload + 1, len - 1);
            break;

        default:
            LOG_INFO("LightElSrv subopcode: 0x%02X not supported", subopcode);
            break;
    }
}

void EmgLTest_LightElTestSrvProcessMessage(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len)
{
    if ((p_payload == NULL) || (p_header == NULL) || (len < 1))
    {
        return;
    }

    EmgLightingTestSubOpcode subopcode = (EmgLightingTestSubOpcode)p_payload[0];

    LOG_INFO("LightElTestSrv subopcode: 0x%02X", subopcode);

    switch (subopcode)
    {
        case EMG_LIGHTING_TEST_SUBOPCODE_FUNCTIONAL_TEST_GET:
            EltFunctionalTestGet(p_header, p_payload + 1, len - 1);
            break;

        case EMG_LIGHTING_TEST_SUBOPCODE_FUNCTIONAL_TEST_START:
            EltFunctionalTestStart(p_header, p_payload + 1, len - 1);
            break;

        case EMG_LIGHTING_TEST_SUBOPCODE_FUNCTIONAL_TEST_STOP:
            EltFunctionalTestStop(p_header, p_payload + 1, len - 1);
            break;

        case EMG_LIGHTING_TEST_SUBOPCODE_DURATION_TEST_GET:
            EltDurationTestGet(p_header, p_payload + 1, len - 1);
            break;

        case EMG_LIGHTING_TEST_SUBOPCODE_DURATION_TEST_START:
            EltDurationTestStart(p_header, p_payload + 1, len - 1);
            break;

        case EMG_LIGHTING_TEST_SUBOPCODE_DURATION_TEST_STOP:
            EltDurationTestStop(p_header, p_payload + 1, len - 1);
            break;

        default:
            LOG_INFO("LightElTestSrv subopcode: 0x%02X not supported", subopcode);
            break;
    }
}

void LoopEmgLTest(void)
{
    if ((InstanceIndex == INSTANCE_INDEX_UNKNOWN) || (!ENABLE_EMG_L_TEST))
    {
        return;
    }

    static unsigned long last_measurement_timestamp = 0;

    if ((Timestamp_GetTimeElapsed(last_measurement_timestamp, Timestamp_GetCurrent()) > EMG_BATTERY_MEASUREMENT_PERIOD_MS) || (last_measurement_timestamp == 0))
    {
        last_measurement_timestamp = millis();

        uint8_t battery_level_percent = SimulateBatteryLevel();

        LOG_INFO("ELT battery level: %d%%", battery_level_percent);

        if (battery_level_percent == 0)
        {
            ElState = EMG_LIGHTING_STATE_BATTERY_DISCHARGED;
        }
        else if (ElState == EMG_LIGHTING_STATE_BATTERY_DISCHARGED)
        {
            ElState = EMG_LIGHTING_STATE_NORMAL;
        }

        UpdateBatteryStatus(battery_level_percent);
    }
}

void EmgLTest_SetInstanceIdx(uint8_t instance_index)
{
    InstanceIndex = instance_index;
}

uint8_t EmgLTest_GetInstanceIdx(void)
{
    return InstanceIndex;
}

void EmgLTest_Init(void)
{
    pinMode(PIN_ENCODER_SW, INPUT_PULLUP);
}

static void UpdateBatteryStatus(uint8_t battery_level_percent)
{
    uint8_t battery_flags = BATTERY_FLAGS_PRESENCE_PRESENT_AND_NON_REMOVABLE | BATTERY_FLAGS_CHARGING_IS_CHARGEABLE_AND_IS_NOT_CHARGING |
                            BATTERY_FLAGS_SERVICEABILITY_BATTERY_DOES_NOT_REQUIRE_SERVICE;

    if (battery_level_percent <= EMG_BATTERY_LEVEL_CRITICAL_LOW_PERCENT)
    {
        battery_flags |= BATTERY_FLAGS_INDICATOR_CRITICALLY_LOW_LEVEL;
    }
    else if (battery_level_percent <= EMG_BATTERY_LEVEL_LOW_PERCENT)
    {
        battery_flags |= BATTERY_FLAGS_INDICATOR_LOW_LEVEL;
    }
    else
    {
        battery_flags |= BATTERY_FLAGS_INDICATOR_GOOD_LEVEL;
    }

    uint8_t payload[] = {
        InstanceIndex,
        battery_level_percent,
        ((uint8_t)(BATTERY_TIME_TO_DISCHARGE_UNKNOWN & 0xFF)),
        ((uint8_t)((BATTERY_TIME_TO_DISCHARGE_UNKNOWN >> 8) & 0xFF)),
        ((uint8_t)((BATTERY_TIME_TO_DISCHARGE_UNKNOWN >> 16) & 0xFF)),
        (BATTERY_TIME_TO_CHARGE_UNKNOWN & 0xFF),
        ((BATTERY_TIME_TO_CHARGE_UNKNOWN >> 8) & 0xFF),
        ((BATTERY_TIME_TO_CHARGE_UNKNOWN >> 16) & 0xFF),
        battery_flags,
    };

    UART_SendBatteryStatusSetRequest(payload, sizeof(payload));
}

static void MeshMessageRequest1Send(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t subopcode, uint8_t *p_payload, size_t len)
{
    size_t size = sizeof(p_header->instance_index) + sizeof(p_header->instance_subindex) + sizeof(subopcode) + p_header->mesh_cmd_size + len;

    uint8_t buff[size];
    size_t  index = 0;

    buff[index++] = p_header->instance_index;
    buff[index++] = p_header->instance_subindex;

    if (p_header->mesh_cmd_size == 3)
    {
        buff[index++] = (uint8_t)(p_header->mesh_cmd >> 16);
        buff[index++] = (uint8_t)(p_header->mesh_cmd >> 8);
        buff[index++] = (uint8_t)(p_header->mesh_cmd);
    }
    else if (p_header->mesh_cmd_size == 2)
    {
        buff[index++] = (uint8_t)(p_header->mesh_cmd >> 8);
        buff[index++] = (uint8_t)(p_header->mesh_cmd);
    }
    else
    {
        buff[index++] = (uint8_t)(p_header->mesh_cmd);
    }

    buff[index++] = subopcode;

    memcpy(buff + index, p_payload, len);
    index += len;

    UART_SendMeshMessageRequest1(buff, index);
}

//EL
static void ElInhibitEnter(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len)
{
    if (len != 0)
    {
        return;
    }

    ElState = EMG_LIGHTING_STATE_INHIBIT;

    ElSrvStateStatus_T resp;
    resp.state = ElState;

    MeshMessageRequest1Send(p_header, EMG_LIGHTING_SUBOPCODE_STATE_STATUS, (uint8_t *)&resp, sizeof(resp));
}

static void ElInhibitExit(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len)
{
    if (len != 0)
    {
        return;
    }

    ElState = EMG_LIGHTING_STATE_NORMAL;

    ElSrvStateStatus_T resp;
    resp.state = ElState;

    MeshMessageRequest1Send(p_header, EMG_LIGHTING_SUBOPCODE_STATE_STATUS, (uint8_t *)&resp, sizeof(resp));
}

static void ElStateGet(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len)
{
    if (len != 0)
    {
        return;
    }

    SimulateEltTest();

    ElSrvStateStatus_T resp;
    resp.state = ElState;

    // Encoder switch used to emulate emergency mode
    if ((!digitalRead(PIN_ENCODER_SW)) && (resp.state == EMG_LIGHTING_STATE_REST))
    {
        resp.state = EMG_LIGHTING_STATE_REST;
    }
    else if (!digitalRead(PIN_ENCODER_SW))
    {
        resp.state = EMG_LIGHTING_STATE_EMERGENCY;
    }

    MeshMessageRequest1Send(p_header, EMG_LIGHTING_SUBOPCODE_STATE_STATUS, (uint8_t *)&resp, sizeof(resp));
}

static void ElPropertyStatus(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len)
{
    if (sizeof(ElSrvPropertyStatus_T) != len)
    {
        return;
    }

    ElSrvPropertyStatus_T *frame = (ElSrvPropertyStatus_T *)p_payload;

    if ((frame->property_id != EMG_LIGHTING_PROPERTY_ID_LIGHTNESS) && (frame->property_id != EMG_LIGHTING_PROPERTY_ID_PROLONG_TIME))
    {
        LOG_INFO("LightElTestSrv property_id status: 0x%04X not supported", frame->property_id);
    }
    else
    {
        LOG_INFO("LightElTestSrv property_id status: 0x%04X, value: 0x%04X", frame->property_id, frame->property_value);
    }
}

static void ElLampOperationTimeGet(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len)
{
    if (len != 0)
    {
        return;
    }

    ElSrvOperationTimeStatus_T resp;
    resp.emergency_time       = ElEmergencyTimeMs / 1000;
    resp.total_operation_time = Timestamp_GetTimeElapsed(ElTotalOperationTimeOffsetMs, Timestamp_GetCurrent()) / 1000;

    MeshMessageRequest1Send(p_header, EMG_LIGHTING_SUBOPCODE_LAMP_OPERATION_TIME_STATUS, (uint8_t *)&resp, sizeof(resp));
}

static void ElLampOperationTimeClear(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len)
{
    if (len != 0)
    {
        return;
    }

    ElTotalOperationTimeOffsetMs = Timestamp_GetCurrent();
    ElEmergencyTimeMs            = 0;

    ElSrvOperationTimeStatus_T resp;
    resp.emergency_time       = 0;
    resp.total_operation_time = 0;

    MeshMessageRequest1Send(p_header, EMG_LIGHTING_SUBOPCODE_LAMP_OPERATION_TIME_STATUS, (uint8_t *)&resp, sizeof(resp));
}

static void ElRestEnter(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len)
{
    if (len != 0)
    {
        return;
    }

    ElState = EMG_LIGHTING_STATE_REST;

    ElSrvStateStatus_T resp;
    resp.state = ElState;

    MeshMessageRequest1Send(p_header, EMG_LIGHTING_SUBOPCODE_STATE_STATUS, (uint8_t *)&resp, sizeof(resp));
}

static void ElRestExit(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len)
{
    if (len != 0)
    {
        return;
    }

    ElState = EMG_LIGHTING_STATE_NORMAL;

    ElSrvStateStatus_T resp;

    // Encoder switch used to emulate emergency mode
    if (!digitalRead(PIN_ENCODER_SW))
    {
        resp.state = EMG_LIGHTING_STATE_EMERGENCY;
    }
    else
    {
        resp.state = ElState;
    }

    MeshMessageRequest1Send(p_header, EMG_LIGHTING_SUBOPCODE_STATE_STATUS, (uint8_t *)&resp, sizeof(resp));
}

//ELT
static void EltFunctionalTestGet(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len)
{
    if (len != 0)
    {
        return;
    }

    EltSrvFunctionalTestStatus_T resp;
    resp.status                        = ELTFunctionalTestStatus;
    resp.result.battery_duration_fault = false;
    resp.result.battery_fault          = false;
    resp.result.circuit_fault          = false;
    resp.result.lamp_fault             = false;
    resp.result.rfu                    = 0;

    MeshMessageRequest1Send(p_header, EMG_LIGHTING_TEST_SUBOPCODE_FUNCTIONAL_TEST_STATUS, (uint8_t *)&resp, sizeof(resp));
}

static void EltFunctionalTestStart(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len)
{
    if (len != 0)
    {
        return;
    }

    if (ElState == EMG_LIGHTING_STATE_NORMAL)
    {
        ElState = EMG_LIGHTING_STATE_FUNCTIONAL_TEST_IN_PROGRESS;

        ELTFunctionalTestTimeStartMs = Timestamp_GetCurrent();
        ELTFunctionalTestStatus      = EMG_LIGHTING_TEST_STATUS_UNKNOWN;
    }

    ElSrvStateStatus_T resp;
    resp.state = ElState;

    p_header->mesh_cmd = EMG_LIGHTING_OPCODE;
    MeshMessageRequest1Send(p_header, EMG_LIGHTING_SUBOPCODE_STATE_STATUS, (uint8_t *)&resp, sizeof(resp));
}

static void EltFunctionalTestStop(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len)
{
    if (len != 0)
    {
        return;
    }

    ElState                 = EMG_LIGHTING_STATE_NORMAL;
    ELTFunctionalTestStatus = EMG_LIGHTING_TEST_STATUS_UNKNOWN;

    ElSrvStateStatus_T resp;
    resp.state = ElState;

    p_header->mesh_cmd = EMG_LIGHTING_OPCODE;
    MeshMessageRequest1Send(p_header, EMG_LIGHTING_SUBOPCODE_STATE_STATUS, (uint8_t *)&resp, sizeof(resp));
}

static void EltDurationTestGet(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len)
{
    if (len != 0)
    {
        return;
    }

    EltSrvDurationTestStatus_T resp;
    resp.status                        = ELTDurationTestStatus;
    resp.result.battery_duration_fault = false;
    resp.result.battery_fault          = false;
    resp.result.circuit_fault          = false;
    resp.result.lamp_fault             = false;
    resp.result.rfu                    = 0;

    if (ELTDurationTestStatus == EMG_LIGHTING_TEST_STATUS_FINISHED)
    {
        resp.test_length = ELT_DURATION_TEST_TIME_MS / 1000;
    }
    else
    {
        resp.test_length = EMG_DURATION_RESULT_TEST_LENGTH_UNKNOWN;
    }

    MeshMessageRequest1Send(p_header, EMG_LIGHTING_TEST_SUBOPCODE_DURATION_TEST_STATUS, (uint8_t *)&resp, sizeof(resp));
}

static void EltDurationTestStart(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len)
{
    if (len != 0)
    {
        return;
    }

    if ((ElState == EMG_LIGHTING_STATE_NORMAL) && (SimulateBatteryLevel() == 100))
    {
        ElState = EMG_LIGHTING_STATE_DURATION_TEST_IN_PROGRESS;

        ELTDurationTestTimeStartMs = Timestamp_GetCurrent();
        ELTDurationTestStatus      = EMG_LIGHTING_TEST_STATUS_UNKNOWN;
    }

    ElSrvStateStatus_T resp;
    resp.state = ElState;

    p_header->mesh_cmd = EMG_LIGHTING_OPCODE;
    MeshMessageRequest1Send(p_header, EMG_LIGHTING_SUBOPCODE_STATE_STATUS, (uint8_t *)&resp, sizeof(resp));
}

static void EltDurationTestStop(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len)
{
    if (len != 0)
    {
        return;
    }

    ElState               = EMG_LIGHTING_STATE_NORMAL;
    ELTDurationTestStatus = EMG_LIGHTING_TEST_STATUS_UNKNOWN;

    ElSrvStateStatus_T resp;
    resp.state = ElState;

    p_header->mesh_cmd = EMG_LIGHTING_OPCODE;
    MeshMessageRequest1Send(p_header, EMG_LIGHTING_SUBOPCODE_STATE_STATUS, (uint8_t *)&resp, sizeof(resp));
}

static void SimulateEltTest(void)
{
    if (ElState == EMG_LIGHTING_STATE_FUNCTIONAL_TEST_IN_PROGRESS)
    {
        if (Timestamp_GetTimeElapsed(ELTFunctionalTestTimeStartMs, Timestamp_GetCurrent()) > ELT_FUNCTIONAL_TEST_TIME_MS)
        {
            ELTFunctionalTestStatus = EMG_LIGHTING_TEST_STATUS_FINISHED;
            ElState                 = EMG_LIGHTING_STATE_NORMAL;
        }
    }
    else if (ElState == EMG_LIGHTING_STATE_DURATION_TEST_IN_PROGRESS)
    {
        if (Timestamp_GetTimeElapsed(ELTDurationTestTimeStartMs, Timestamp_GetCurrent()) > ELT_DURATION_TEST_TIME_MS)
        {
            ELTDurationTestStatus = EMG_LIGHTING_TEST_STATUS_FINISHED;
            ElState               = EMG_LIGHTING_STATE_NORMAL;
        }
    }
}

static uint8_t SimulateBatteryLevel(void)
{
    //Simulate/Read battery level from potentiometer position
    uint16_t analog_value = EMG_ANALOG_MAX_READOUT - analogRead(PIN_ANALOG);

    if (analog_value > EMG_ANALOG_MAX_READOUT - EMG_ANALOG_DEAD_RANGE_VALUE)
    {
        analog_value = EMG_ANALOG_MAX_READOUT - EMG_ANALOG_DEAD_RANGE_VALUE;
    }

    if (analog_value < EMG_ANALOG_DEAD_RANGE_VALUE)
    {
        analog_value = EMG_ANALOG_DEAD_RANGE_VALUE;
    }

    return (analog_value - EMG_ANALOG_DEAD_RANGE_VALUE) * BATTERY_LEVEL_MAX / (EMG_ANALOG_MAX_READOUT - 2 * EMG_ANALOG_DEAD_RANGE_VALUE);
}
