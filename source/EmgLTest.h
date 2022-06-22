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

#ifndef LIGHTELTESTSRV_H_
#define LIGHTELTESTSRV_H_

#include <stdint.h>
#include <stdio.h>

#include "Mesh.h"

/**
 * UART frame message structures for EL
*/
typedef enum
{
    EMG_LIGHTING_SUBOPCODE_INHIBIT_ENTER              = 0x00,
    EMG_LIGHTING_SUBOPCODE_INHIBIT_EXIT               = 0x02,
    EMG_LIGHTING_SUBOPCODE_STATE_GET                  = 0x04,
    EMG_LIGHTING_SUBOPCODE_STATE_STATUS               = 0x05,
    EMG_LIGHTING_SUBOPCODE_PROPERTY_STATUS            = 0x09,
    EMG_LIGHTING_SUBOPCODE_LAMP_OPERATION_TIME_GET    = 0x0A,
    EMG_LIGHTING_SUBOPCODE_LAMP_OPERATION_TIME_CLEAR  = 0x0B,
    EMG_LIGHTING_SUBOPCODE_LAMP_OPERATION_TIME_STATUS = 0x0D,
    EMG_LIGHTING_SUBOPCODE_REST_ENTER                 = 0x0E,
    EMG_LIGHTING_SUBOPCODE_REST_EXIT                  = 0x10,
} EmgLightingSubOpcode;

typedef enum
{
    EMG_LIGHTING_STATE_NORMAL                      = 0x03,
    EMG_LIGHTING_STATE_EMERGENCY                   = 0x05,
    EMG_LIGHTING_STATE_EXTENDED_EMERGENCY          = 0x05,
    EMG_LIGHTING_STATE_REST                        = 0x08,
    EMG_LIGHTING_STATE_INHIBIT                     = 0x0A,
    EMG_LIGHTING_STATE_DURATION_TEST_IN_PROGRESS   = 0x0C,
    EMG_LIGHTING_STATE_FUNCTIONAL_TEST_IN_PROGRESS = 0x0E,
    EMG_LIGHTING_STATE_BATTERY_DISCHARGED          = 0x0F
} EmgLightingState_T;

typedef enum
{
    EMG_LIGHTING_PROPERTY_ID_LIGHTNESS    = 0xFF80,
    EMG_LIGHTING_PROPERTY_ID_PROLONG_TIME = 0xFF83
} EmgLightingPropertyId_T;

typedef struct __attribute__((packed))
{
    EmgLightingState_T state;
} ElSrvStateStatus_T;

typedef struct __attribute__((packed))
{
    EmgLightingPropertyId_T property_id;
    uint16_t                property_value;
} ElSrvPropertyStatus_T;

typedef struct __attribute__((packed))
{
    uint32_t total_operation_time;
    uint32_t emergency_time;
} ElSrvOperationTimeStatus_T;

/**
 * Validate UART frame message structures size for EL
 */
static_assert(sizeof(ElSrvStateStatus_T) == 1, "Wrong size of the struct ElSrvStateStatus_T");
static_assert(sizeof(ElSrvPropertyStatus_T) == 4, "Wrong size of the struct ElSrvPropertyStatus_T");
static_assert(sizeof(ElSrvOperationTimeStatus_T) == 8, "Wrong size of the struct ElSrvOperationTimeStatus_T");

/**
 * UART frame message structures for ELT
 */
typedef enum
{
    EMG_LIGHTING_TEST_SUBOPCODE_FUNCTIONAL_TEST_GET    = 0x00,
    EMG_LIGHTING_TEST_SUBOPCODE_FUNCTIONAL_TEST_START  = 0x01,
    EMG_LIGHTING_TEST_SUBOPCODE_FUNCTIONAL_TEST_STOP   = 0x02,
    EMG_LIGHTING_TEST_SUBOPCODE_FUNCTIONAL_TEST_STATUS = 0x03,
    EMG_LIGHTING_TEST_SUBOPCODE_DURATION_TEST_GET      = 0x04,
    EMG_LIGHTING_TEST_SUBOPCODE_DURATION_TEST_START    = 0x05,
    EMG_LIGHTING_TEST_SUBOPCODE_DURATION_TEST_STOP     = 0x06,
    EMG_LIGHTING_TEST_SUBOPCODE_DURATION_TEST_STATUS   = 0x07
} EmgLightingTestSubOpcode;

typedef enum
{
    EMG_LIGHTING_TEST_STATUS_FINISHED = 0x00,
    EMG_LIGHTING_TEST_STATUS_UNKNOWN  = 0x07
} EmgLightingTestTestExecutionStatus;

typedef struct __attribute__((packed))
{
    uint8_t lamp_fault : 1;
    uint8_t battery_fault : 1;
    uint8_t circuit_fault : 1;
    uint8_t battery_duration_fault : 1;
    uint8_t rfu : 4;
} EmgLightingTestTestExecutionResult;

typedef struct __attribute__((packed))
{
    EmgLightingTestTestExecutionStatus status;
    EmgLightingTestTestExecutionResult result;
} EltSrvFunctionalTestStatus_T;

typedef struct __attribute__((packed))
{
    EmgLightingTestTestExecutionStatus status;
    EmgLightingTestTestExecutionResult result;
    uint16_t                           test_length;
} EltSrvDurationTestStatus_T;

/**
 * Validate UART frame message structures size for ELT
 */
static_assert(sizeof(EltSrvFunctionalTestStatus_T) == 2, "Wrong size of the struct EltSrvFunctionalTestStatus_T");
static_assert(sizeof(EltSrvDurationTestStatus_T) == 4, "Wrong size of the struct EltSrvDurationTestStatus_T");

/*
 *  Light EL Server message process
 *
 *  @param * p_header    Mesh Message Request1 header
 *  @param * p_payload   Pointer to mesh message payload
 *  @param len           Payload length
 */
void EmgLTest_LightElSrvProcessMessage(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len);

/*
 *  Light EL Test Server message process
 *
 *  @param * p_header    Mesh Message Request1 header
 *  @param * p_payload   Pointer to mesh message payload
 *  @param len           Payload length
 */
void EmgLTest_LightElTestSrvProcessMessage(Mesh_MeshMessageRequest1Cmd_T *p_header, uint8_t *p_payload, size_t len);

/*
 *  Light EL Test Server loop process
 */
void LoopEmgLTest(void);

/*
 *  Light EL Test Server instance index setter
 *
 *  @param instance_index Instance index
 */
void EmgLTest_SetInstanceIdx(uint8_t instance_index);

/*
 *  Light EL Test Server instance index getter
 *
 *  @return             Instance index
 */
uint8_t EmgLTest_GetInstanceIdx(void);

/*
 *  Light EL Test Server initialization
 *
 */
void EmgLTest_Init(void);

#endif    // LIGHTELTESTSRV_H_
