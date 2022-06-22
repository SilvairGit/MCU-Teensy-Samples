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

#ifndef MESH_H_
#define MESH_H_


#include "SensorInput.h"
#include "stddef.h"
#include "stdint.h"


/**
 * Supported Mesh Model IDs definitions
 */
#define MESH_MODEL_ID_GENERIC_ONOFF_CLIENT 0x1001
#define MESH_MODEL_ID_SENSOR_CLIENT 0x1102
#define MESH_MODEL_ID_GENERIC_LEVEL_CLIENT 0x1003
#define MESH_MODEL_ID_LIGHT_L_CLIENT 0x1302
#define MESH_MODEL_ID_LIGHT_LC_CLIENT 0x1311
#define MESH_MODEL_ID_LIGHT_CTL_CLIENT 0x1305
#define MESH_MODEL_ID_LIGHT_LC_SERVER 0x130F
#define MESH_MODEL_ID_LIGHT_CTL_SERVER 0x1303
#define MESH_MODEL_ID_SENSOR_SERVER 0x1100
#define MESH_MODEL_ID_HEALTH_SERVER 0x0002
#define MESH_MODEL_ID_TIME_SERVER 0x1200
#define MESH_MODEL_ID_LIGHT_ELT_SERVER 0xE500

/**
* Supported Mesh Property IDs definitions
*/
#define MESH_PROP_ID_PRESENCE_DETECTED 0x004D
#define MESH_PROP_ID_PRESENT_AMBIENT_LIGHT_LEVEL 0x004E
#define MESH_PROP_ID_PRESENT_INPUT_CURRENT 0x0057
#define MESH_PROP_ID_PRESENT_INPUT_VOLTAGE 0x0059
#define MESH_PROP_ID_PRESENT_DEVICE_INPUT_POWER 0x0052
#define MESH_PROP_ID_PRECISE_TOTAL_DEVICE_ENERGY_USE 0x0072

/*
 *  Structure definition
 */
typedef struct
{
    uint8_t  instance_index;
    uint8_t  instance_subindex;
    uint32_t mesh_cmd;
    uint8_t  mesh_cmd_size;
} Mesh_MeshMessageRequest1Cmd_T;

/*
 *  This function should be called in Arduino main loop
 */
void Mesh_Loop(void);

/*
 *  Search for model ID in a message
 *
 *  @param p_payload            Pointer to payload
 *  @param len                  Payload len
 *  @param expected_model_id    Expected model ID
 *  @return                     True if found, false otherwise
 */
bool Mesh_IsModelAvailable(uint8_t *p_payload, uint8_t len, uint16_t expected_model_id);

/*
 *  Process Mesh Message Request command
 *
 *  @param p_payload    Pointer to payload
 *  @param len          Payload len
 */
void Mesh_ProcessMeshCommand(uint8_t *p_payload, size_t len);

/*
 *  Process Mesh Message Request1 command
 *
 *  @param p_payload    Pointer to payload
 *  @param len          Payload len
 */
void Mesh_ProcessMeshMessageRequest1(uint8_t *p_payload, size_t len);

/*
 *  Send Light Lightness Get message
 *
 *  @param instance_idx    Instance index
 */
void Mesh_SendLightLGet(uint8_t instance_idx);

/*
 *  Process new target lightness
 *
 *  @param current             Current lightness value
 *  @param target              Target lightness value
 *  @param transition_time     Transition time
 */
void ProcessTargetLightness(uint16_t current, uint16_t target, uint32_t transition_time);

/*
 *  Process new target lightness temperature
 *
 *  @param current             Current lightness temperature value
 *  @param target              Target lightness temperature value
 *  @param transition_time     Transition time
 */
void ProcessTargetLightnessTemp(uint16_t current, uint16_t target, uint32_t transition_time);

/*
 *  Send Generic OnOff Set Unacknowledged message with repeats.
 *
 *  @param instance_idx        Instance index.
 *  @param value               Generic OnOff target value.
 *  @param transition_time     Transition time (mesh format).
 *  @param delay_ms            Delay in miliseconds.
 *  @param num_of_repeats      Number of message repeats.
 *  @param is_new_transaction  Is it a new transaction?
 */
void Mesh_SendGenericOnOffSet(uint8_t instance_idx, bool value, uint32_t transition_time, uint32_t delay_ms, uint8_t num_of_repeats, bool is_new_transaction);

/*
 *  Send Generic OnOff Set Unacknowledged message with repeats.
 *
 *  @param instance_idx          Instance index.
 *  @param value                 Generic OnOff target value.
 *  @param transition_time       Transition time (mesh format).
 *  @param delay_ms              Delay in miliseconds.
 *  @param num_of_repeats        Number of message repeats.
*   @param repeats_interval_ms   time between repeats in milliseconds
 *  @param is_new_transaction    Is it a new transaction?
 */
void Mesh_SendGenericOnOffSetWithRepeatsInterval(uint8_t  instance_idx,
                                                 bool     value,
                                                 uint32_t transition_time,
                                                 uint32_t delay_ms,
                                                 uint8_t  num_of_repeats,
                                                 uint16_t repeats_interval_ms,
                                                 bool     is_new_transaction);


/*
 *  Send Light Lightness Set Unacknowledged message with repeats.
 *
 *  @param instance_idx        Instance index.
 *  @param value               Light Lightness target value.
 *  @param transition_time     Transition time (mesh format).
 *  @param delay_ms            Delay in miliseconds.
 *  @param num_of_repeats      Number of message repeats.
 *  @param is_new_transaction  Is it a new transaction?
 */
void Mesh_SendLightLSet(uint8_t instance_idx, uint16_t value, uint32_t transition_time, uint32_t delay_ms, uint8_t num_of_repeats, bool is_new_transaction);

/*
 *  Send Generic Delta Set Unacknowledged message with repeats.
 *
 *  @param instance_idx        Instance index.
 *  @param value               Delta change of the value.
 *  @param transition_time     Transition time (mesh format).
 *  @param delay_ms            Delay in miliseconds.
 *  @param num_of_repeats      Number of message repeats.
 *  @param is_new_transaction  Is it a new transaction?
 */
void Mesh_SendGenericDeltaSet(uint8_t instance_idx, int32_t value, uint32_t transition_time, uint32_t delay_ms, uint8_t num_of_repeats, bool is_new_transaction);


/*
 *  Send Generic Delta Set Unacknowledged message with repeats.
 *
 *  @param instance_idx          Instance index.
 *  @param value                 Delta change of the value.
 *  @param transition_time       Transition time (mesh format).
 *  @param delay_ms              Delay in miliseconds.
 *  @param num_of_repeats        Number of message repeats.
*   @param repeats_interval_ms   time between repeats in milliseconds
 *  @param is_new_transaction    Is it a new transaction?
 */
void Mesh_SendGenericDeltaSetWithRepeatsInterval(uint8_t  instance_idx,
                                                 int32_t  value,
                                                 uint32_t transition_time,
                                                 uint32_t delay_ms,
                                                 uint8_t  num_of_repeats,
                                                 uint16_t repeats_interval_ms,
                                                 bool     is_new_transaction);


/*
 *  Send Generic Delta Set Unacknowledged message with repeats.
 *
 *  @param instance_idx          Instance index.
 *  @param value                 Delta change of the value.
 *  @param transition_time       Transition time (mesh format).
 *  @param delay_ms              mesh delay in miliseconds.
 *  @param dispatch_time_ms      time between call this function and send this command through UART
 *  @param is_new_transaction    Is it a new transaction?
 */
void Mesh_SendGenericDeltaSetWithDispatchTime(uint8_t  instance_idx,
                                              int32_t  value,
                                              uint32_t transition_time,
                                              uint32_t delay_ms,
                                              uint16_t dispatch_time_ms,
                                              bool     is_new_transaction);

/*
 *  Send Generic Level Set Unacknowledged message with repeats.
 *
 *  @param instance_idx        Instance index.
 *  @param value               Generic Level target value.
 *  @param transition_time     Transition time (mesh format).
 *  @param delay_ms            Delay in miliseconds.
 *  @param num_of_repeats      Number of message repeats.
 *  @param is_new_transaction  Is it a new transaction?
 */
void Mesh_SendGenericLevelSet(uint8_t  instance_idx,
                              uint16_t value,
                              uint32_t transition_time,
                              uint32_t delay_ms,
                              uint8_t  num_of_repeats,
                              bool     is_new_transaction);

#endif    // MESH_H_
