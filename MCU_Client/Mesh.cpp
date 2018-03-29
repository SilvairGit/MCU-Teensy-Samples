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

/********************************************
 * INCLUDES                                 *
 ********************************************/

#include "Mesh.h"
#include "Arduino.h"
#include "Config.h"
#include "UART.h"

/********************************************
 * LOCAL #define CONSTANTS AND MACROS       *
 ********************************************/

/**
 * Supported Mesh Opcodes definitions
 */
#define MESH_MESSAGE_GENERIC_ONOFF_GET                      0x8201
#define MESH_MESSAGE_GENERIC_ONOFF_SET                      0x8202
#define MESH_MESSAGE_GENERIC_ONOFF_SET_UNACKNOWLEDGED       0x8203
#define MESH_MESSAGE_GENERIC_ONOFF_STATUS                   0x8204
#define MESH_MESSAGE_GENERIC_LEVEL_GET                      0x8205
#define MESH_MESSAGE_GENERIC_LEVEL_SET                      0x8206
#define MESH_MESSAGE_GENERIC_LEVEL_SET_UNACKNOWLEDGED       0x8207
#define MESH_MESSAGE_GENERIC_LEVEL_STATUS                   0x8208
#define MESH_MESSAGE_GENERIC_DELTA_SET                      0x8209
#define MESH_MESSAGE_GENERIC_DELTA_SET_UNACKNOWLEDGED       0x820A
#define MESH_MESSAGE_LIGHT_LIGHTNESS_GET                    0x824B
#define MESH_MESSAGE_LIGHT_LIGHTNESS_SET                    0x824C
#define MESH_MESSAGE_LIGHT_LIGHTNESS_SET_UNACKNOWLEDGED     0x824D
#define MESH_MESSAGE_LIGHT_LIGHTNESS_STATUS                 0x824E
#define MESH_MESSAGE_LIGHT_LC_MODE_GET                      0x8291
#define MESH_MESSAGE_LIGHT_LC_MODE_SET                      0x8292
#define MESH_MESSAGE_LIGHT_LC_MODE_SET_UNACKNOWLEDGED       0x8293
#define MESH_MESSAGE_LIGHT_LC_MODE_STATUS                   0x8294
#define MESH_MESSAGE_SENSOR_STATUS                          0x0052

/**
 * Default communication properties
 */
#define MESH_REPEATS_INTERVAL_MS                            50

/**
 * Used Mesh Messages len
 */
#define MESH_MESSAGE_GENERIC_ONOFF_SET_LEN                  8
#define MESH_MESSAGE_LIGHT_LIGHTNESS_SET_LEN                9
#define MESH_MESSAGE_GENERIC_DELTA_SET_LEN                  11
#define MESH_MESSAGE_LIGHT_LC_MODE_SET_LEN                  5

/*
 * Mesh time conversion definitions
 */
#define MESH_NUMBER_OF_MS_IN_100_MS                         (100UL)
#define MESH_NUMBER_OF_MS_IN_1S                             (10*MESH_NUMBER_OF_MS_IN_100_MS)
#define MESH_NUMBER_OF_MS_IN_10S                            (10*MESH_NUMBER_OF_MS_IN_1S)
#define MESH_NUMBER_OF_MS_IN_10MIN                          (60*MESH_NUMBER_OF_MS_IN_10S)
#define MESH_TRANSITION_TIME_STEP_RESOLUTION_MASK           0xC0
#define MESH_TRANSITION_TIME_STEP_RESOLUTION_100_MS         0x00
#define MESH_TRANSITION_TIME_STEP_RESOLUTION_1_S            0x40
#define MESH_TRANSITION_TIME_STEP_RESOLUTION_10_S           0x80
#define MESH_TRANSITION_TIME_STEP_RESOLUTION_10_MIN         0xC0
#define MESH_TRANSITION_TIME_NUMBER_OF_STEPS_MASK           0x3F
#define MESH_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE  0x3F
#define MESH_DELAY_TIME_STEP_MS                             5

/**
 * Sensor status description
 */
#define SS_FORMAT_MASK                                      0x01
#define SS_SHORT_LEN_MASK                                   0x1E
#define SS_SHORT_LEN_OFFSET                                 1
#define SS_SHORT_PROPERTY_ID_LOW_MASK                       0xE0
#define SS_SHORT_PROPERTY_ID_LOW_OFFSET                     5
#define SS_SHORT_PROPERTY_ID_HIGH_OFFSET                    3
#define SS_LONG_LEN_MASK                                    0xFE
#define SS_LONG_LEN_OFFSET                                  1

/**
 * Property IDs description
 */
#define PRESENCE_DETECTED_PROPERTY_ID                       0x004D
#define PRESENT_AMBIENT_LIGHT_LEVEL                         0x004E

/********************************************
 * LOCAL FUNCTIONS PROTOTYPES               *
 ********************************************/

/*
 *  Send Generic OnOff Set message
 *
 *  @param instance_idx      Instance index
 *  @param value             Generic OnOff value
 *  @param transition_time   Transition time (mesh format)
 *  @param delay_ms          Delay in miliseconds
 *  @param is_new            Is it a new transation?
 */
static void MeshInternal_SendGenericOnOffSet(uint8_t instance_idx,
                                             bool    value,
                                             uint8_t transition_time,
                                             uint8_t delay_ms,
                                             bool    is_new);

/*
 *  Send Light Lightness Set message
 *
 *  @param instance_idx      Instance index
 *  @param value             Light Lightness value
 *  @param transition_time   Transition time (mesh format)
 *  @param delay_ms          Delay in miliseconds
 *  @param is_new            Is it a new transation?
 */
static void MeshInternal_SendLightLightnessSet(uint8_t  instance_idx,
                                               uint16_t value,
                                               uint8_t  transition_time,
                                               uint8_t  delay_ms,
                                               bool     is_new);

/*
 *  Send Generic Delta Set message
 *
 *  @param instance_idx      Instance index
 *  @param value             Generic OnOff value
 *  @param transition_time   Transition time (mesh format)
 *  @param delay_ms          Delay in miliseconds
 *  @param is_new            Is it a new transation?
 */
static void MeshInternal_SendGenericDeltaSet(uint8_t instance_idx,
                                             int32_t value,
                                             uint8_t transition_time,
                                             uint8_t delay_ms,
                                             bool    is_new);

/*
 *  Send Light Lightness Controller Mode Set message
 *
 *  @param instance_idx      Instance index
 *  @param value             Generic OnOff value
 *  @param transition_time   Transition time (mesh format)
 *  @param delay_ms          Delay in miliseconds
 *  @param is_new            Is it a new transation?
 */
static void MeshInternal_SendLightLightnessControllerModeSet(uint8_t instance_idx, bool value);

/*
 *  Convert time from miliseconds to mesh format
 *
 *  @param time_ms   Time in miliseconds
 *  @return          Time in mesh format
 */
static uint8_t MeshInternal_ConvertFromMsToMeshFormatTransitionTime(uint32_t time_ms);

/*
 *  Process Sensor Status message
 *
 *  @param * p_payload    Pointer to message p_payload
 *  @param len            Payload length
 */
static void MeshInternal_ProcessSensorStatus(uint8_t * p_payload, size_t len);

/*
 *  Process Sensor Property
 *
 *  @param property_id   Property ID
 *  @param * p_payload   Pointer to message p_payload
 *  @param len           Payload length
 *  @param src_addr      Source address
 */
static void MeshInternal_ProcessSensorProperty(uint16_t  property_id,
                                               uint8_t * p_payload,
                                               size_t    len,
                                               uint16_t  src_addr);

/*
 *  Process PIR update
 *
 *  @param * p_payload    Pointer to message p_payload
 *  @param len            Payload length
 *  @param src_addr       Source address
 */
static void MeshInternal_ProcessPresenceDetected(uint8_t * p_payload, size_t len, uint16_t src_addr);

/*
 *  Process ALS update
 *
 *  @param * p_payload    Pointer to message p_payload
 *  @param len            Payload length
 *  @param src_addr       Source address
 */
static void MeshInternal_ProcessPresentAmbientLightLevel(uint8_t * p_payload,
                                                         size_t    len,
                                                         uint16_t  src_addr);

/********************************************
 * COMMON EXPORTED FUNCTION DEFINITIONS     *
 ********************************************/

bool Mesh_IsModelAvailable(uint8_t * p_payload, uint8_t len, uint16_t expected_model_id)
{
  for (size_t index = 0; index < len;)
  {
    uint16_t model_id = ((uint16_t)p_payload[index++]);
    model_id         |= ((uint16_t)p_payload[index++] << 8);

    if (expected_model_id == model_id)
    {
      return true;
    }
  }
  return false;
}

void Mesh_ProcessMeshCommand(uint8_t * p_payload, size_t len)
{
  size_t   index             = 0;
  uint8_t  instance_index    = p_payload[index++];
  uint8_t  instance_subindex = p_payload[index++];
  uint16_t mesh_cmd          = ((uint16_t)p_payload[index++]);
  mesh_cmd                  |= ((uint16_t)p_payload[index++] << 8);

  DEBUG_INTERFACE.printf("Process Mesh Command [%d %d 0x%02X]\n",
                         instance_index,
                         instance_subindex,
                         mesh_cmd);
  switch (mesh_cmd)
  {
    case MESH_MESSAGE_SENSOR_STATUS:
    {
      MeshInternal_ProcessSensorStatus(p_payload + index, len - index);
      break;
    }
  }
}

/****************************************************
 * LIGHTNESS SPECIFIC EXPORTED FUNCTION DEFINITIONS *
 ****************************************************/

void Mesh_SendGenericOnOffSet(uint8_t  instance_idx,
                              bool     value,
                              unsigned transition_time,
                              unsigned delay_ms)
{
  uint8_t transition_mesh_format =
    MeshInternal_ConvertFromMsToMeshFormatTransitionTime(transition_time);
  MeshInternal_SendGenericOnOffSet(instance_idx, value, transition_mesh_format, delay_ms/MESH_DELAY_TIME_STEP_MS, true);
  while(delay_ms >= MESH_REPEATS_INTERVAL_MS)
  {
    delay_ms -= MESH_REPEATS_INTERVAL_MS;
    MeshInternal_SendGenericOnOffSet(instance_idx, value, transition_mesh_format, delay_ms/MESH_DELAY_TIME_STEP_MS, false);
  }
}

void Mesh_SendLightLightnessSet(uint8_t  instance_idx,
                                uint16_t value,
                                unsigned transition_time,
                                unsigned delay_ms)
{
  uint8_t transition_mesh_format =
    MeshInternal_ConvertFromMsToMeshFormatTransitionTime(transition_time);
  MeshInternal_SendLightLightnessSet(instance_idx, value, transition_mesh_format, delay_ms/MESH_DELAY_TIME_STEP_MS, true);
  while(delay_ms >= MESH_REPEATS_INTERVAL_MS)
  {
    delay_ms -= MESH_REPEATS_INTERVAL_MS;
    MeshInternal_SendLightLightnessSet(instance_idx, value, transition_mesh_format, delay_ms/MESH_DELAY_TIME_STEP_MS, false);
  }
}

void Mesh_SendGenericDeltaSet(uint8_t  instance_idx,
                              int32_t  value,
                              bool     is_new,
                              unsigned transition_time,
                              unsigned delay_ms)
{
  uint8_t transition_mesh_format =
    MeshInternal_ConvertFromMsToMeshFormatTransitionTime(transition_time);
  MeshInternal_SendGenericDeltaSet(instance_idx, value, transition_mesh_format, delay_ms/MESH_DELAY_TIME_STEP_MS, is_new);
  while(delay_ms >= MESH_REPEATS_INTERVAL_MS)
  {
    delay_ms -= MESH_REPEATS_INTERVAL_MS;
    MeshInternal_SendGenericDeltaSet(instance_idx, value, transition_mesh_format, delay_ms/MESH_DELAY_TIME_STEP_MS, false);
  }
}

void Mesh_SendLightLightnessControllerModeSet(uint8_t instance_idx, bool value, unsigned repeats)
{
  MeshInternal_SendLightLightnessControllerModeSet(instance_idx, value);
  for (size_t i = 0; i < repeats; i++)
  {
    MeshInternal_SendLightLightnessControllerModeSet(instance_idx, value);
  }
}

/*************************************************
 * SENSOR SPECIFIC EXPORTED FUNCTION DEFINITIONS *
 *************************************************/

/*************************************************
 * LIGHTNESS SPECIFIC LOCAL FUNCTION DEFINITIONS *
 *************************************************/

static void MeshInternal_SendGenericOnOffSet(uint8_t instance_idx,
                                             bool    value,
                                             uint8_t transition_time,
                                             uint8_t delay_ms,
                                             bool    is_new)
{
  uint8_t        buf[MESH_MESSAGE_GENERIC_ONOFF_SET_LEN];
  size_t         index = 0;
  static uint8_t tid   = 0;

  buf[index++] = instance_idx;
  buf[index++] = 0x00;
  buf[index++] = lowByte(MESH_MESSAGE_GENERIC_ONOFF_SET_UNACKNOWLEDGED);
  buf[index++] = highByte(MESH_MESSAGE_GENERIC_ONOFF_SET_UNACKNOWLEDGED);
  buf[index++] = (value) ? 0x01 : 0x00;
  buf[index++] = is_new ? ++tid : tid;
  buf[index++] = transition_time;
  buf[index++] = delay_ms;

  UART_SendMeshMessageRequest(buf, sizeof(buf));
}

static void MeshInternal_SendLightLightnessSet(uint8_t  instance_idx,
                                               uint16_t value,
                                               uint8_t  transition_time,
                                               uint8_t  delay_ms,
                                               bool     is_new)
{
  uint8_t        buf[MESH_MESSAGE_LIGHT_LIGHTNESS_SET_LEN];
  size_t         index = 0;
  static uint8_t tid   = 0;

  buf[index++] = instance_idx;
  buf[index++] = 0x00;
  buf[index++] = lowByte(MESH_MESSAGE_LIGHT_LIGHTNESS_SET_UNACKNOWLEDGED);
  buf[index++] = highByte(MESH_MESSAGE_LIGHT_LIGHTNESS_SET_UNACKNOWLEDGED);
  buf[index++] = lowByte(value);
  buf[index++] = highByte(value);
  buf[index++] = is_new ? ++tid : tid;
  buf[index++] = transition_time;
  buf[index++] = delay_ms;

  UART_SendMeshMessageRequest(buf, sizeof(buf));
}

static void MeshInternal_SendGenericDeltaSet(uint8_t instance_idx,
                                             int32_t value,
                                             uint8_t transition_time,
                                             uint8_t delay_ms,
                                             bool    is_new)
{
  uint8_t        buf[MESH_MESSAGE_GENERIC_DELTA_SET_LEN];
  size_t         index = 0;
  static uint8_t tid   = 0;

  buf[index++] = instance_idx;
  buf[index++] = 0x00;
  buf[index++] = lowByte(MESH_MESSAGE_GENERIC_DELTA_SET_UNACKNOWLEDGED);
  buf[index++] = highByte(MESH_MESSAGE_GENERIC_DELTA_SET_UNACKNOWLEDGED);
  buf[index++] = ((value >> 0) & 0xFF);
  buf[index++] = ((value >> 8) & 0xFF);
  buf[index++] = ((value >> 16) & 0xFF);
  buf[index++] = ((value >> 24) & 0xFF);
  buf[index++] = is_new ? ++tid : tid;
  buf[index++] = transition_time;
  buf[index++] = delay_ms;

  UART_SendMeshMessageRequest(buf, sizeof(buf));
}

static void MeshInternal_SendLightLightnessControllerModeSet(uint8_t instance_idx, bool value)
{
  uint8_t buf[MESH_MESSAGE_LIGHT_LC_MODE_SET_LEN];
  size_t  index = 0;

  buf[index++] = instance_idx;
  buf[index++] = 0x00;
  buf[index++] = lowByte(MESH_MESSAGE_LIGHT_LC_MODE_SET_UNACKNOWLEDGED);
  buf[index++] = highByte(MESH_MESSAGE_LIGHT_LC_MODE_SET_UNACKNOWLEDGED);
  buf[index++] = (value) ? 0x01 : 0x00;

  UART_SendMeshMessageRequest(buf, sizeof(buf));
}

static uint8_t MeshInternal_ConvertFromMsToMeshFormatTransitionTime(uint32_t time_ms)
{
  if ((time_ms / MESH_NUMBER_OF_MS_IN_100_MS) < MESH_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE)
  {
    return (MESH_TRANSITION_TIME_STEP_RESOLUTION_100_MS | (time_ms / MESH_NUMBER_OF_MS_IN_100_MS));
  }
  else if ((time_ms / MESH_NUMBER_OF_MS_IN_1S) < MESH_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE)
  {
    return (MESH_TRANSITION_TIME_STEP_RESOLUTION_1_S | (time_ms / MESH_NUMBER_OF_MS_IN_1S));
  }
  else if ((time_ms / MESH_NUMBER_OF_MS_IN_10S)
           < MESH_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE)
  {
    return (MESH_TRANSITION_TIME_STEP_RESOLUTION_10_S | (time_ms / MESH_NUMBER_OF_MS_IN_10S));
  }
  else if ((time_ms / MESH_NUMBER_OF_MS_IN_10MIN)
           < MESH_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE)
  {
    return (MESH_TRANSITION_TIME_STEP_RESOLUTION_10_MIN | (time_ms / MESH_NUMBER_OF_MS_IN_10MIN));
  }
  else
  {
    return (MESH_TRANSITION_TIME_STEP_RESOLUTION_10_MIN
            | (MESH_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE
               & MESH_TRANSITION_TIME_NUMBER_OF_STEPS_MASK));
  }
}

/**********************************************
 * SENSOR SPECIFIC LOCAL FUNCTION DEFINITIONS *
 **********************************************/

static void MeshInternal_ProcessSensorStatus(uint8_t * p_payload, size_t len)
{
  if (len < 2)
  {
    DEBUG_INTERFACE.println("Received empty Sensor Status message");
    return;
  }

  uint16_t src_addr = ((uint16_t)p_payload[len-2]);
  src_addr         |= ((uint16_t)p_payload[len-1] << 8);

  if (len <= 2)
  {
    DEBUG_INTERFACE.printf("Received empty Sensor Status message from: %d\n", src_addr);
    return;
  }

  len -= 2;
  size_t index = 0;
  while (index < len)
  {
    DEBUG_INTERFACE.printf("ProcessSensorStatus index: %d\n", index);
    if (p_payload[index] & SS_FORMAT_MASK)
    {
      size_t   message_len = (p_payload[index++] & SS_LONG_LEN_MASK) >> SS_LONG_LEN_OFFSET;
      uint16_t property_id = ((uint16_t)p_payload[index++]);
      property_id         |= ((uint16_t)p_payload[index++] << 8);

      MeshInternal_ProcessSensorProperty(property_id, p_payload + index, message_len, src_addr);
      index += message_len;
    }
    else
    {
      size_t   message_len = (p_payload[index] & SS_SHORT_LEN_MASK) >> SS_SHORT_LEN_OFFSET;
      uint16_t property_id = (p_payload[index++] & SS_SHORT_PROPERTY_ID_LOW_MASK) >> SS_SHORT_PROPERTY_ID_LOW_OFFSET;
      property_id         |= ((uint16_t) p_payload[index++]) << SS_SHORT_PROPERTY_ID_HIGH_OFFSET;

      MeshInternal_ProcessSensorProperty(property_id, p_payload + index, message_len, src_addr);
      index += message_len;
    }
  }
}

static void MeshInternal_ProcessSensorProperty(uint16_t  property_id,
                                               uint8_t * p_payload,
                                               size_t    len,
                                               uint16_t  src_addr)
{
  switch (property_id)
  {
    case PRESENCE_DETECTED_PROPERTY_ID:
    {
      MeshInternal_ProcessPresenceDetected(p_payload, len, src_addr);
      break;
    }
    case PRESENT_AMBIENT_LIGHT_LEVEL:
    {
      MeshInternal_ProcessPresentAmbientLightLevel(p_payload, len, src_addr);
      break;
    }
    default:
    {
      DEBUG_INTERFACE.println("Invalid property id");
    }
  }
}

static void MeshInternal_ProcessPresenceDetected(uint8_t * p_payload, size_t len, uint16_t src_addr)
{
  if (len != 0)
  {
    DEBUG_INTERFACE.println("Invalid Length Sensor Status message");
    return;
  }

  size_t  index = 0;
  uint8_t value = p_payload[index++];

  if (value <= 1)
  {
    ProcessPresenceDetected(src_addr, (value == 1));
  }
  else
  {
    DEBUG_INTERFACE.printf("Decoded Sensor Status message from 0x%04X, PRESENCE DETECTED with prohibited value\n", src_addr);
  }
}

static void MeshInternal_ProcessPresentAmbientLightLevel(uint8_t * p_payload,
                                                         size_t    len,
                                                         uint16_t  src_addr)
{
  if (len != 2)
  {
    DEBUG_INTERFACE.println("Invalid Length Sensor Status message");
    return;
  }

  size_t   index      = 0;
  uint32_t value_clux = ((uint32_t)p_payload[index++]);
  value_clux         |= ((uint32_t)p_payload[index++] << 8);
  value_clux         |= ((uint32_t)p_payload[index++] << 16);

  float value = ((float)value_clux) / 100;
  ProcessPresentAmbientLightLevel(src_addr, value);
}