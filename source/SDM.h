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

#ifndef SDM_H_
#define SDM_H_


#include <stddef.h>
#include <stdint.h>


/**
 * SDM configuration values definitons
 */
#define SDM_BAUD_2400 0
#define SDM_BAUD_4800 1
#define SDM_BAUD_9600 2
#define SDM_BAUD_1200 5

#define SDM_STOP_1_PARITY_NO 0
#define SDM_STOP_1_PARITY_EVEN 1
#define SDM_STOP_1_PARITY_ODD 2
#define SDM_STOP_2_PARITY_NO 3

#define SDM_PULSE_60_MS 60
#define SDM_PULSE_100_MS 100
#define SDM_PULSE_200_MS 200

#define SDM_IMPORT_ACTIVE_ENERGY 1
#define SDM_IMPORT_EXPORT_ACTIVE_ENERGY 2
#define SDM_EXPORT_ACTIVE_ENERGY 4
#define SDM_IMPORT_REACTIVE_ENERGY 5
#define SDM_IMPORT_EXPORT_REACTIVE_ENERGY 6
#define SDM_EXPORT_REACTIVE_ENERGY 8

#define SDM_KWH_0_001 0
#define SDM_KWH_0_01 1
#define SDM_KWH_0_1 2
#define SDM_KWH_1 3

#define SDM_IMPORT 1
#define SDM_IMPORT_PLUS_EXPORT 2
#define SDM_IMPORT_MINUS_EXPORT 3


typedef struct SDM_State_Tag
{
    float voltage;
    float current;
    float active_power;
    float apparent_power;
    float reactive_power;
    float power_factor;
    float frequency;
    float import_active_energy;
    float export_active_energy;
    float import_reactive_energy;
    float export_reactive_energy;
    float total_system_power_demand;
    float max_total_system_power_demand;
    float import_system_power_demand;
    float max_import_system_power_demand;
    float export_system_power_demand;
    float max_export_system_power_demand;
    float current_demand;
    float max_current_demand;
    float total_active_energy;
    float total_reactive_energy;

    uint8_t  relay_pulse_width;
    uint8_t  network_parity_stop;
    uint8_t  meter_id;
    uint8_t  baud_rate;
    uint8_t  ct_primary_current;
    uint8_t  pulse1_output_mode;
    uint16_t time_of_scroll_display;
    uint16_t pulse1_output;
    uint16_t measurement_mode;
} SDM_State_T;


/**
 * SDM Loop, call this inside Arduino main loop()
 */
void LoopSDM(void);

/**
 * SDM Setup, call this from inside Arduino setup()
 */
void SetupSDM(void);

/**
 * Get SDM state.
 *
 * @return  Pointer to SDM state. Returns NULL if SDM120 is not connected.
 */
const SDM_State_T *SDM_GetState(void);

/**
 * Set SDM property
 *
 * @param relay_pulse_width
 */
void SDM_SetRelayPulseWidth(uint8_t relay_pulse_width);

/**
 * Set SDM property
 *
 * @param network_parity_stop
 */
void SDM_SetNetworkParityStop(uint8_t network_parity_stop);

/**
 * Set SDM property
 *
 * @param meter_id
 */
void SDM_SetMeterID(uint8_t meter_id);

/**
 * Set SDM property
 *
 * @param baud_rate
 */
void SDM_SetBaudRate(uint8_t baud_rate);

/**
 * Set SDM property
 *
 * @param ct_primary_current
 */
void SDM_SetCTPrimaryCurrent(uint16_t ct_primary_current);

/**
 * Set SDM property
 *
 * @param pulse1_output_mode
 */
void SDM_SetPulse1OutputMode(uint8_t pulse1_output_mode);

/**
 * Set SDM property
 *
 * @param time_of_scroll_display
 */
void SDM_SetTimeOfScrollDisplay(uint16_t time_of_scroll_display);

/**
 * Set SDM property
 *
 * @param pulse1_output
 */
void SDM_SetPulse1Output(uint16_t pulse1_output);

/**
 * Set SDM property
 *
 * @param measurement_mode
 */
void SDM_SetMeasurementMode(uint16_t measurement_mode);

#endif    // SDM_H_
