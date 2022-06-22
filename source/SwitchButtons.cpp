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

#include "SwitchButtons.h"

#include "Arduino.h"
#include "Config.h"
#include "Log.h"
#include "Mesh.h"


#define FSM_INFO(x) LOG_INFO("Button %s %s %d\n", (button_fsm->button_type ? "ON" : "OFF"), x, button_fsm->action);

enum MachineStates
{
    FSM_IDLE = 0,
    FSM_PRESS,
    FSM_LONG_PRESS,
};

enum MachineActions
{
    NO_ACTION = 0,
    ACTION_PRESSED,
    ACTION_RELEASED
};

enum ButtonType
{
    BUTTON_OFF = 0,
    BUTTON_ON  = 1
};


#define SEQUENCE_A_TIMEOUT 400
#define SEQUENCE_C_TIMEOUT 100
#define SEQUENCE_B_TIMEOUT 250

#define BUTTON_DEBOUNCE_TIME_MS 20 /**< Defines buttons debounce time in milliseconds. */


#define SEQUENCE_A_NUMBER_OF_REPEATS 3 /**< Defines number of repeats for Generic OnOff message */
#define SEQUENCE_B_NUMBER_OF_REPEATS 3 /**< Defines number of repeats for first Generic Delta message  */
#define SEQUENCE_C_NUMBER_OF_REPEATS 0 /**< Defines number of repeats for Generic Deltas during dimming  */
#define SEQUENCE_D_NUMBER_OF_REPEATS 4 /**< Defines number of repeats for last Generic Delta message */

#define ON_OFF_TRANSITION_TIME_MS 1000 /**< Defines transition time */
#define ON_OFF_DELAY_TIME_MS 0         /**< Defines message delay in milliseconds */
#define REPEATS_INTERVAL_MS 50         /**< Defines interval between repeats in milliseconds */

#define DIMMING_STEP_VALUE 0xA00     /**< Defines Generic Delta minimal step on button long press */
#define DELTA_TRANSITION_TIME_MS 200 /**< Defines transition time */
#define DELTA_DELAY_TIME_MS 0        /**< Defines delay time */


#define GENERIC_OFF 0x00 /**< Defines Generic OnOff Off payload */
#define GENERIC_ON 0x01  /**< Defines Generic OnOff On payload */

#define MAX_DELTA ((0x10000) / DIMMING_STEP_VALUE)

struct ButtonInstance
{
    uint8_t             pin_number;
    enum ButtonType     button_type;
    uint32_t            event_time;
    uint32_t            timeout;
    enum MachineStates  state;
    int16_t             delta_value_on_press;
    enum MachineActions action;
    uint8_t             instance_idx;
};

struct ButtonInstance Button1;
struct ButtonInstance Button2;
struct ButtonInstance Button3;
struct ButtonInstance Button4;

static void CheckButtonState(struct ButtonInstance *button_fsm, const struct ButtonInstance *paired_button_fsm);
static void ClearButtonStates(ButtonInstance *button_fsm);

static void InterruptOn1PBPress(void);
static void InterruptOff1PBPress(void);
static void InterruptOn2PBPress(void);
static void InterruptOff2PBPress(void);
static void OnButtonInterrupt(struct ButtonInstance *button);
static void SendSequenceDGenericDelta(uint16_t instance_Index, uint32_t delta_value);

void SetupFSM(uint8_t light_lc_client_instance_index, uint8_t light_ctl_client_instance_index)
{
    Button1.instance_idx = light_lc_client_instance_index;
    Button1.pin_number   = PIN_SW_1;
    ClearButtonStates(&Button4);
    Button1.state                = FSM_IDLE;
    Button1.delta_value_on_press = 0;
    Button1.button_type          = BUTTON_ON;
    pinMode(Button1.pin_number, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Button1.pin_number), InterruptOn1PBPress, CHANGE);

    Button2.instance_idx = light_lc_client_instance_index;
    Button2.pin_number   = PIN_SW_2;
    ClearButtonStates(&Button4);
    Button2.state                = FSM_IDLE;
    Button2.delta_value_on_press = 0;
    Button2.button_type          = BUTTON_OFF;
    pinMode(Button2.pin_number, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Button2.pin_number), InterruptOff1PBPress, CHANGE);

    Button3.instance_idx = light_ctl_client_instance_index;
    Button3.pin_number   = PIN_SW_3;
    ClearButtonStates(&Button4);
    Button3.state                = FSM_IDLE;
    Button3.delta_value_on_press = 0;
    Button3.button_type          = BUTTON_ON;
    pinMode(Button3.pin_number, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Button3.pin_number), InterruptOn2PBPress, CHANGE);

    Button4.instance_idx = light_ctl_client_instance_index;
    Button4.pin_number   = PIN_SW_4;
    ClearButtonStates(&Button4);
    Button4.button_type = BUTTON_OFF;
    pinMode(Button4.pin_number, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Button4.pin_number), InterruptOff2PBPress, CHANGE);
}

void LoopButtons(void)
{
    CheckButtonState(&Button1, &Button2);
    CheckButtonState(&Button2, &Button1);
    CheckButtonState(&Button3, &Button4);
    CheckButtonState(&Button4, &Button3);
}

static void InterruptOn1PBPress(void)
{
    static elapsedMillis time_since_last_interrupt;
    if (time_since_last_interrupt > BUTTON_DEBOUNCE_TIME_MS)
    {
        OnButtonInterrupt(&Button1);
        time_since_last_interrupt = 0;
    }
}

static void InterruptOff1PBPress(void)
{
    static elapsedMillis time_since_last_interrupt;
    if (time_since_last_interrupt > BUTTON_DEBOUNCE_TIME_MS)
    {
        OnButtonInterrupt(&Button2);
        time_since_last_interrupt = 0;
    }
}

static void InterruptOn2PBPress(void)
{
    static elapsedMillis time_since_last_interrupt;
    if (time_since_last_interrupt > BUTTON_DEBOUNCE_TIME_MS)
    {
        OnButtonInterrupt(&Button3);
        time_since_last_interrupt = 0;
    }
}

static void InterruptOff2PBPress(void)
{
    static elapsedMillis time_since_last_interrupt;
    if (time_since_last_interrupt > BUTTON_DEBOUNCE_TIME_MS)
    {
        OnButtonInterrupt(&Button4);
        time_since_last_interrupt = 0;
    }
}

static void OnButtonInterrupt(struct ButtonInstance *button)
{
    if (digitalRead(button->pin_number) == LOW)
    {
        button->action = ACTION_PRESSED;
    }
    else
    {
        button->action = ACTION_RELEASED;
    }
}

static void CheckButtonState(struct ButtonInstance *button_fsm, const struct ButtonInstance *paired_button_fsm)
{
    if (paired_button_fsm->state != FSM_IDLE)
    {
        return;
    }

    switch (button_fsm->state)
    {
        case FSM_IDLE:
        {
            if (button_fsm->action == ACTION_PRESSED)
            {
                FSM_INFO("Button pressed");
                button_fsm->state      = FSM_PRESS;
                button_fsm->action     = NO_ACTION;
                button_fsm->event_time = millis();
                button_fsm->timeout    = SEQUENCE_A_TIMEOUT;
            }
            else if (button_fsm->action == ACTION_RELEASED)
            {
                ClearButtonStates(button_fsm);
            }
        }
        break;

        case FSM_PRESS:
        {
            if (button_fsm->action == ACTION_RELEASED)
            {
                FSM_INFO("Short press - turn on/off lightness");
                ClearButtonStates(button_fsm);

                Mesh_SendGenericOnOffSetWithRepeatsInterval(button_fsm->instance_idx,
                                                            (button_fsm->button_type ? GENERIC_ON : GENERIC_OFF),
                                                            ON_OFF_TRANSITION_TIME_MS,
                                                            ON_OFF_DELAY_TIME_MS,
                                                            SEQUENCE_A_NUMBER_OF_REPEATS,
                                                            REPEATS_INTERVAL_MS,
                                                            true);
                return;
            }

            if ((millis() - button_fsm->event_time) > button_fsm->timeout)
            {
                FSM_INFO("Long Press - dim lightness");
                button_fsm->state      = FSM_LONG_PRESS;
                button_fsm->event_time = millis();
                button_fsm->timeout    = SEQUENCE_B_TIMEOUT;

                button_fsm->button_type == BUTTON_ON ? button_fsm->delta_value_on_press++ : button_fsm->delta_value_on_press--;

                Mesh_SendGenericDeltaSetWithRepeatsInterval(button_fsm->instance_idx,
                                                            DIMMING_STEP_VALUE * button_fsm->delta_value_on_press,
                                                            DELTA_TRANSITION_TIME_MS,
                                                            DELTA_DELAY_TIME_MS,
                                                            SEQUENCE_B_NUMBER_OF_REPEATS,
                                                            REPEATS_INTERVAL_MS,
                                                            true);
            }
        }
        break;

        case FSM_LONG_PRESS:
        {
            if (millis() - button_fsm->event_time <= button_fsm->timeout)
            {
                return;
            }

            if (button_fsm->action == ACTION_RELEASED)
            {
                FSM_INFO("Released long press");
                int16_t delta_end_correction = DIMMING_STEP_VALUE * (millis() - button_fsm->event_time) / button_fsm->timeout;
                if (button_fsm->button_type == BUTTON_OFF)
                {
                    delta_end_correction = -delta_end_correction;
                }

                SendSequenceDGenericDelta(button_fsm->instance_idx, DIMMING_STEP_VALUE * button_fsm->delta_value_on_press + delta_end_correction);
                ClearButtonStates(button_fsm);

                return;
            }

            FSM_INFO("Long Press dimming");
            button_fsm->button_type == BUTTON_ON ? button_fsm->delta_value_on_press++ : button_fsm->delta_value_on_press--;

            if ((button_fsm->delta_value_on_press == MAX_DELTA) || (button_fsm->delta_value_on_press == -MAX_DELTA))
            {
                SendSequenceDGenericDelta(button_fsm->instance_idx, DIMMING_STEP_VALUE * button_fsm->delta_value_on_press);
                ClearButtonStates(button_fsm);
            }
            else
            {
                button_fsm->event_time = millis();
                button_fsm->timeout    = SEQUENCE_C_TIMEOUT;
                Mesh_SendGenericDeltaSet(button_fsm->instance_idx,
                                         DIMMING_STEP_VALUE * button_fsm->delta_value_on_press,
                                         DELTA_TRANSITION_TIME_MS,
                                         DELTA_DELAY_TIME_MS,
                                         SEQUENCE_C_NUMBER_OF_REPEATS,
                                         false);
            }
        }
        break;
    }
}

static void SendSequenceDGenericDelta(uint16_t instance_Index, uint32_t delta_value)
{
    for (uint8_t i = 0; i < SEQUENCE_D_NUMBER_OF_REPEATS; i++)
    {
        Mesh_SendGenericDeltaSetWithDispatchTime(instance_Index, delta_value, DELTA_TRANSITION_TIME_MS, DELTA_DELAY_TIME_MS, 50 * i, false);
    }
}


static void ClearButtonStates(ButtonInstance *button_fsm)
{
    button_fsm->state                = FSM_IDLE;
    button_fsm->action               = NO_ACTION;
    button_fsm->event_time           = 0;
    button_fsm->timeout              = 0;
    button_fsm->delta_value_on_press = 0;
}
