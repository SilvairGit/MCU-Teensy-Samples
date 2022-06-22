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

#ifndef MCU_LIGHTNESS_SERVER_H
#define MCU_LIGHTNESS_SERVER_H


#include <stdint.h>


#define LIGHTNESS_MIN 0               /**< Defines lower range of light lightness */
#define LIGHTNESS_MAX UINT16_MAX      /**< Defines upper range of light lightness */
#define LIGHTNESS_TEMP_MIN 0x0320     /**< Defines lower range of light temperature */
#define LIGHTNESS_TEMP_MAX 0x4E20     /**< Defines upper range of light temperature */
#define LIGHTNESS_TEMP_UNKNOWN 0xFFFF /**< Defines unknown  light temperature value */

#define LIGHT_CTL_TEMP_RANGE_MIN 2700 /**< Min allowed color temperature of white light in Kelvin. */
#define LIGHT_CTL_TEMP_RANGE_MAX 6500 /**< Max allowed color temperature of white light in Kelvin. */

/*
 *  Set Lightness Server instance index
 *
 *  @param idx  Lightness value
 */
void SetLightnessServerIdx(uint8_t idx);

/*
 *  Set Lightness CTL support
 *
 *  @param support  True to enable, false to disable
 */
void SetLightCTLSupport(bool support);

/*
 *  Get Lightness Server instance index
 *
 *  @return     Lightness value
 */
uint8_t GetLightnessServerIdx(void);

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
 *  Setup light lightness server hardware
 */
void SetupLightnessServer(void);

/*
 *  Lightness server main function, should be called in Arduino main loop
 */
void LoopLightnessServer(void);

/*
 *  Enable startup sequence
 */
void EnableStartupSequence(void);

/*
 *  Synchronize Lightness with Modem
 */
void SynchronizeLightness(void);

/*
 *  Indicate attention using lightness output
 *
 *  @param attention_state     Attention state
 *  @param led_state           Attention led state
 */
void IndicateAttentionLightness(bool attention_state, bool led_state);

#endif    // MCU_LIGHTNESS_SERVER_H
