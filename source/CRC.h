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

#ifndef CRC_H_
#define CRC_H_


#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>


#define CRC16_INIT_VAL 0xFFFFu     /**< CRC16 init value */
#define CRC32_INIT_VAL 0xFFFFFFFFu /**< CRC32 init value */


/*
 *  Calculate CRC16
 *
 *  @param * data       Pointer to data
 *  @param len          Data len
 *  @param init_val     CRC init val
 *  @return             Calculated CRC
 */
uint16_t CalcCRC16(uint8_t *data, size_t len, uint16_t init_val);

/*
 *  Calculate CRC16 MODBUS
 *
 *  @param * data       Pointer to data
 *  @param len          Data len
 *  @param init_val     CRC init val
 *  @return             Calculated CRC
 */
uint16_t CalcCRC16_Modbus(uint8_t *data, size_t len, uint16_t init_val);

/*
 *  Calculate CRC32
 *
 *  @param * data       Pointer to data
 *  @param len          Data len
 *  @param init_val     CRC init val
 *  @return             Calculated CRC
 */
uint32_t CalcCRC32(uint8_t *data, size_t len, uint32_t init_val);

/*
 *  Calculate CRC32
 *
 *  @param * data       Pointer to data
 *  @param len          Data len
 *  @param * sha256     [out] calculated SHA256
 */
void CalcSHA256(uint8_t *data, size_t len, uint8_t *sha256);

#endif    // CRC_H_
