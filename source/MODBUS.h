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

#ifndef MODBUS_H_
#define MODBUS_H_


#include <stddef.h>
#include <stdint.h>


/**
 * MODBUS error codes
 */
#define MODBUS_ERROR_ILLEGAL_FUNCTION 0x01u
#define MODBUS_ERROR_ILLEGAL_DATA_ADDRESS 0x02u
#define MODBUS_ERROR_ILLEGAL_DATA_VALUE 0x03u
#define MODBUS_ERROR_SLAVE_DEVICE_FAILURE 0x04u
#define MODBUS_ERROR_ACKNOWLEDGE 0x05u
#define MODBUS_ERROR_SLAVE_DEVICE_BUSY 0x06u
#define MODBUS_ERROR_NEGATIVE_ACKNOWLEDGE 0x07u
#define MODBUS_ERROR_MEMORY_PARITY_ERROR 0x08u


/**
 * Process incoming MODBUS data
 */
void MODBUS_ProcessIncoming(void);

/**
 * Clear MODBUS receiving buffer
 */
void MODBUS_ClearBuffer(void);

/**
 * Send Read Holding Registers MODBUS command
 *
 * @param slave_address     Destination address
 * @param starting_address  Read starting address
 * @param num_of_points     Number of register to be read
 */
void MODBUS_SendReadHoldingRegisters(uint8_t slave_address, uint16_t starting_address, uint16_t num_of_points);

/**
 * Send Read Input Registers MODBUS command
 *
 * @param slave_address     Destination address
 * @param starting_address  Read starting address
 * @param num_of_points     Number of register to be read
 */
void MODBUS_SendReadInputRegisters(uint8_t slave_address, uint16_t starting_address, uint16_t num_of_points);

/**
 * Send Preset Single Register MODBUS command
 *
 * @param slave_address     Destination address
 * @param register_address  Register address
 * @param preset_data       Data to be written
 */
void MODBUS_SendPresetSingleRegister(uint8_t slave_address, uint16_t register_address, uint16_t preset_data);

/**
 * Send Preset Multiple Registers MODBUS command
 *
 * @param slave_address     Destination address
 * @param starting_address  Write start address
 * @param register_count    Number of registers to be written
 * @param p_registers       New registers values
 */
void MODBUS_SendPresetMultipleRegisters(uint8_t slave_address, uint16_t starting_address, uint8_t register_count, uint16_t *p_registers);

/**
 * Incoming Read Holding Register MODBUS command handler
 *
 * @param slave_address     Source node address
 * @param data_len          Number of registers read
 * @param p_data            Registers values
 */
extern void MODBUS_ProcessReadHoldingRegisters(uint8_t slave_address, size_t data_len, uint16_t *p_data);

/**
 * Incoming Read Input Register MODBUS command handler
 *
 * @param slave_address     Source node address
 * @param data_len          Number of registers read
 * @param p_data            Registers values
 */
extern void MODBUS_ProcessReadInputRegisters(uint8_t slave_address, size_t data_len, uint16_t *p_data);

/**
 * Incoming Preset Single Register MODBUS command handler
 *
 * @param slave_address     Source node address
 * @param register_address  Register address
 * @param preset_data       Written data
 */
extern void MODBUS_ProcessReadPresetSingleRegister(uint8_t slave_address, uint16_t register_address, uint16_t preset_data);

/**
 * Incoming Preset Multiple Registers MODBUS command handler
 *
 * @param slave_address     Source node address
 * @param starting_address  Write start address
 * @param num_of_registers  Number of registers written
 */
extern void MODBUS_ProcessReadPresetMultipleRegisters(uint8_t slave_address, uint16_t starting_address, uint16_t num_of_registers);

/**
 * Incoming MODBUS Exception handler.
 *
 * @param slave_address             Source node address
 * @param original_function_code    Original function code
 * @param error_code                Error code
 */
extern void MODBUS_ProcessException(uint8_t slave_address, uint8_t original_function_code, uint8_t error_code);

#endif    // MODBUS_H_
