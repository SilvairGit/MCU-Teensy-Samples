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


#include "Flasher.h"

#include <kinetis.h>
#include <stdint.h>


#define FLASH_END_ADDR 0x10000u                   /**< Pointer to end of flash. */
#define FLASH_SECTOR_SIZE 0x400u                  /**< Flash sector size */
#define FLASH_EEPROM_SIZE (2 * FLASH_SECTOR_SIZE) /**< Size of space reserved for dummy eeprom */
#define FLASH_CONFIG_FIELD_ADDR 0x40u             /**< Config field address */
#define FLASH_CONFIG_FIELD_VAL 0xFFFFFFFEu        /**< Config field desirable value */
#define FLASH_ERASED_WORD_VAL 0xFFFFFFFFu         /**< Erased word value */
#define FLASH_WRITE_WORD_CMD 0x06                 /**< Flash write word command code */
#define FLASH_ERASE_SECTOR_CMD 0x09               /**< Flash sector erase command code */
#define CPU_RESTART_ADDR ((uint32_t *)0xE000ED0C) /**< CPU restart register address */
#define CPU_RESTART_VAL 0x5FA0004                 /**< CPU restart register value  */

/**< Data memory barrier instruction definition. */
#define _DMB()                 \
    do                         \
    {                          \
        __asm volatile("dmb"); \
    } while (0)

extern unsigned long _etext; /**< End of .text section label */
extern unsigned long _sdata; /**< Start of .data section label */
extern unsigned long _edata; /**< Ennd of .data section label */


/*
 *  Save word to flash not in the EEPROM area.
 *
 *  @param address       Destination pointer
 *  @param word_value    Value to flash
 *  @param reenable_irq  If true will leave IRQ enabled, disabled if false.
 *  @return              Flasher return code
 */
RAMFUNC static int Flasher_FlashWordNotEeprom(uint32_t address, uint32_t word_value, bool reenable_irq);

/*
 *  Erase sector.
 *
 *  @param address       Pointer to first byte in sector to be erased.
 *  @param unsafe        If false will prevent from erasing FLASH CONFIG FIELD
 *  @param reenable_irq  If true will leave IRQ enabled, disabled if false.
 *  @return              Flasher return code
 */
RAMFUNC static int Flasher_SectorErase(uint32_t address, bool unsafe, bool reenable_irq);


RAMFUNC int Flasher_UpdateFirmware(uint32_t num_of_words)
{
    uint32_t src = Flasher_GetSpaceAddr();
    uint32_t dst = 0;

    __disable_irq();

    for (uint32_t i = 0; i <= num_of_words; i++)
    {
        uint32_t source      = src + i * 4;
        uint32_t destination = dst + i * 4;

        if (destination % FLASH_SECTOR_SIZE == 0)
        {
            Flasher_SectorErase(destination, true, false);
            if (destination + FLASH_SECTOR_SIZE > FLASH_CONFIG_FIELD_ADDR && destination <= FLASH_CONFIG_FIELD_ADDR)
            {
                Flasher_FlashWordNotEeprom(FLASH_CONFIG_FIELD_ADDR, FLASH_CONFIG_FIELD_VAL, false);
            }
        }

        Flasher_FlashWordNotEeprom(destination, *(volatile uint32_t *)source, false);
    }

    *CPU_RESTART_ADDR = CPU_RESTART_VAL;

    return FLASHER_SUCCESS;
}

RAMFUNC int Flasher_FlashWord(uint32_t address, uint32_t word_value, bool reenable_irq)
{
    if ((uint32_t)address % sizeof(uint32_t) != 0)
    {
        return FLASHER_ERROR_ALIGNMENT;
    }

    if (address == FLASH_CONFIG_FIELD_ADDR)
    {
        word_value = FLASH_CONFIG_FIELD_VAL;
        return FLASHER_SUCCESS;
    }

    if (*(volatile uint32_t *)address == word_value)
    {
        return FLASHER_SUCCESS;
    }

    while ((FTFL_FSTAT & FTFL_FSTAT_CCIF) == 0)
    {
    };
    FTFL_FSTAT = FTFL_FSTAT_ACCERR | FTFL_FSTAT_FPVIOL;

    *(uint32_t *)&FTFL_FCCOB3 = address;
    *(uint32_t *)&FTFL_FCCOB7 = word_value;
    FTFL_FCCOB0               = FLASH_WRITE_WORD_CMD;

    __disable_irq();

    FTFL_FSTAT = FTFL_FSTAT_CCIF;
    while ((FTFL_FSTAT & FTFL_FSTAT_CCIF) == 0)
    {
    };
    MCM_PLACR |= MCM_PLACR_CFCC;

    _DMB();

    if (reenable_irq)
        __enable_irq();

    if (*(volatile uint32_t *)address != word_value)
    {
        return FLASHER_ERROR_VERIFY;
    }
    if (FTFL_FSTAT & FTFL_FSTAT_RDCOLERR)
    {
        return FLASHER_ERROR_COLLISION;
    }
    if (FTFL_FSTAT & FTFL_FSTAT_ACCERR)
    {
        return FLASHER_ERROR_ACCESS;
    }
    if (FTFL_FSTAT & FTFL_FSTAT_FPVIOL)
    {
        return FLASHER_ERROR_PROTECTION;
    }
    if (FTFL_FSTAT & FTFL_FSTAT_MGSTAT0)
    {
        return FLASHER_ERROR_CONTROLLER;
    }

    return FLASHER_SUCCESS;
}

uint32_t Flasher_GetSpaceAddr(void)
{
    uint32_t first_free_addr = (uint32_t)&_etext + (uint32_t)&_edata - (uint32_t)&_sdata;
    return (FLASH_SECTOR_SIZE * ((first_free_addr / FLASH_SECTOR_SIZE) + 1));
}

size_t Flasher_GetSpaceSize(void)
{
    return FLASH_END_ADDR - Flasher_GetSpaceAddr() - FLASH_EEPROM_SIZE;
}

int Flasher_EraseSpace(void)
{
    for (uint32_t p_sector = Flasher_GetSpaceAddr(); p_sector < FLASH_END_ADDR; p_sector += FLASH_SECTOR_SIZE)
    {
        int ret_val = Flasher_SectorErase(p_sector, false, true);

        if (ret_val != FLASHER_SUCCESS)
        {
            return ret_val;
        }
    }

    return FLASHER_SUCCESS;
}

int Flasher_SaveMemoryToFlash(uint32_t address, const uint32_t *src, uint32_t num_of_words)
{
    if (address % sizeof(uint32_t) != 0)
    {
        return FLASHER_ERROR_ALIGNMENT;
    }

    for (uint32_t i = 0; i < num_of_words; i++)
    {
        uint32_t word_to_flash = *(src + i);
        int      ret_val       = Flasher_FlashWordNotEeprom(address + i * 4, word_to_flash, true);

        if (ret_val != FLASHER_SUCCESS)
        {
            return ret_val;
        }
    }

    return FLASHER_SUCCESS;
}


RAMFUNC int Flasher_FlashWordNotEeprom(uint32_t address, uint32_t word_value, bool reenable_irq)
{
    if (address >= FLASH_END_ADDR - FLASH_EEPROM_SIZE)
    {
        return FLASHER_ERROR_RANGE;
    }

    if (*(volatile uint32_t *)address != FLASH_ERASED_WORD_VAL)
    {
        return FLASHER_ERROR_NOT_ERASED;
    }

    return Flasher_FlashWord(address, word_value, reenable_irq);
}

RAMFUNC int Flasher_SectorErase(uint32_t address, bool unsafe, bool reenable_irq)
{
    if (address >= FLASH_END_ADDR - FLASH_EEPROM_SIZE)
    {
        return FLASHER_ERROR_RANGE;
    }

    if (address % FLASH_SECTOR_SIZE != 0)
    {
        return FLASHER_ERROR_ALIGNMENT;
    }

    if (address <= FLASH_CONFIG_FIELD_ADDR && address + FLASH_SECTOR_SIZE > FLASH_CONFIG_FIELD_ADDR && !unsafe)
    {
        return FLASHER_ERROR_UNSAFE;
    }

    while ((FTFL_FSTAT & FTFL_FSTAT_CCIF) == 0)
    {
    };

    FTFL_FSTAT = FTFL_FSTAT_ACCERR | FTFL_FSTAT_FPVIOL;

    *(uint32_t *)&FTFL_FCCOB3 = address;
    FTFL_FCCOB0               = FLASH_ERASE_SECTOR_CMD;

    __disable_irq();

    FTFL_FSTAT = FTFL_FSTAT_CCIF;
    while ((FTFL_FSTAT & FTFL_FSTAT_CCIF) == 0)
    {
    }
    MCM_PLACR |= MCM_PLACR_CFCC;

    _DMB();

    if (reenable_irq)
        __enable_irq();

    if (FTFL_FSTAT & FTFL_FSTAT_RDCOLERR)
    {
        return FLASHER_ERROR_COLLISION;
    }
    if (FTFL_FSTAT & FTFL_FSTAT_ACCERR)
    {
        return FLASHER_ERROR_ACCESS;
    }
    if (FTFL_FSTAT & FTFL_FSTAT_FPVIOL)
    {
        return FLASHER_ERROR_PROTECTION;
    }
    if (FTFL_FSTAT & FTFL_FSTAT_MGSTAT0)
    {
        return FLASHER_ERROR_CONTROLLER;
    }

    return FLASHER_SUCCESS;
}
