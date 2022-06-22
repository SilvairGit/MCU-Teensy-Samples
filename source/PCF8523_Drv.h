#ifndef PCF8523_DRV_H_
#define PCF8523_DRV_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PCF8523_ADDRESS 0x68

#define PCF8523_CONTROL_1 0x00
#define PCF8523_CONTROL_2 0x01
#define PCF8523_CONTROL_3 0x02
#define PCF8523_SECONDS 0x03
#define PCF8523_MINUTES 0x04
#define PCF8523_HOURS 0x05
#define PCF8523_DAYS 0x06
#define PCF8523_WEEKDAYS 0x07
#define PCF8523_MONTHS 0x08
#define PCF8523_YEARS 0x09
#define PCF8523_MINUTE_ALARM 0x0A
#define PCF8523_HOUR_ALARM 0x0B
#define PCF8523_DAY_ALARM 0x0C
#define PCF8523_WEEKDAY_ALARM 0x0D
#define PCF8523_OFFSET 0x0E
#define PCF8523_TMR_CLKOUT_CTRL 0x0F
#define PCF8523_TMR_A_FREQ_CTRL 0x10
#define PCF8523_TMR_A_REG 0x11
#define PCF8523_TMR_B_FREQ_CTRL 0x12
#define PCF8523_TMR_B_REG 0x13

#define PCF8523_CONTROL_1_CAP_SEL_BIT 7
#define PCF8523_CONTROL_1_T_BIT 6
#define PCF8523_CONTROL_1_STOP_BIT 5
#define PCF8523_CONTROL_1_SR_BIT 4
#define PCF8523_CONTROL_1_1224_BIT 3
#define PCF8523_CONTROL_1_SIE_BIT 2
#define PCF8523_CONTROL_1_AIE_BIT 1
#define PCF8523_CONTROL_1CIE_BIT 0

#define PCF8523_CONTROL_2_WTAF_BIT 7
#define PCF8523_CONTROL_2_CTAF_BIT 6
#define PCF8523_CONTROL_2_CTBF_BIT 5
#define PCF8523_CONTROL_2_SF_BIT 4
#define PCF8523_CONTROL_2_AF_BIT 3
#define PCF8523_CONTROL_2_WTAIE_BIT 2
#define PCF8523_CONTROL_2_CTAIE_BIT 1
#define PCF8523_CONTROL_2_CTBIE_BIT 0

#define PCF8523_CONTROL_3_PM2_BIT 7
#define PCF8523_CONTROL_3_PM1_BIT 6
#define PCF8523_CONTROL_3_PM0_BIT 5
#define PCF8523_CONTROL_3_BSF_BIT 3
#define PCF8523_CONTROL_3_BLF_BIT 2
#define PCF8523_CONTROL_3_BSIE_BIT 1
#define PCF8523_CONTROL_3_BLIE_BIT 0

#define PCF8523_SECONDS_OS_BIT 7
#define PCF8523_SECONDS_10_BIT 6
#define PCF8523_SECONDS_10_LENGTH 3
#define PCF8523_SECONDS_1_BIT 3
#define PCF8523_SECONDS_1_LENGTH 4

#define PCF8523_MINUTES_10_BIT 6
#define PCF8523_MINUTES_10_LENGTH 3
#define PCF8523_MINUTES_1_BIT 3
#define PCF8523_MINUTES_1_LENGTH 4

#define PCF8523_HOURS_MODE_BIT 3    // 0 = 24-hour mode, 1 = 12-hour mode
#define PCF8523_HOURS_AMPM_BIT 5    // 2nd HOURS_10 bit if in 24-hour mode
#define PCF8523_HOURS_10_BIT 4
#define PCF8523_HOURS_1_BIT 3
#define PCF8523_HOURS_1_LENGTH 4

#define PCF8523_WEEKDAYS_BIT 2
#define PCF8523_WEEKDAYS_LENGTH 3

#define PCF8523_DAYS_10_BIT 5
#define PCF8523_DAYS_10_LENGTH 2
#define PCF8523_DAYS_1_BIT 3
#define PCF8523_DAYS_1_LENGTH 4

#define PCF8523_MONTH_10_BIT 4
#define PCF8523_MONTH_1_BIT 3
#define PCF8523_MONTH_1_LENGTH 4

#define PCF8523_YEAR_10H_BIT 7
#define PCF8523_YEAR_10H_LENGTH 4
#define PCF8523_YEAR_1H_BIT 3
#define PCF8523_YEAR_1H_LENGTH 4

#define PCF8523_TMR_CLKOUT_CTRL_TAM_BIT 7
#define PCF8523_TMR_CLKOUT_CTRL_TBM_BIT 6
#define PCF8523_TMR_CLKOUT_CTRL_COF2_BIT 5
#define PCF8523_TMR_CLKOUT_CTRL_COF1_BIT 4
#define PCF8523_TMR_CLKOUT_CTRL_COF0_BIT 3
#define PCF8523_TMR_CLKOUT_CTRL_TAC1_BIT 2
#define PCF8523_TMR_CLKOUT_CTRL_TAC0_BIT 1
#define PCF8523_TMR_CLKOUT_CTRL_TBC_BIT 0

#define PCF8523_CONTROL_3_VALID_BIT_MASK 0xEF
#define PCF8523_CONTROL_3_RESET_DEFAULT_VALUE 0xE0

struct __attribute__((packed)) TimeDate
{
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  seconds;
    uint16_t milliseconds;
};

bool Pcf8523_Drv_Init(void);

bool PCF8523_Drv_IsInitialized(void);

uint8_t PCF8523_Drv_RtcReadReg(uint8_t address);

void PCF8523_Drv_RtcWriteReg(uint8_t address, uint8_t data);

void PCF8523_Drv_RtcReadRegBuff(uint8_t *p_buf, uint8_t size, uint8_t address);

void PCF8523_Drv_RtcWriteRegBuff(uint8_t address, uint8_t *p_buf, uint8_t size);

void PCF8523_Drv_SetTime(struct TimeDate *p_time);

void PCF8523_Drv_GetTime(struct TimeDate *p_time);

bool PCF8523_Drv_IsControl3Default(void);

void PCF8523_Drv_RtcStart(void);

void PCF8523_Drv_ConfigureIntEverySecond(void);

void PCF8523_Drv_ConfigureBatterySwitchOver(void);

void PCF8523_Drv_ConfigureInternalCapacitors(void);

#endif /* PCF8523_DRV_H_ */
