#include "PCF8523_Drv.h"

#include <Wire.h>

#include "Log.h"

static bool isInitialized = false;

static uint8_t bcd2bin(uint8_t val)
{
    return val - 6 * (val >> 4);
}
static uint8_t bin2bcd(uint8_t val)
{
    return val + 6 * (val / 10);
}

bool Pcf8523_Drv_Init(void)
{
    Wire.begin();

    if (PCF8523_Drv_RtcReadReg(PCF8523_TMR_B_FREQ_CTRL) == __UINT8_MAX__)
    {
        LOG_INFO("RTC is not connected");

        return false;
    }

    isInitialized = true;

    return true;
}

bool PCF8523_Drv_IsInitialized(void)
{
    return isInitialized;
}

uint8_t PCF8523_Drv_RtcReadReg(uint8_t address)
{
    uint8_t data;
    PCF8523_Drv_RtcReadRegBuff(&data, 1, address);
    return data;
}

void PCF8523_Drv_RtcWriteReg(uint8_t address, uint8_t data)
{
    PCF8523_Drv_RtcWriteRegBuff(address, &data, 1);
}

void PCF8523_Drv_RtcReadRegBuff(uint8_t *p_buf, uint8_t size, uint8_t address)
{
    int addrByte = address;
    Wire.beginTransmission(PCF8523_ADDRESS);
    Wire.write(addrByte);
    Wire.endTransmission();

    Wire.requestFrom((uint8_t)PCF8523_ADDRESS, size);
    for (uint8_t pos = 0; pos < size; ++pos)
    {
        p_buf[pos] = Wire.read();
    }
}

void PCF8523_Drv_RtcWriteRegBuff(uint8_t address, uint8_t *p_buf, uint8_t size)
{
    int addrByte = address;
    Wire.beginTransmission(PCF8523_ADDRESS);
    Wire.write(addrByte);
    for (uint8_t pos = 0; pos < size; ++pos)
    {
        Wire.write(p_buf[pos]);
    }
    Wire.endTransmission();
}

void PCF8523_Drv_SetTime(struct TimeDate *p_time)
{
    Wire.beginTransmission(PCF8523_ADDRESS);
    Wire.write(PCF8523_SECONDS);
    Wire.write(bin2bcd(p_time->seconds));
    Wire.write(bin2bcd(p_time->minute));
    Wire.write(bin2bcd(p_time->hour));
    Wire.write(bin2bcd(p_time->day));
    Wire.write(bin2bcd(0));
    Wire.write(bin2bcd(p_time->month));
    Wire.write(bin2bcd(p_time->year - 2000));
    Wire.write(0);
    Wire.endTransmission();
}

void PCF8523_Drv_GetTime(struct TimeDate *p_time)
{
    Wire.beginTransmission(PCF8523_ADDRESS);
    Wire.write(PCF8523_SECONDS);
    Wire.endTransmission();

    Wire.requestFrom(PCF8523_ADDRESS, 7);
    p_time->milliseconds = 0;
    p_time->seconds      = bcd2bin(Wire.read() & 0x7F);
    p_time->minute       = bcd2bin(Wire.read());
    p_time->hour         = bcd2bin(Wire.read());
    p_time->day          = bcd2bin(Wire.read());
    Wire.read();
    p_time->month = bcd2bin(Wire.read());
    p_time->year  = bcd2bin(Wire.read()) + 2000;
}

bool PCF8523_Drv_IsControl3Default(void)
{
    uint8_t reg_val = PCF8523_Drv_RtcReadReg(PCF8523_CONTROL_3);
    return (reg_val & PCF8523_CONTROL_3_VALID_BIT_MASK) == PCF8523_CONTROL_3_RESET_DEFAULT_VALUE;
}

void PCF8523_Drv_RtcStart(void)
{
    if (!isInitialized)
    {
        return;
    }

    uint8_t reg_val = PCF8523_Drv_RtcReadReg(PCF8523_CONTROL_1);
    reg_val &= ~_BV(PCF8523_CONTROL_1_STOP_BIT);
    PCF8523_Drv_RtcWriteReg(PCF8523_CONTROL_1, reg_val);
}

void PCF8523_Drv_ConfigureIntEverySecond(void)
{
    if (!isInitialized)
    {
        return;
    }

    uint8_t reg_val = PCF8523_Drv_RtcReadReg(PCF8523_TMR_CLKOUT_CTRL);
    reg_val |= (1 << PCF8523_TMR_CLKOUT_CTRL_TAM_BIT) | (1 << PCF8523_TMR_CLKOUT_CTRL_COF2_BIT) | (1 << PCF8523_TMR_CLKOUT_CTRL_COF1_BIT) |
               (1 << PCF8523_TMR_CLKOUT_CTRL_COF0_BIT);
    PCF8523_Drv_RtcWriteReg(PCF8523_TMR_CLKOUT_CTRL, reg_val);

    reg_val = PCF8523_Drv_RtcReadReg(PCF8523_CONTROL_1);
    reg_val |= (1 << PCF8523_CONTROL_1_SIE_BIT);
    PCF8523_Drv_RtcWriteReg(PCF8523_CONTROL_1, reg_val);
}

void PCF8523_Drv_ConfigureBatterySwitchOver(void)
{
    if (!isInitialized)
    {
        return;
    }

    uint8_t reg_val = PCF8523_Drv_RtcReadReg(PCF8523_CONTROL_3);
    reg_val &= ~((1 << PCF8523_CONTROL_3_PM0_BIT) | (1 << PCF8523_CONTROL_3_PM1_BIT) | (1 << PCF8523_CONTROL_3_PM2_BIT));
    PCF8523_Drv_RtcWriteReg(PCF8523_CONTROL_3, reg_val);
}

void PCF8523_Drv_ConfigureInternalCapacitors(void)
{
    if (!isInitialized)
    {
        return;
    }

    uint8_t reg_val = PCF8523_Drv_RtcReadReg(PCF8523_CONTROL_1);
    reg_val |= (1 << PCF8523_CONTROL_1_CAP_SEL_BIT);
    PCF8523_Drv_RtcWriteReg(PCF8523_CONTROL_1, reg_val);
}
