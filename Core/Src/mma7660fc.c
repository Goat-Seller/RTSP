/*
 * mma7660fc.c
 *
 *  Created on: Jul 28, 2025
 *      Author: justyna
 */

#include "mma7660fc.h"
#include "i2c.h"

#define MMA7660_ADDR			(0x4C << 1) // 12C address

#define MMA7660_X     0x00
#define MMA7660_Y     0x01
#define MMA7660_Z     0x02
#define MMA7660_TILT  0x03
#define MMA7660_SRST  0x04
#define MMA7660_SPCNT 0x05
#define MMA7660_INTSU 0x06
#define MMA7660_SHINTX 0x80
#define MMA7660_SHINTY 0x40
#define MMA7660_SHINTZ 0x20
#define MMA7660_GINT 0x10
#define MMA7660_ASINT 0x08
#define MMA7660_PDINT 0x04
#define MMA7660_PLINT 0x02
#define MMA7660_FBINT 0x01
#define MMA7660_MODE  0x07
#define MMA7660_SR    0x08      //sample rate register
#define MMA7660_STAND_BY 0x00
#define MMA7660_ACTIVE   0x01
#define AUTO_SLEEP_120  0X00//120 sample per second
#define AUTO_SLEEP_64   0X01
#define AUTO_SLEEP_32   0X02
#define AUTO_SLEEP_16   0X03
#define AUTO_SLEEP_8    0X04
#define AUTO_SLEEP_4    0X05
#define AUTO_SLEEP_2    0X06
#define AUTO_SLEEP_1    0X07
#define MMA7660_PDET  0x09
#define MMA7660_PD    0x0A


#define TIMEOUT 				1000

uint8_t mma_read_reg(uint8_t reg)
{
	uint8_t value = 0;
	HAL_I2C_Mem_Read(&hi2c1, MMA7660_ADDR, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);
	return value;
}

static void mma_write_reg(uint8_t reg, uint8_t value)
{
    HAL_I2C_Mem_Write(&hi2c1, MMA7660_ADDR, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);
}

HAL_StatusTypeDef mma_init(void)
{
	mma_write_reg(MMA7660_MODE,  0x00);
	mma_write_reg(MMA7660_SR, 0x01);
	mma_write_reg(MMA7660_MODE,  0x01);
    return HAL_OK;
}

int8_t mma_get_z(void)
{
	uint8_t value = mma_read_reg(MMA7660_Z);
	// MMA7660 returns 6-bit 2's complement. Sign extend manually
	 if (value & 0x20)  // Check sign bit (bit 5)
	    value |= 0xC0; // Set upper bits to 1s for negative values

	 //callibration - the sensor has an +3 offset for the measurements

	 return ((int8_t)value-3);
}

