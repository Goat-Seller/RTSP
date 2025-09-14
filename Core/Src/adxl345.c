/*
 * adxl345.c
 *
 *  Created on: Jul 28, 2025
 *      Author: Justyna, Emil
 */

#include "adxl345.h"
#include "i2c.h"
#include <stdio.h>

#define ADXL345_ADDR			(0x53 << 1) // 12C address
#define ADXL345_DEVID			0x00 // Device ID
#define ADXL345_THRESH_TAP 		0x1D // Tap threshold
#define ADXL345_OFSX			0x1E // X-axis offset
#define ADXL345_OFSY			0x1F // Y-axis offset
#define ADXL345_OFSZ			0x20 // Z-axis offset
#define ADXL345_DUR				0x21 // Tap duration
#define ADXL345_Latent			0x22 // Tap latency
#define ADXL345_Window			0x23 // Tap window
#define ADXL345_THRESH_ACT		0x24 // Activity threshold
#define ADXL345_THRESH_INACT	0x25 // Inactivity threshold
#define ADXL345_TIME_INACT		0x26 // Inactivity time
#define ADXL345A_ACT_INACT_CTL	0x27 // Axis enable control for activity and inactivity detection
#define ADXL345_THRESH_FF		0x28 // Free-fall threshold
#define ADXL345_TIME_FF 		0x29 // Free-fall time
#define ADXL345_TAP_AXES		0x2A // Axis control for single tap/double tap
#define ADXL345_ACT_TAP_STATUS	0x2B // Source of single tap/double tap
#define ADXL345_BW_RATE			0x2C // Data rate and power mode control
#define ADXL345_POWER_CTL		0x2D // Power-saving features control
#define ADXL345_INT_ENABLE		0x2E // Interrupt enable control
#define ADXL345_INT_MAP			0x2F // Interrupt mapping control
#define ADXL345_INT_SOURCE		0x30 // Source of interrupts
#define ADXL345_DATA_FORMAT		0x31 // Data format control
#define ADXL345_DATAX0			0x32 // X-Axis Data 0
#define ADXL345_DATAX1			0x33 // X-Axis Data 1
#define ADXL345_DATAY0			0x34 // Y-Axis Data 0
#define ADXL345_DATAY1			0x35 // Y-Axis Data 1
#define ADXL345_DATAZ0			0x36 // Z-Axis Data 0
#define ADXL345_DATAZ1			0x37 // Z-Axis Data 1
#define ADXL345_FIFO_CTL		0x38 // FIFO control
#define ADXL345_FIFO_STATUS		0x39 // FIFO status

// ---- Macro ----
#define BIT(n) ((uint8_t)(1U << (n))) 					// simplified bit shifting
#define FIELD(val, shift) ((uint8_t)((val) << (shift))) // more value shift

// ---- Config ----
#define ODR_400HZ                0x0C         		// BW_RATE = 0x0C -> 400 Hz
#define ODR_1600HZ				 0x0E		  		// BW_RATE = 0x0F -> 1600 Hz
#define ODR_3200HZ               0x0F         		// BW_RATE = 0x0F -> 3200 Hz
#define RANGE_2G                 0x00         		// DATA_FORMAT range = ±2g
#define FULL_RES                 BIT(3)		  		// FULL RESOLUTION BIT
#define FIFO_MODE_STREAM         FIELD(0x02, 6)  	// 10b at bits 7:6
#define FIFO_WATERMARK_SAMPLES   31           		// pick 8..31
#define SAMPLE_BYTES             6           	 	// X0,X1,Y0,Y1,Z0,Z1
#define FIFO_BYTES              (FIFO_WATERMARK_SAMPLES * SAMPLE_BYTES) // How much bytes we need to store one FIFO
#define TIMEOUT 				1000

static uint8_t isInitDone;

static uint8_t fifo_array[FIFO_BYTES];
static uint8_t *fifo_pointer =fifo_array;
volatile uint8_t fifo_collecting = 0; //flag for collecting FIFO

// 4 arrays (2 for fft and 2 for raw data) to avoid problems with reading and writing in the same time
float z_array_1_fft[FFT_BUFFER_SIZE];
float z_array_2_fft[FFT_BUFFER_SIZE];
float z_array_1_timeD[FFT_BUFFER_SIZE];
float z_array_2_timeD[FFT_BUFFER_SIZE];
uint8_t use_one;	// Variable determining what array to use 1 or 2


static uint8_t fifo_count;
static uint16_t fft_count;
static uint8_t offset = 0;

const float lsb_to_g = 0.0039f; //Constant float to convert lsb to g in the +-2g configuration

HAL_StatusTypeDef adxl_write_reg(uint8_t reg, uint8_t buffer)
{
	return HAL_I2C_Mem_Write(&hi2c1, ADXL345_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &buffer, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef adxl_read_reg(uint8_t reg, uint8_t *buffer, uint16_t len)
{
	return HAL_I2C_Mem_Read(&hi2c1, ADXL345_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buffer, len, HAL_MAX_DELAY);
}

float adxl_read_x(void)
{
	int16_t x;
	if (HAL_I2C_Mem_Read(&hi2c1, ADXL345_ADDR, ADXL345_DATAX0 | 0x80, 1, (uint8_t*)&x, 2, TIMEOUT) != HAL_OK)
		Error_Handler();
	return (float)x/256;
}

float adxl_read_y(void)
{
	int16_t y;
	if (HAL_I2C_Mem_Read(&hi2c1, ADXL345_ADDR, ADXL345_DATAY0 | 0x80, 1, (uint8_t*)&y, 2, TIMEOUT) != HAL_OK)
		Error_Handler();
	return (float)y/256;
}

float adxl_read_z(void)
{
	int16_t z;
	if (HAL_I2C_Mem_Read(&hi2c1, ADXL345_ADDR, ADXL345_DATAZ0 | 0x80, 1, (uint8_t*)&z, 2, TIMEOUT) != HAL_OK)
		Error_Handler();
	return (float)z/256;
}

ADXL345_Data adxl_read_all(void)
{
	uint8_t buffer[6];
	int16_t raw_x, raw_y, raw_z;
	ADXL345_Data data;


	if (HAL_I2C_Mem_Read(&hi2c1, ADXL345_ADDR, ADXL345_DATAX0 | 0x80, I2C_MEMADD_SIZE_8BIT, buffer, sizeof(buffer), TIMEOUT) != HAL_OK)
		Error_Handler();

	raw_x = (int16_t)(buffer[1] << 8 | buffer[0]);
	raw_y = (int16_t)(buffer[3] << 8 | buffer[2]);
	raw_z = (int16_t)(buffer[5] << 8 | buffer[4]);

	data.x = (float)raw_x / 256.0f;
	data.y = (float)raw_y / 256.0f;
	data.z = (float)raw_z / 256.0f;

	return data;
}

uint8_t getFifoEntries(void){
	uint8_t fifo_status;
	adxl_read_reg(ADXL345_FIFO_STATUS, &fifo_status, 1);
	uint8_t fifo_entries = fifo_status & 0x3F;
	return fifo_entries;
}

uint8_t isWatermarkOn(void){
	uint8_t watermark = 0;
	adxl_read_reg(ADXL345_INT_SOURCE, &watermark, 1);
	watermark &= 0x02;
	if(watermark == 2) return 1;
	return 0;
}

uint8_t isAdxlRunning(void){
	uint8_t work = 0;
	adxl_read_reg(ADXL345_POWER_CTL, &work, 1);
	if(work & 0x08) {
		return 1;
	}
	return 0;
}

void adxlOn(void){
	adxl_write_reg(ADXL345_POWER_CTL, 1 << 3);
}

void adxlOff(void){
	adxl_write_reg(ADXL345_POWER_CTL, 0 << 3);
}

/**
 * @brief  Function to collect FIFO.
 *
 */
void CollectFifo(void){
	if(!isInitDone){
		return;
	}
	adxlOff();
	fifo_collecting = 1;
	HAL_I2C_Mem_Read_DMA(&hi2c1, ADXL345_ADDR, ADXL345_DATAX0, I2C_MEMADD_SIZE_8BIT, fifo_pointer + offset, 6);
}

/**
 * @brief Function triggered by completing DMA Read
 *
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c != &hi2c1) return;
	// Write data from accelerometer
//	int16_t x,y,z;
	int16_t z;
//	x = (int16_t)((fifo_pointer[fifo_count*6+1] << 8) | fifo_pointer[fifo_count*6+0]);
//	y = (int16_t)((fifo_pointer[fifo_count*6+3] << 8) | fifo_pointer[fifo_count*6+2]);
	z = (int16_t)((fifo_pointer[fifo_count*6+5] << 8) | fifo_pointer[fifo_count*6+4]);

	//Process your data here
	float *z_fft_pointer = use_one ? z_array_1_fft : z_array_2_fft;
	float *z_timeD_pointer = use_one ? z_array_1_timeD : z_array_2_timeD;

	if (fft_count>=FFT_BUFFER_SIZE){
		fft_count = 0;
		fft_full_flag = 1;
	}

	z_fft_pointer[fft_count] = z*lsb_to_g;
	z_timeD_pointer[fft_count] = z*lsb_to_g;

	//check if FIFO is done
	if(fifo_count >= FIFO_WATERMARK_SAMPLES)
	{
		// After collecting all FIFO data
		// Reset for next FIFO samples
		offset = 0;
		fifo_count = 0;
		fifo_collecting = 0;
		adxlOn();
	}
	else{
		offset += 6;
		fifo_count ++;

		HAL_I2C_Mem_Read_DMA(&hi2c1, ADXL345_ADDR, ADXL345_DATAX0, I2C_MEMADD_SIZE_8BIT, fifo_pointer + offset, 6);
	}

	fft_count ++;
}

HAL_StatusTypeDef adxl345_init(void)
{
	// Check if adxl345 is connected
	uint8_t id = 0;
	adxl_read_reg(ADXL345_DEVID, &id, 1);
	if ( id !=  0xE5) return HAL_ERROR;
	// Put adxl in standby mode before configuration
	adxl_write_reg(ADXL345_POWER_CTL, 0x00);

	// Set output data rate to 3200 Hz (0x0F)
	adxl_write_reg(ADXL345_BW_RATE, ODR_3200HZ);

	// Configure data format
	adxl_write_reg(ADXL345_DATA_FORMAT, FULL_RES | RANGE_2G);
	// Turn off FIFO mode to avoid problems in initialization
	adxl_write_reg(ADXL345_FIFO_CTL, 0x00);


	// Map watermark interrupt to INT1 an all other to INT2 (0 -> INT1 1 -> INT2)
	adxl_write_reg(ADXL345_INT_MAP, 0xFD);

	//	adxl_write_reg(ADXL345_INT_ENABLE, 0x00);

	// Enable watermark interrupt
	adxl_write_reg(ADXL345_INT_ENABLE, 0x02);
	// Turn on FIFO control in Stream mode 					 		threshold = 32 → 0x10 |
	adxl_write_reg(ADXL345_FIFO_CTL, FIFO_MODE_STREAM | (FIFO_WATERMARK_SAMPLES & 0x1F));
	// Enable measurement
	adxl_write_reg(ADXL345_POWER_CTL, BIT(3));
	isInitDone = 1;

	return HAL_OK;
}
