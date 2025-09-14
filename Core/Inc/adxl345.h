/*
 * adxl345.h
 *
 *  Created on: Jul 28, 2025
 *      Author: Justyna, Emil
 */

#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_
#pragma once

#include "stm32l4xx.h"

typedef struct {
    float x;
    float y;
    float z;
} ADXL345_Data;

extern volatile uint8_t fifo_collecting;

#define FFT_BUFFER_SIZE 512

extern float z_array_1_fft[FFT_BUFFER_SIZE];
extern float z_array_2_fft[FFT_BUFFER_SIZE];
extern float z_array_1_timeD[FFT_BUFFER_SIZE];
extern float z_array_2_timeD[FFT_BUFFER_SIZE];
extern uint8_t use_one;

HAL_StatusTypeDef adxl345_init(void);

HAL_StatusTypeDef adxl_read_reg(uint8_t reg, uint8_t *buffer, uint16_t len);

float adxl_read_x(void);

float adxl_read_y(void);

float adxl_read_z(void);

uint8_t checkWatermark(void);

uint8_t updateFifoEntries(void);

uint8_t isAdxlRunning(void);

void adxlOn(void);

void adxlOff(void);

void CollectFifo(void);

ADXL345_Data adxl_read_all(void);

#endif /* INC_ADXL345_H_ */
