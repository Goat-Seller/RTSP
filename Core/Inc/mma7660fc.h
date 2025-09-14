/*
 * mma7660fc.h
 *
 *  Created on: Jul 28, 2025
 *      Author: justyna
 */

#ifndef SRC_MMA7660FC_H_
#define SRC_MMA7660FC_H_
#pragma once

#include "stm32l4xx.h"

uint8_t mma_read_reg(uint8_t reg);

HAL_StatusTypeDef mma_init(void);

int8_t mma_get_z(void);

#endif /* SRC_MMA7660FC_H_ */
