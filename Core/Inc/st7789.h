/*
 * st7789.h
 *
 *  Created on: Jun 3, 2025
 *      Author: Saula
 */

#ifndef ST7789_H
#define ST7789_H

#include "stm32l4xx_hal.h" // HAL_SPI_Transmit

#define WIDTH  205
#define HEIGHT 832

void ST7789_Init(void);
void ST7789_Reset(void);
void ST7789_WriteCommand(uint8_t cmd);
void ST7789_WriteData(uint8_t *data, uint16_t size);
void ST7789_SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);


#endif // ST7789_H
