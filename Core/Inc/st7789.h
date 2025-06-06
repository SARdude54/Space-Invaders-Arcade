/*
 * st7789.h
 *
 *  Created on: Jun 3, 2025
 *      Author: Saula
 */

#ifndef ST7789_H
#define ST7789_H

#include "stm32l4xx_hal.h" // HAL_SPI_Transmit

#define CS_LOW()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
#define CS_HIGH()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
#define DC_LOW()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET)
#define DC_HIGH()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET)
#define RST_LOW()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET)
#define RST_HIGH()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET)

#define WIDTH  205
#define HEIGHT 832

extern volatile uint8_t dma_transfer_complete;

void ST7789_Init(void);
void ST7789_Reset(void);
void ST7789_WriteCommand(uint8_t cmd);
void ST7789_WriteDataBlocking(uint8_t *data, uint16_t len);

void ST7789_WriteDataDMA(uint8_t *data, uint16_t len);
void ST7789_WriteData(uint8_t *data, uint16_t len);
void ST7789_SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);


#endif // ST7789_H
