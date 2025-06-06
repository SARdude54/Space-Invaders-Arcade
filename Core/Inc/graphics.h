/*
 * graphics.h
 *
 *  Created on: Jun 3, 2025
 *      Author: Saula
 */

/*
 * st7789.h
 *
 *  Created on: Jun 3, 2025
 *      Author: Saula
 */

#ifndef GRAPHICS_H
#define GRAPHICS_H

#include <stdint.h>
#include "stm32l4xx_hal.h" // HAL_SPI_Transmit
#include "st7789.h"

void FillScreenRed(void);
void FillScreenBlack(void);
void FillScreenYellow(void);
void FillScreenGreen(void);
void FillScreenBlue(void);
void FillSCreenColor(uint16_t color);

/*
 * Sprite functionality
 * */

void DrawSprite(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *sprite);

extern uint8_t dma_row_buffer[2 * LCD_WIDTH];
void DrawSpriteScaled_DMA(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *sprite, uint8_t scale);
void DrawSpriteScaled(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *sprite, uint8_t scale);


#endif // ST7789_H
