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
#include "stm32l4xx_hal.h"
#include "st7789.h"
#include "sprite.h"

extern uint8_t dma_row_buffer[LCD_WIDTH * 2]; // 2 bytes per pixel (RGB565)

void FillScreenRed(void);
void FillScreenBlack(void);
void FillScreenYellow(void);
void FillScreenGreen(void);
void FillScreenBlue(void);
void FillScreenColor(uint16_t color);


void DrawSprite(const struct Sprite* p);
void ClearSprite(const struct Sprite* p, uint16_t bg_color);

#endif

