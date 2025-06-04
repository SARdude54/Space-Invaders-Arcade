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

#include "stm32l4xx_hal.h" // HAL_SPI_Transmit
#include "st7789.h"

void FillScreenRed(void);
void FillScreenBlack(void);
void FillScreenYellow(void);
void FillScreenGreen(void);
void FillSCreenColor(uint16_t color);

#endif // ST7789_H
