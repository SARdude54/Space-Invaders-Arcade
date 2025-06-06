/*
 * graphics.c
 *
 *  Created on: Jun 3, 2025
 *      Author: Saula
 */
#include "st7789.h"
#include "graphics.h"
#include "sprite.h"
/*
 * Fill Background screen functions in RGB565 format
 * */

uint8_t dma_row_buffer[LCD_WIDTH * 2]; // 2 bytes per pixel (RGB565)


void FillScreenBlack() {
    uint16_t color = 0x0000;
    ST7789_SetAddrWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

    for (uint32_t i = 0; i < LCD_WIDTH * LCD_HEIGHT; ++i) {
        uint8_t data[2] = {color >> 8, color & 0xFF};
        ST7789_WriteData(data, 2);
    }
}

void FillScreenRed() {
    uint16_t color = 0xF800;
    ST7789_SetAddrWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

    for (uint32_t i = 0; i < LCD_WIDTH * LCD_HEIGHT; ++i) {
        uint8_t data[2] = {color >> 8, color & 0xFF};
        ST7789_WriteData(data, 2);
    }
}

void FillScreenBlue() {
    uint16_t color = 0x001f;
    ST7789_SetAddrWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

    for (uint32_t i = 0; i < LCD_WIDTH * LCD_HEIGHT; ++i) {
        uint8_t data[2] = {color >> 8, color & 0xFF};
        ST7789_WriteData(data, 2);
    }
}

void FillScreenGreen() {
    uint16_t color = 0x07e0;
    ST7789_SetAddrWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

    for (uint32_t i = 0; i < LCD_WIDTH * LCD_HEIGHT; ++i) {
        uint8_t data[2] = {color >> 8, color & 0xFF};
        ST7789_WriteData(data, 2);
    }
}

void FillScreenYellow() {
    uint16_t color = 0xffe0;
    ST7789_SetAddrWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

    for (uint32_t i = 0; i < LCD_WIDTH * LCD_HEIGHT; ++i) {
        uint8_t data[2] = {color >> 8, color & 0xFF};
        ST7789_WriteData(data, 2);
    }
}

void FillScreenColor(uint16_t color) {
    ST7789_SetAddrWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

    for (uint32_t i = 0; i < LCD_WIDTH * LCD_HEIGHT; ++i) {
        uint8_t data[2] = {color >> 8, color & 0xFF};
        ST7789_WriteData(data, 2);
    }
}

/*
 * Render and clear sprites
 * */

void DrawSprite(const struct Sprite* p) {
    for (uint16_t row = 0; row < p->height; row++) {
        for (uint8_t dy = 0; dy < p->scale; dy++) {
            ST7789_SetAddrWindow(
                p->x,
                p->y + (row * p->scale) + dy,
                p->x + (p->width * p->scale) - 1,
                p->y + (row * p->scale) + dy
            );

            uint16_t idx = 0;
            for (uint16_t col = 0; col < p->width; col++) {
                uint16_t pixel = p->sprite_map[row * p->width + col];
                for (uint8_t dx = 0; dx < p->scale; dx++) {
                    dma_row_buffer[idx++] = pixel >> 8;
                    dma_row_buffer[idx++] = pixel & 0xFF;
                }
            }

            dma_transfer_complete = 0;
            ST7789_WriteDataDMA(dma_row_buffer, idx);
            while (!dma_transfer_complete);
            CS_HIGH();
        }
    }
}


void ClearSprite(const struct Sprite* p, uint16_t bg_color) {
    for (uint16_t row = 0; row < p->height; row++) {
        for (uint8_t dy = 0; dy < p->scale; dy++) {
            ST7789_SetAddrWindow(
                p->prev_x,
                p->y + (row * p->scale) + dy,
                p->prev_x + (p->width * p->scale) - 1,
                p->y + (row * p->scale) + dy
            );

            uint16_t idx = 0;
            for (uint16_t col = 0; col < p->width; col++) {
                for (uint8_t dx = 0; dx < p->scale; dx++) {
                    dma_row_buffer[idx++] = bg_color >> 8;
                    dma_row_buffer[idx++] = bg_color & 0xFF;
                }
            }

            dma_transfer_complete = 0;
            ST7789_WriteDataDMA(dma_row_buffer, idx);
            while (!dma_transfer_complete);
            CS_HIGH();
        }
    }
}


