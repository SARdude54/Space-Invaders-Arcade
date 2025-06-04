/*
 * graphics.c
 *
 *  Created on: Jun 3, 2025
 *      Author: Saula
 */
#include "st7789.h"
#include "graphics.h"
/*
 * Fill Background screen functions in RGB565 format
 * */

void FillScreenBlack() {
    uint16_t color = 0x0000;
    ST7789_SetAddrWindow(0, 0, WIDTH - 1, HEIGHT - 1);

    for (uint32_t i = 0; i < WIDTH * HEIGHT; ++i) {
        uint8_t data[2] = {color >> 8, color & 0xFF};
        ST7789_WriteData(data, 2);
    }
}

void FillScreenRed() {
    uint16_t color = 0xF800;
    ST7789_SetAddrWindow(0, 0, WIDTH - 1, HEIGHT - 1);

    for (uint32_t i = 0; i < WIDTH * HEIGHT; ++i) {
        uint8_t data[2] = {color >> 8, color & 0xFF};
        ST7789_WriteData(data, 2);
    }
}

void FillScreenBlue() {
    uint16_t color = 0x001f;
    ST7789_SetAddrWindow(0, 0, WIDTH - 1, HEIGHT - 1);

    for (uint32_t i = 0; i < WIDTH * HEIGHT; ++i) {
        uint8_t data[2] = {color >> 8, color & 0xFF};
        ST7789_WriteData(data, 2);
    }
}

void FillScreenGreen() {
    uint16_t color = 0x07e0;
    ST7789_SetAddrWindow(0, 0, WIDTH - 1, HEIGHT - 1);

    for (uint32_t i = 0; i < WIDTH * HEIGHT; ++i) {
        uint8_t data[2] = {color >> 8, color & 0xFF};
        ST7789_WriteData(data, 2);
    }
}

void FillScreenYellow() {
    uint16_t color = 0xffe0;
    ST7789_SetAddrWindow(0, 0, WIDTH - 1, HEIGHT - 1);

    for (uint32_t i = 0; i < WIDTH * HEIGHT; ++i) {
        uint8_t data[2] = {color >> 8, color & 0xFF};
        ST7789_WriteData(data, 2);
    }
}

void FillScreenColor(uint16_t color) {
    ST7789_SetAddrWindow(0, 0, WIDTH - 1, HEIGHT - 1);

    for (uint32_t i = 0; i < WIDTH * HEIGHT; ++i) {
        uint8_t data[2] = {color >> 8, color & 0xFF};
        ST7789_WriteData(data, 2);
    }
}
