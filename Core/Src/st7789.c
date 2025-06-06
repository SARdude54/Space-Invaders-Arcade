#include "st7789.h"

#define CS_LOW()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
#define CS_HIGH()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
#define DC_LOW()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET)
#define DC_HIGH()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET)
#define RST_LOW()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET)
#define RST_HIGH()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET)

extern volatile uint8_t dma_transfer_complete;
extern SPI_HandleTypeDef hspi1;


/*
 * Low level driver for the ST7789 LCD Screen
 * */


void ST7789_WriteCommand(uint8_t cmd) {
    DC_LOW();
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);

    CS_HIGH();
}

void ST7789_WriteData(uint8_t *data, uint16_t len) {
    DC_HIGH();
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, data, len, HAL_MAX_DELAY);
    CS_HIGH();
}

void ST7789_WriteDataBlocking(uint8_t *data, uint16_t len) {
    DC_HIGH();
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, data, len, HAL_MAX_DELAY);
    CS_HIGH();
}

void ST7789_WriteDataDMA(uint8_t *data, uint16_t len) {
    DC_HIGH();
    CS_LOW();
    dma_transfer_complete = 0;
    HAL_SPI_Transmit_DMA(&hspi1, data, len);
}


void ST7789_Reset() {
    RST_LOW();
    HAL_Delay(10);
    RST_HIGH();
    HAL_Delay(120);
}

void ST7789_Init() {
    ST7789_Reset();

    ST7789_WriteCommand(0x36); // MADCTL
    uint8_t madctl = 0x00;
    ST7789_WriteData(&madctl, 1);

    ST7789_WriteCommand(0x3A); // COLMOD
    uint8_t colmod = 0x55;     // 16-bit/pixel
    ST7789_WriteData(&colmod, 1);

    ST7789_WriteCommand(0x21); // INVON (optional)
    ST7789_WriteCommand(0x11); // SLPOUT
    HAL_Delay(120);
    ST7789_WriteCommand(0x29); // DISPON
}

void ST7789_SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint8_t data[4];

    ST7789_WriteCommand(0x2A); // CASET
    data[0] = x0 >> 8;
    data[1] = x0 & 0xFF;
    data[2] = x1 >> 8;
    data[3] = x1 & 0xFF;
    ST7789_WriteData(data, 4);

    ST7789_WriteCommand(0x2B); // RASET
    data[0] = y0 >> 8;
    data[1] = y0 & 0xFF;
    data[2] = y1 >> 8;
    data[3] = y1 & 0xFF;
    ST7789_WriteData(data, 4);

    ST7789_WriteCommand(0x2C); // RAMWR
}


