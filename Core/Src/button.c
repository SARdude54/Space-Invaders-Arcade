/*
 * Button.c
 *
 *  Created on: Jun 5, 2025
 *      Author: alish
 */

#include "button.h"
#include "main.h"

#define BUTTON_PORT   GPIOA
#define BUTTON_PIN    GPIO_PIN_10  // D2

bool Button_IsPressed(void) {
    return HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_RESET;
}
