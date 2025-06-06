/*
 * Joystick.c
 *
 *  Created on: Jun 5, 2025
 *      Author: alish
 */

#include "joystick.h"
#include "main.h"

#define JOY_LEFT_PORT   GPIOA
#define JOY_LEFT_PIN    GPIO_PIN_0  // A0

#define JOY_RIGHT_PORT  GPIOA
#define JOY_RIGHT_PIN   GPIO_PIN_1  // A1

int Joystick_ReadDirection(void) {
    if (HAL_GPIO_ReadPin(JOY_LEFT_PORT, JOY_LEFT_PIN) == GPIO_PIN_RESET)
        return 1;   // LEFT pressed
    else if (HAL_GPIO_ReadPin(JOY_RIGHT_PORT, JOY_RIGHT_PIN) == GPIO_PIN_RESET)
        return -1;  // RIGHT pressed
    else
        return 0;   // Neutral
}
