/*
 * player.h
 *
 *  Created on: Jun 4, 2025
 *      Author: Saula
 */

#pragma once
#include <stdint.h>

#define PLAYER_WIDTH  13
#define PLAYER_HEIGHT 14

extern const uint16_t *player_sprite;
extern int16_t player_x;
extern int16_t player_y;

struct Player {
	const uint16_t *player_sprite;
	uint16_t x;
	uint16_t y;
	uint16_t prev_x;
	uint16_t prev_y;
	const uint8_t scale;
	const uint16_t sprite_w_scaled;
};

extern struct Player player;

