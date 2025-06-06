/*
 * Entity.h
 *
 *  Created on: Jun 6, 2025
 *      Author: Saula
 */

#include<stdint.h>

#ifndef INC_SPRITE_H_
#define INC_SPRITE_H_

extern const uint8_t player_map[182 * 2];
extern const uint8_t bullet_map[2090 * 2];
extern const uint8_t enemy_map[182 * 2];

struct Sprite {
	const uint16_t *sprite_map;
	uint16_t x;
	uint16_t y;
	uint16_t prev_x;
	uint16_t prev_y;
	uint16_t width;
	uint16_t height;
	uint8_t scale;
	uint16_t sprite_w_scaled;
};


extern struct Sprite enemy;

void init_sprites(void);
struct Sprite init_bullet(const uint16_t player_x);
struct Sprite init_player(void);



#endif /* INC_SPRITE_H_ */
