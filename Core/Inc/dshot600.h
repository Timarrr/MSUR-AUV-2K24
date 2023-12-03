/*
 * dshot600.h
 *
 *  Created on: Dec 1, 2023
 *      Author: mango
 */

#ifndef INC_DSHOT600_H_
#define INC_DSHOT600_H_

#include <stdint.h>

enum _controller{
	ESC_0,
	ESC_1,
};

typedef enum _controller ESC;

void SendFrame(int16_t frame, ESC esc);

#endif /* INC_DSHOT600_H_ */
