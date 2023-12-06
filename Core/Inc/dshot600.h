/*
 * dshot600.h
 *
 *  Created on: Dec 1, 2023
 *      Author: mango
 */

#ifndef INC_DSHOT600_H_
#define INC_DSHOT600_H_

#include <stdint.h>
#include "logger.h"
#include "main.h"


enum _controller{
	ESC_0,
	ESC_1,
};

enum _thruster{
	THR_0,
	THR_1,
	THR_2,
	THR_3,
};

typedef enum _controller ESC;
typedef enum _thruster THR;

void PrepareFrame(int16_t thrust, int8_t telemetry, ESC esc, THR thruster);
void SendFrame(int16_t frame, ESC esc, THR thruster);

#endif /* INC_DSHOT600_H_ */
