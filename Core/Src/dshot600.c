/*
 * dshot600.c
 *
 *  Created on: Dec 1, 2023
 *      Author: mango
 */
#include "dshot600.h"

uint16_t pframe[8] =  {0b0000000000000000};
//DSHOT600 sample frame: ^          ^^
//                       |          |\_CRC
//                       |          \__Telemetry bit
//                       \_____________11bit throttle
uint8_t frame_progress[8] = {0}; //0..15
uint8_t active = 0b00000000;
//				   ^   ^
//				   |   \_ESC_0 THR_3
//				   \_____ESC_1 THR_3

uint8_t pbits = 0b00000000;
//				  ^   ^
//				  |   \_ESC_0 THR_3
//				  \_____ESC_1 THR_3

void PrepareFrame(int16_t thrust, int8_t telemetry, ESC esc, THR thruster){
		int16_t frame = thrust << 1 | telemetry;

		int8_t crc = (frame ^ (frame >> 4) ^ (frame >> 8)) & 0x0F;
		frame = (frame << 4) | crc;
		SendFrame(frame, esc, thruster);
}

void SendFrame(int16_t frame, ESC esc, THR thruster){
	uint8_t bitnum = esc<<2 | thruster;
	if(active & (1<<bitnum)){
		LOG_CDC_FS(ERR, "Too much frames @ ESC%d THR%d, progress is %d, pbits is %d", esc, thruster, frame_progress[bitnum], pbits);
	}
	else{
		pframe[bitnum] = frame;
		frame_progress[bitnum] = 0;
		active |= (1<<bitnum);
		pbits &= (frame & 1) << (bitnum);
	}
}

//this writes dshot600 "zeros"
void HAL_HRTIM_Compare1EventCallback(HRTIM_HandleTypeDef * hhrtim, uint32_t TimerIdx)
{
	if(TimerIdx == HRTIM_TIMERINDEX_MASTER){
		TimMBit0s++;
		GPIOF->BSRR = (~pbits & active) << 16;
		GPIOB->BSRR = (~pbits & (active & 1)) << 30;
	}
}

//this writes dshot600 "ones"
void HAL_HRTIM_Compare2EventCallback(HRTIM_HandleTypeDef * hhrtim, uint32_t TimerIdx)
{
	if(TimerIdx == HRTIM_TIMERINDEX_MASTER){
		TimMBit1s++;
		GPIOF->BSRR = (pbits & active) << 16;
		GPIOB->BSRR = (pbits & (active & 1)) << 30;
	}
}

//nya
void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef * hhrtim, uint32_t TimerIdx){
	if(TimerIdx==HRTIM_TIMERINDEX_MASTER){
		TimMRollovers++;
//		uint16_t pactive = 0;
//		uint16_t ppbits = 0;
//		active <<= 1;              		  // and shifted to the left to make room for the next bit
//		pbits <<=1;						  // and shifted to the left to make room for the next bit

		for(int i = 0; i<8; i++){
			if(!(active & (1<<i))) continue;
			frame_progress[i]++; //increment frame progress
			int not_frame_ended = !(frame_progress[i]==16); // this checks if frame is not ended
			pframe[i] >>= 1;
			active <<= 1;              		  // shifted to the left to make room for the next bit
			active &= not_frame_ended; 		  // LSB is set to 1 if frame is still active
			pbits <<=1;						  // shifted to the left to make room for the next bit
			pbits |= pframe[i] & 1;    		  // pending^2 bit is set if there is a set bit in the frame "now"
			frame_progress[i] *= not_frame_ended; //zero out the frame progress if we're at the end
		}

//		ppbits >>= 1;  // overshift compensation
//		pbits = ppbits;// assignment to th
//
//		pactive >>= 1;
//		active = pactive;
		GPIOF->BSRR = active;
		GPIOB->BSRR = (active & 1) << 14;
	}
}
