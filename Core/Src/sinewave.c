/*
 * sinewave.c
 *
 *  Created on: 04 Mar 2020
 *      Author: lvisagie
 */

#include <math.h>
#include <stdint.h>

int16_t sintab[256];
uint16_t offset_440hz;
uint16_t offset_523hz;

void wave_init()
{
	for (int i = 0; i < 256; i++)
		sintab[i] = (int) (900.0f * sinf(i * 0.02454369261f));

	offset_440hz = 0;
	offset_523hz = 0;
}


void wave_fillbuffer(uint16_t* buffer, uint8_t type, uint16_t len)
{
	for (int i = 0; i < len; i++)
	{
		int16_t sample = 0;
		switch (type)
		{
			case 1:
				sample = sintab[offset_440hz >> 8];
				offset_440hz += 654;
				break;
			case 2:
				sample = sintab[offset_523hz >> 8];
				offset_523hz += 777;
				break;
			case 3:
				sample = (sintab[offset_440hz >> 8] >> 1) + (sintab[offset_523hz >> 8] >> 1);
				offset_440hz += 654;
				offset_523hz += 777;
				break;
		}
		buffer[i] = (uint16_t)(sample + 2048);
	}
}
