/*
 * sd.h
 *
 *  Created on: 18 May 2020
 *      Author: lvisagie
 */

#ifndef INC_SD_H_
#define INC_SD_H_

uint8_t SD_Init(void);
uint8_t SD_Read(uint8_t* rxbuffer, uint32_t address, uint32_t numblocks);
uint8_t SD_Write(uint8_t* txbuffer, uint32_t address, uint32_t numblocks);

#endif /* INC_SD_H_ */
