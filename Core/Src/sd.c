/*
 * sd.c
 *
 *  Created on: 18 May 2020
 *      Author: lvisagie
 */


#include "main.h"

extern SPI_HandleTypeDef hspi2;

uint8_t SD_ReceiveByte(void)
{
	uint8_t dummy, data;
	dummy = 0xFF;

	HAL_SPI_TransmitReceive(&hspi2, &dummy, &data, 1, 100);

	return data;
}

uint8_t SD_SendCommand(uint8_t cmd, uint32_t arg)
{
	// wait for SD to become ready
	uint32_t ticksstart = HAL_GetTick();
	uint8_t res = 0;
	do
	{
		res = SD_ReceiveByte();
	} while (((HAL_GetTick() - ticksstart) >= 500) && (res != 0xff));

	uint8_t cmdbuffer[6];
	cmdbuffer[0] = 0x40 | cmd;
	cmdbuffer[1] = (uint8_t)(arg >> 24);
	cmdbuffer[2] = (uint8_t)(arg >> 16);
	cmdbuffer[3] = (uint8_t)(arg >> 8);
	cmdbuffer[4] = (uint8_t)arg;

	// crc
	if(cmd == 0)
		cmdbuffer[5] = 0x95;	// CRC for CMD0
	else if (cmd == 8)
		cmdbuffer[5] = 0x87;	// CRC for CMD8 (with 32-bit argument = 0x1AA)
	else
		cmdbuffer[5] = 1;

	// transmit command
	HAL_SPI_Transmit(&hspi2, cmdbuffer, 6, 100);

	// read response code
	uint8_t n = 10;
	do
	{
		res = SD_ReceiveByte();
		n--;
	} while ((res & 0x80) && (n > 0));

	return res;
}

void Deselect()
{
	// set chipselect high
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
	HAL_Delay(1);
}

void Select()
{
	// set chipselect low
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
	HAL_Delay(1);
}

uint8_t SD_Init()
{
	Deselect();

	uint8_t dummy = 0xff;
	for (int i = 0; i < 10; i++)
	{
	  HAL_SPI_Transmit(&hspi2, &dummy, 1, 100);
	}

	Select();

	// send command 0
	if (SD_SendCommand(0, 0) != 1)
		return 0;

	// send command 8
	if (SD_SendCommand(8, 0x1aa) != 1)
		return 0;

	// read R7 response, after CMD8
	uint8_t cmd8return[4];
	for (int i = 0; i < 4; i++)
		cmd8return[i] = SD_ReceiveByte();

	if ((cmd8return[2] != 1) || (cmd8return[3] != 0xaa))
		return 0;

	uint32_t ticksstart = HAL_GetTick();
	do
	{
		if (SD_SendCommand(55, 0) <= 1)
		{
			if (SD_SendCommand(41, 1<< 30) == 0)
			{
				//ok!
				break;
			}
		}
	} while ((HAL_GetTick() - ticksstart) < 1000);

	//add a check here for standard vs high capacity card (Command 58)

	Deselect();

	return 1;
}


uint8_t SD_RxDataBlock(uint8_t* buff)
{
	uint8_t token;

	uint32_t ticksstart = HAL_GetTick();

	// loop until receive a response or timeout
	do
	{
		token = SD_ReceiveByte();
	} while ((token == 0xFF) && ((HAL_GetTick() - ticksstart) < 200));

	// check for invalid response
	if(token != 0xFE)
		return 0;

	// receive data
	for (int i = 0; i < 512; i++)
	{
		buff[i] = SD_ReceiveByte();
	}

	// read and discard CRC
	SD_ReceiveByte();
	SD_ReceiveByte();

	// return success
	return 1;

}

uint8_t SD_Read(uint8_t* rxbuffer, uint32_t address, uint32_t numblocks)
{
	uint32_t sector = address >> 9;

	Select();

	if (numblocks == 1)
	{
		// read single block
		if (SD_SendCommand(17, sector) != 0)
			return 0;

		SD_RxDataBlock(rxbuffer);

		return 1;
	}
	else
	{
		// read multiple blocks
		if (SD_SendCommand(18, sector) != 0)
			return 0;

		for (int i = 0; i < numblocks; i++)
		{
			if (!SD_RxDataBlock(rxbuffer))
				break;

			rxbuffer += 512;
		}

		// stop receiving
		SD_SendCommand(12, 0);
	}

	Deselect();

	return 1;
}

uint8_t SD_TxDataBlock(uint8_t *buff, uint8_t token)
{
	uint8_t resp = 0;
	uint8_t crc[2] = {0, 0};

	// transmit token
	HAL_SPI_Transmit(&hspi2, &token, 1, 100);

	// transmit data bytes
	HAL_SPI_Transmit(&hspi2, buff, 512, 100);

	// transmit dummy crc
	HAL_SPI_Transmit(&hspi2, crc, 2, 100);

	// wait for data response token
	for (int i = 0; i < 64; i++)
	{
		resp = SD_ReceiveByte();

		if ((resp & 0x1F) == 0x05)
			break;
	}

	// now wait for programming to finish. TO DO: add a timeout here...
	while (SD_ReceiveByte() == 0);

	// return success if data was accepted
	if ((resp & 0x1F) == 0x05)
		return 1;

	return 0;
}

uint8_t SD_Write(uint8_t* txbuffer, uint32_t address, uint32_t numblocks)
{
	Select();

	uint32_t sector = address >> 9;

	if (numblocks == 1)
	{
		if (SD_SendCommand(24, sector) == 0)
		{
			SD_TxDataBlock(txbuffer, 0xFE);
		}
	}
	else
	{
		if (SD_SendCommand(25, sector) == 0)
		{
			for (int i = 0; i < numblocks; i++)
			{
				if (!SD_TxDataBlock(txbuffer, 0xFC))
					break;

				txbuffer += 512;
			}

			uint8_t token = 0xFD;
			HAL_SPI_Transmit(&hspi2, &token, 1, 100);
		}
	}

	Deselect();

	return 1;
}
