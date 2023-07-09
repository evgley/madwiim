/*
 * myCRC.c
 *
 *  Created on: 17 окт. 2019 г.
 *      Author: Игорь
 */
#include "stdint.h"


uint8_t calccrc(void* data,int size){
	uint8_t* pdata=data;
	pdata++;				//с позиции 1 т.к. в нулевой хранится сам CRC
	uint8_t crc=55;
	for (int i=1;i<size;i++){//с позиции 1 т.к. в нулевой хранится сам CRC
		crc+=*pdata + i;
		pdata++;
	}
	if (crc==255)	crc=254;
	if (crc==0)		crc=1;
	return crc;
}



