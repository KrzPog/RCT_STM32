#ifndef _DATAFLASH_H
#define _DATAFLASH_H

#include <stdbool.h>

#include "main.h"

#define ALIGN4(x) (((x) + 3) & ~3)

extern uint8_t __dataflash_start__;
extern uint8_t __dataflash_end__;

#define DATAFLASH_START ((uint32_t)(&__dataflash_start__))
#define DATAFLASH_END ((uint32_t)(&__dataflash_end__))
#define DATAFLASH_SIZE (DATAFLASH_END - DATAFLASH_START)

bool readDataFlash(uint32_t addressOffset, uint32_t *data, uint16_t size);

bool writeDataFlash(uint32_t addressOffset, uint32_t *data, uint16_t size);

#endif