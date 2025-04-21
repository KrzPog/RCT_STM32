#ifndef _REG_INPUT_H
#define _REG_INPUT_H

#define REG_INPUT_START 0x0000
#define REG_INPUT_COUNT 5

#define REG_INPUT_COMM_MODE 0x0000
#define REG_INPUT_ROT_POSITION 0x0002
#define REG_INPUT_ELEV_POSITION 0x0004
#define REG_INPUT_ROT_SPEED 0x0006
#define ROG_INPUT_ELEV_SPEED 0x0008

#include <stdint.h>

#include "mb.h"
#include "main.h"

extern uint16_t regInput[REG_INPUT_COUNT];

void initRegInput(void);

#endif