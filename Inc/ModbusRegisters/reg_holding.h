#ifndef _REG_HOLDING_H
#define _REG_HOLDING_H

#define REG_HOLDING_START 0x0000
#define REG_HOLDING_COUNT 3

#define REG_HOLDING_TEST1 0x0000
#define REG_HOLDING_TEST2 0x0002
#define REG_HOLDING_TEST3 0x0004

#include <stdint.h>

#include "mb.h"
#include "main.h"

extern uint16_t regHolding[REG_HOLDING_COUNT];

void initRegHolding(void);

#endif