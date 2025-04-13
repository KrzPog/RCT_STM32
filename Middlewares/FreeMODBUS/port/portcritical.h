#ifndef _MB_PORTCRITICAL_H
#define _MB_PORTCRITICAL_H

#include "stm32f4xx_hal.h"

void __critical_enter(void);
void __critical_exit(void);

#endif