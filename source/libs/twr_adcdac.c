#include "twr_adcdac.h"

#include "MK40D10.h"


void ltc1859_cs_enable()
{
	GPIOE->PSOR |= (GPIO7 | GPIO8);
	GPIOC->PSOR |= GPIO9;
}

void ltc1859_cs_disable()
{
	GPIOE->PCOR |= (GPIO7 | GPIO8);
	GPIOC->PCOR |= GPIO9;
}
