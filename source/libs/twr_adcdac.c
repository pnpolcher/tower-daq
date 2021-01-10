#include "twr_adcdac.h"

#include "fsl_gpio.h"
#include "MK40D10.h"


void ltc1859_cs_enable()
{
	GPIO_SetPinsOutput(BOARD_INITPINS_GPIO7_GPIO, GPIO7 | GPIO8);
	GPIO_SetPinsOutput(BOARD_INITPINS_GPIO9_GPIO, GPIO9);
}

void ltc2498_cs_enable()
{
	GPIO_ClearPinsOutput(BOARD_INITPINS_GPIO7_GPIO, GPIO7);
	GPIO_SetPinsOutput(BOARD_INITPINS_GPIO8_GPIO, GPIO8);
	GPIO_SetPinsOutput(BOARD_INITPINS_GPIO9_GPIO, GPIO9);
}

void cs_disable()
{
	GPIO_ClearPinsOutput(BOARD_INITPINS_GPIO7_GPIO, GPIO7 | GPIO8);
	GPIO_ClearPinsOutput(BOARD_INITPINS_GPIO9_GPIO, GPIO9);
}
