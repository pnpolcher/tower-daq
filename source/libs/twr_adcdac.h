#ifndef TWR_ADCDAC_H
#define TWR_ADCDAC_H

/*
 * GPIO7 -> PTE27 (SD_WP_DET)
 * GPIO8 -> PTE5 (SDHC_D2)
 * GPIO9 -> PTC19 (CTS1)
 */

#include "pin_mux.h"

#define GPIO7 (1 << BOARD_INITPINS_GPIO7_PIN)
#define GPIO8 (1 << BOARD_INITPINS_GPIO8_PIN)
#define GPIO9 (1 << BOARD_INITPINS_GPIO9_PIN)


void ltc1859_cs_enable();
void ltc2498_cs_enable();
void cs_disable();

#endif
