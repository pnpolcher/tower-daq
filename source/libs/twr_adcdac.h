#ifndef TWR_ADCDAC_H
#define TWR_ADCDAC_H

/*
 * GPIO7 -> PTE27 (SD_WP_DET)
 * GPIO8 -> PTE5 (SDHC_D2)
 * GPIO9 -> PTC19 (CTS1)
 */
#define GPIO7 (1 << 27)
#define GPIO8 (1 << 5)
#define GPIO9 (1 << 19)


void ltc1859_cs_enable();
void ltc1859_cs_disable();

#endif
