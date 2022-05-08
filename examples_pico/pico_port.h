#ifndef _PICO_PORT_H
#define _PICO_PORT_H

void pico_port_init(void);

void pico_set_ce_pin(unsigned char v);
void pico_set_csn_pin(unsigned char v);

unsigned char pico_spi_transfer(unsigned char v);

void pico_delay_ms(unsigned int ms);
void pico_delay_us(unsigned int us);

unsigned int pico_millis(void);

#endif
