#include "pico_port.h"
#include "defaultPins.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"

void pico_port_init(void)
{
    gpio_init(CE_PIN);
    gpio_set_dir(CE_PIN, GPIO_OUT);
    gpio_init(CSN_PIN);
    gpio_set_dir(CSN_PIN, GPIO_OUT);

    spi_init(spi_default, 1000 * 1000);

    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
}

void pico_set_ce_pin(unsigned char v)
{
    gpio_put(CE_PIN, v);
}

void pico_set_csn_pin(unsigned char v)
{
    gpio_put(CSN_PIN, v);
}

unsigned char pico_spi_transfer(unsigned char v)
{
    uint8_t recv = 0;
    spi_write_read_blocking(spi_default, &v, &recv, 1);
    return recv;
}

void pico_delay_ms(unsigned int ms)
{
    sleep_ms(ms);
}

void pico_delay_us(unsigned int us)
{
    sleep_us(us);
}

unsigned int pico_millis(void)
{
    return to_ms_since_boot(get_absolute_time());
}
