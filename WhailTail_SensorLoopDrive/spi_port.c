#include <stdint.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include "spi_port.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"


#include "dac161s997.h"

int dac161s997_spi_xfer(dac161s997_dev_t *dev, uint8_t* tx_buf,
		uint8_t* rx_buf, size_t size) 
        
{
    if(dev == NULL || tx_buf == NULL || rx_buf == NULL)
        return 1;


    //Because there is no 'spi_get_instance' :(
	spi_inst_t * spi = dev->spi_index == 1 ? spi1 : spi0;

    gpio_put(dev->cs_num, false);
	spi_write_read_blocking(spi, tx_buf, rx_buf, size);
    gpio_put(dev->cs_num, true);	

    return 0;
}