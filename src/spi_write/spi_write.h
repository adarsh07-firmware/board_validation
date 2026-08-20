#ifndef SPI_WRITE_H
#define SPI_WRITE_H

#include<Arduino.h>
#include "freertos/task.h"
#define spi_write_bit_time 1
#include "params/params.h"

void spi_write(int chip_select,uint8_t addr,uint8_t data);

#endif