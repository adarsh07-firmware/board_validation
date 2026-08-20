#ifndef SPI_READ_H
#define SPI_READ_H

#include <Arduino.h>
#include "freertos/task.h"
#include "../src/spi_write/spi_write.h"
#include "params/params.h"
#define spi_read_bit_time 1
uint8_t spi_read(uint8_t chip_select,uint8_t addr);
#endif
