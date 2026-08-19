#ifndef SPI_WRITE_H
#define SPI_WRITE_H

#include<Arduino.h>
#include "freertos/task.h"
#define CSB 1
#define SCLK 2
#define SDI  3
#define SDO  4
#define spi_write_bit_time 1e-4

typedef struct{
    int chip_select;
    uint8_t addr;
    uint8_t data;
}SPIRequest;

extern QueueHandle_t spiQueue;
void spi_write_data_blocking(int chip_select,uint8_t addr,uint8_t data);
bool spi_write_data(int chip_select,uint8_t addr,uint8_t data);
void spiTask(void *parameter);

#endif