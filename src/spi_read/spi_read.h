#ifndef SPI_READ_H
#define SPI_READ_H

#include <Arduino.h>
#include "freertos/task.h"
#include "../src/spi_write/spi_write.h"
#define spi_read_bit_time 1e-4
TaskHandle_t read_task=NULL;

extern QueueHandle_t spi_read_Queue;
struct spi_read_request{
    uint8_t chip_select;
    uint8_t addr;
    TaskHandle_t requester;
};
void spi_read_task(void *parameter);
uint8_t spi_read_data(uint8_t chip_select,uint8_t addr);
uint8_t spi_read(uint8_t chip_select,uint8_t addr);
#endif
