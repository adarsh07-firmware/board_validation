#ifndef RF_COMM_H
#define RF_COMM_H

#include <Arduino.h>
#include "spi_write/spi_write.h"
#include "spi_read/spi_read.h"
#include "rf_config/rf_config.h"

void clear_fifo();
void write_fifo();
void send_tx();
void cmt2310a_start_tx();

#endif