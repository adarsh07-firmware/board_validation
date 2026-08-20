#ifndef RF_CONFIG_H
#define RF_CONFIG_H

#include "params/params.h"
#include "spi_read/spi_read.h"
#include "spi_write/spi_write.h"

void go_ready();
void ir_caliberation();
void rf_configuration();
#endif