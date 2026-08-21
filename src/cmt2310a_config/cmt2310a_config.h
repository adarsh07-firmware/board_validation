#ifndef CMT2310A_CONFIG_H
#define CMT2310A_CONFIG_H

#include "cmt2310a_config\cmt2310a_params.h"
#include "spi_read/spi_read.h"
#include "spi_write/spi_write.h"
#include "params/params.h"

void write_page0_reg();
void write_page1_reg();
void power_boot();
void cmt2310a_configuration();
#endif