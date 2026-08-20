#ifndef CMT2310A_H
#define CMT2310A_H

#include "spi_write/spi_write.h"
#include "spi_read/spi_read.h"

#define GPIO0 0
#define GPIO1 1
#define CTL_REG_7 0x07
void spi_initialise();
void spi_wire_cs();
void cmt2310a_initialise();
#endif CMT2310A_H