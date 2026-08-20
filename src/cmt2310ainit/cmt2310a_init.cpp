#include "cmt2310a_init.h"
#define SOFT_RST 0X7F

void spi_initialise(){
    digitalWrite(GPIO0,INPUT);
    digitalWrite(GPIO1,INPUT);
    digitalWrite(CSB,OUTPUT);    
    digitalWrite(SCLK,OUTPUT);
    digitalWrite(SDI,OUTPUT);
    digitalWrite(SDO,OUTPUT);
}

void spi_wire_cs(){
    uint8_t data=(spi_read_data(CSB,CTL_REG_7)&0xF7);
    spi_write_data(CSB,CTL_REG_7,data);
}

void soft_reset(){
    spi_write_data(CSB,SOFT_RST,0xFF);
}

void cmt2310a_initialise(){
    spi_ini
}