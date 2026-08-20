#include "cmt2310a_init.h"
#define SOFT_RST 0X7F

void spi_initialise(){
    pinMode(GPIO0,INPUT);
    pinMode(GPIO1,INPUT);
    pinMode(CSB,OUTPUT);    
    pinMode(SCLK,OUTPUT);
    pinMode(SDI,OUTPUT);
    pinMode(SDO,INPUT);
}

void spi_wire_cs(){
    uint8_t data=(spi_read(CSB,CTL_REG_7)&0xF7);
    spi_write(CSB,CTL_REG_7,data);
}

void soft_reset(){
    spi_write(CSB,SOFT_RST,0xFF);
}

void cmt2310a_initialise(){
    spi_initialise();
    spi_wire_cs();
    soft_reset();
    delay(20);
}