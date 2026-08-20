#include "rf_config.h"
void go_ready(){
    spi_write(CSB,CTL_REG_1,0x02);
    delay(2);
    if(spi_read(CSB,CTL_REG_10)==0x82) Serial.println("Successfully entered in ready state");
    else Serial.println("Error while entering in the ready state");
}

void ir_caliberation(){
    spi_write(CSB,CTL_REG_8,0x01);
    while(spi_read(CSB,CTL_REG_9)&0x10==0) Serial.println("Waiting for ir caliberation");
    spi_write(CSB,CTL_REG_8,0x01);
    while(spi_read(CSB,CTL_REG_9)&0x10==0) Serial.println("Waiting for ir caliberation");
}

void rf_configuration(){
    void go_ready();
    void ir_caliberation()
}