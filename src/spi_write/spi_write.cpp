#include "spi_write.h"

void spi_write(int chip_select,uint8_t addr,uint8_t data){
    //IDLE CONDITION
    digitalWrite(chip_select,HIGH);
    digitalWrite(SCLK,LOW);
    delayMicroseconds(spi_write_bit_time);

    //START WRITING 
    digitalWrite(chip_select,LOW);
    delayMicroseconds(spi_write_bit_time);
    //Writing 0
    delayMicroseconds(spi_write_bit_time);
    digitalWrite(SDI,0);
    digitalWrite(SCLK,HIGH);
    delayMicroseconds(spi_write_bit_time);
    digitalWrite(SCLK,LOW);

    //Writing the address
    for(int i=6;i>=0;i--){
        uint8_t bit=((addr>>i)&1);
        delayMicroseconds(spi_write_bit_time);
        digitalWrite(SDI,bit);
        digitalWrite(SCLK,HIGH);
        delayMicroseconds(spi_write_bit_time);
        digitalWrite(SCLK,LOW);
    }

    //Writing the data
    for(int i=7;i>=0;i--){
        uint8_t bit=((data>>i)&1);
        delayMicroseconds(spi_write_bit_time);
        digitalWrite(SDI,bit);
        digitalWrite(SCLK,HIGH);
        delayMicroseconds(spi_write_bit_time);
        digitalWrite(SCLK,LOW);
    }

    //IDLE CONDITION
    delayMicroseconds(spi_write_bit_time);
    digitalWrite(chip_select,HIGH);
    digitalWrite(SCLK,LOW);
}