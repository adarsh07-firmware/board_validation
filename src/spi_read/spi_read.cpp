#include "spi_read.h"

uint8_t spi_read(uint8_t chip_select,uint8_t addr){
    //START WRITING 
    digitalWrite(chip_select,LOW);
    delayMicroseconds(spi_write_bit_time);
    //Writing 1
    delayMicroseconds(spi_write_bit_time);
    digitalWrite(SDI,1);
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

    //Start Reading 
    uint8_t read_byte=0;
    for(int i=0;i<8;i++){
        delayMicroseconds(spi_write_bit_time);
        digitalWrite(SCLK,HIGH);
        read_byte=((read_byte<<1)|digitalRead(SDO));
        delayMicroseconds(spi_write_bit_time);
        digitalWrite(SCLK,LOW);
    }

    //IDLE CONDITION
    delayMicroseconds(2*spi_write_bit_time);
    digitalWrite(chip_select,HIGH);
    digitalWrite(SCLK,LOW);
    return read_byte;
}