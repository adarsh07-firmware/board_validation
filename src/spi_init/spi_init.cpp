
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