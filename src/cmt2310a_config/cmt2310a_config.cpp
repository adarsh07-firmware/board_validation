#include "cmt2310a_config/cmt2310a_config.h"
void write_page0_reg(){
    Serial.println(spi_read(CSB,PAGE_CTL));
    uint8_t data=(spi_read(CSB,PAGE_CTL)&0x3F);
    spi_write(CSB,PAGE_CTL,data);
    data=(spi_read(CSB,PAGE_CTL)&0xC0);
    if(data!=0) Serial.println("Error while switching in page 0");
    else Serial.println("Successfully switched to page0 register");
    for(int i=28;i<=77;i++) spi_write(CSB,FIFO_PORT,(uint8_t)g_cmt2310a_page0[i-28]);
}

void write_page1_reg(){
    spi_write(CSB, PAGE_CTL, 0x40);
    delayMicroseconds(10);
    uint8_t data = spi_read(CSB, PAGE_CTL);
    if ((data & 0xC0) == 0x40) Serial.println("Successfully switched to PAGE 1");
    else Serial.println("ERROR switching to PAGE 1");
    for(uint8_t i=0;i<=(uint8_t) 0x6F;i++) spi_write(CSB,CRW_PORT,(uint8_t)g_cmt2310a_page1[i]);
}

//Currently disabled now 
// void write_page2_reg(){
//     spi_write(CSB,PAGE_CTL,0X80);
//     delayMicroseconds(10);
//     uint8_t data=spi_read(CSB,PAGE_CTL);
//     if(data==0x80) Serial.println("Successfully switched to page 2");
//     else Serial.println("Error occured in page2 switching ");
//     for(uint8_t i=0;i<(uint8_t)0x3F;i++) spi_write(CSB,CRW_PORT,(uint8_t)g_cmt2310a_page2[i]);
// }

void power_boot(){
    Serial.println(spi_read(CSB,PAGE_CTL));
    uint8_t data=(spi_read(CSB,PAGE_CTL)&0x3F);
    spi_write(CSB,PAGE_CTL,data);
    data=(spi_read(CSB,PAGE_CTL)&0xC0);
    if(data!=0) Serial.println("Error while switching in page 0");
    else Serial.println("Successfully switched to page0 register");
    spi_write(CSB,CTL_REG_0,0x03);
    delay(5);
}
