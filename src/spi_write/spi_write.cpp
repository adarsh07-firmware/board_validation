#include "spi_write.h"

void spi_write(int chip_select,uint8_t addr,uint8_t data){
    Serial.println("came here to write at the port");
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
    vTaskDelay(0.5*spi_write_bit_time/portTICK_PERIOD_MS);
    digitalWrite(chip_select,HIGH);
    digitalWrite(SCLK,LOW);
}

void spi_write_Task(void *parameter){
    spi_write_Queue=xQueueCreate(10,sizeof(SPI_write_Request));
    SPI_write_Request request;
    while(1){
        if(xQueueReceive(spi_write_Queue,&request,portMAX_DELAY)==pdTRUE) spi_write(request.chip_select,request.addr,request.data);
    }
}

bool spi_write_data(int chip_select,uint8_t addr,uint8_t data){
    SPI_write_Request request;
    request.chip_select=chip_select;
    request.addr=addr;
    request.data=data;
    bool isdone=(xQueueSend(spi_write_Queue,&request,0)==pdTRUE);
    if(!isdone) Serial.println("SPI write queue is full");
}