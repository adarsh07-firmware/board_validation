#include "spi_read.h"

uint8_t spi_read(uint8_t chip_select,uint8_t addr){
    Serial.println("came here to read at the port");
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

void spi_read_task(void *parameter){
   spi_read_Queue=xQueueCreate(10,sizeof(spi_read_request));
   spi_read_request request;
   while(1){
    if(xQueueReceive(spi_read_Queue,&request,portMAX_DELAY)==pdTRUE) {
        uint32_t read_bytes=spi_read(request.chip_select,request.addr);
        xTaskNotify(request.requester,read_bytes,eSetValueWithOverwrite);
    }
   }
}

uint8_t spi_read_data(uint8_t chip_select,uint8_t addr){
    spi_read_request request;
    request.chip_select=chip_select;
    request.addr=addr;
    request.requester=xTaskGetCurrentTaskHandle();
    if(xQueueSend(spi_read_Queue,&request,0)!=pdTRUE){
        Serial.println("Read operation is failed ");
        return 0x00;
    }
    uint32_t read_bytes;
    xTaskNotifyWait(0,0,&read_bytes,portMAX_DELAY);
    return (uint8_t)read_bytes;
}