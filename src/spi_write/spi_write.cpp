#include "spi_write.h"

void spi_write_data_blocking(int chip_select,uint8_t addr,uint8_t data){
    Serial.println
    //IDLE CONDITION
    digitalWrite(chip_select,HIGH);
    digitalWrite(SCLK,LOW);
    vTaskDelay(2*spi_write_bit_time/portTICK_PERIOD_MS);

    //START WRITING 
    digitalWrite(chip_select,LOW);
    vTaskDelay(0.5*spi_write_bit_time/portTICK_PERIOD_MS);
    //Writing 1
    vTaskDelay(spi_write_bit_time/portTICK_PERIOD_MS);
    digitalWrite(SDI,0);
    digitalWrite(SCLK,HIGH);
    vTaskDelay(spi_write_bit_time/portTICK_PERIOD_MS);
    digitalWrite(SCLK,LOW);

    //Writing the address
    for(int i=6;i>=0;i--){
        uint8_t bit=((addr>>i)&1);
        vTaskDelay(spi_write_bit_time/portTICK_PERIOD_MS);
        digitalWrite(SDI,bit);
        digitalWrite(SCLK,HIGH);
        vTaskDelay(spi_write_bit_time/portTICK_PERIOD_MS);
        digitalWrite(SCLK,LOW);
    }

    //Writing the data
    for(int i=7;i>=0;i--){
        uint8_t bit=((data>>i)&1);
        vTaskDelay(spi_write_bit_time/portTICK_PERIOD_MS);
        digitalWrite(SDI,bit);
        digitalWrite(SCLK,HIGH);
        vTaskDelay(spi_write_bit_time/portTICK_PERIOD_MS);
        digitalWrite(SCLK,LOW);
    }

    //IDLE CONDITION
    vTaskDelay(0.5*spi_write_bit_time/portTICK_PERIOD_MS);
    digitalWrite(chip_select,HIGH);
    digitalWrite(SCLK,LOW);
}

bool spi_write_data(int chip_select,uint8_t addr,uint8_t data){
    SPIRequest request;
    request.chip_select=chip_select;
    request.addr=addr;
    request.data=data;
    return xQueueSend(spiQueue,&request,0)==pdTRUE;
}

void spiTask(void *parameter){
    spiQueue=xQueueCreate(10,sizeof(SPIRequest));
    SPIRequest request;
    while(1){
        if(xQueueReceive(spiQueue,&request,portMAX_DELAY)==pdTRUE) spi_write_data_blocking(request.chip_select,request.addr,request.data);
    }
}