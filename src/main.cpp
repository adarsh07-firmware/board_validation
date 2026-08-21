#include <Arduino.h>
#include "../src/spi_write/spi_write.h"
#include "spi_read/spi_read.h"
#include "freertos/task.h"
#include "cmt2310ainit/cmt2310a_init.h"
#include "params/params.h"
#include "cmt2310a_config/cmt2310a_params.h"
#include "cmt2310a_config/cmt2310a_config.h"
#include "rf_config/rf_config.h"
#include "rf_start_comm/rf_comm.h"

void setup(){
    Serial.begin(115200);
    delay(2000);
    cmt2310a_initialise();
    uint8_t cur_status_chip=spi_read(CSB,CTL_REG_10);
    if(cur_status_chip!=0x00) Serial.println("Soft reset is not working !");
    else Serial.println("Initialization step is done successfully");
    cmt2310a_configuration();
    if(spi_read(CSB,CTL_REG_10)==0X81) Serial.println("Successfully configured the chip");
    else Serial.println("Error while configuration of the chip");
    rf_configuration();
    Serial.println("Starting the tx : ");
    Serial.print("Current state is : ");
    Serial.println(spi_read(CSB,CTL_REG_10),HEX);
    cmt2310a_start_tx();
}

void auto_exit_state(){
    uint8_t data=(spi_read(CSB,0x60)&0x8F);
    data=data | 0x10;
    spi_write(CSB,0x60,data);
}

void loop(){
    Serial.print("Current state of chip is : ");
    Serial.println(spi_read(CSB,CTL_REG_10),HEX);
    auto_exit_state();
    spi_write(CSB,CTL_REG_1,0x01);
    delay(3000);
    // delay(5000);
    // clear_fifo();
    // write_fifo();
    // send_tx();
}

