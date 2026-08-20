#include <Arduino.h>
#include "../src/spi_write/spi_write.h"
#include "spi_read/spi_read.h"
#include "freertos/task.h"
#include "cmt2310ainit/cmt2310a_init.h"
#include "params/params.h"
#include "cmt2310a_config/cmt2310a_params.h"
#include "cmt2310a_config/cmt2310a_config.h"

void setup(){
    Serial.begin(115200);
    delay(2000);
    cmt2310a_initialise();
    uint8_t cur_status_chip=spi_read(CSB,CTL_REG_10);
    if(cur_status_chip!=0x00) Serial.println("Soft reset is not working !");
    else Serial.println("Initialization step is done successfully");
    write_page0_reg();
    write_page1_reg();
}

void loop(){

}

