#include <Arduino.h>
#include "../src/spi_write/spi_write.h"
#include "spi_read/spi_read.h"
#include "freertos/task.h"
#include "cmt2310ainit/cmt2310a_init.h"

#define CTL_REG_10 0x0A //status register

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  vTaskDelay(10/portTICK_PERIOD_MS);
  xTaskCreatePinnedToCore(spi_write_Task,"spiwriteTask",1024,NULL,5,&write_task,0);
  xTaskCreatePinnedToCore(spi_read_task,"spireadTask",1024,NULL,5,&read_task,0);
  cmt2310a_initialise();
  uint8_t data=spi_read_data(CSB,CTL_REG_10);
  if(data!=0x00) Serial.println("Soft reset is not done!");
}

void loop() {
  // put your main code here, to run repeatedly:
}
