#include <Arduino.h>
#include "../src/spi_write/spi_write.h"
#include "spi_read/spi_read.h"
#include "freertos/task.h"
#include "spi_init/spi_init.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  vTaskDelay(10/portTICK_PERIOD_MS);
  xTaskCreatePinnedToCore(spi_write_Task,"spiwriteTask",1024,NULL,5,&write_task,0);
  xTaskCreatePinnedToCore(spi_read_task,"spireadTask",1024,NULL,5,&read_task,0);
  spi_initialise();
  spi_wire_cs()
}

void loop() {
  // put your main code here, to run repeatedly:
}
