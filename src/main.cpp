#include <Arduino.h>
#include "../src/spi_write/spi_write.h"
#include "freertos/task.h"

TaskHandle_t spi_write_task=NULL;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  vTaskDelay(1000/portTICK_PERIOD_MS);
  xTaskCreatePinnedToCore(spiTask,"spiwrite",1024,NULL,5,&spi_write_task,0);
}

void loop() {
  // put your main code here, to run repeatedly:
}
