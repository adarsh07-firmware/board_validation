#include "rf_config.h"
void go_ready(){
    spi_write(CSB,CTL_REG_1,0x02);
    delay(2);
    if((spi_read(CSB,CTL_REG_10) & 0xFF) == 0x82) Serial.println("[go_ready()] Successfully entered in ready state");
    else Serial.println("[go_ready()] Error while entering in the ready state");
}
// void go_sleep(){
//     uint32_t start = millis();
//     while (spi_read(CSB, CTL_REG_10) == 0xA0) {
//         if (millis() - start > 2000) {
//             Serial.println("TX timeout before sleep");
//             return;
//         }
//         delay(10);
//     }
//     spi_write(CSB, CTL_REG_1, 0x01);
//     delay(10);
//     if (spi_read(CSB, CTL_REG_10) == 0x81)
//         Serial.println("came in sleep state");
//     else
//         Serial.println("failed to come in sleep state");
// }

void ir_caliberation(){
    spi_write(CSB,CTL_REG_8,0x01);
    // while((spi_read(CSB,CTL_REG_9) & 0x10) == 0) Serial.println("Waiting for ir caliberation");
    delay(10);
    spi_write(CSB,CTL_REG_8,0x01);
    // while((spi_read(CSB,CTL_REG_9) & 0x10) == 0) Serial.println("Waiting for ir caliberation");
    delay(10);
}

void rf_configuration(){
    go_ready();
    ir_caliberation();
}