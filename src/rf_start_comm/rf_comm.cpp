#include "rf_comm.h"

void clear_fifo(){
    spi_write(CSB, CTL_REG_27, 0x01);
}

void write_fifo(){
    for(int i=0;i<32;i++){
        uint8_t data = 0x8F;
        spi_write(CSB, FIFO_PORT, data);
        delay(15);
    }
}

void send_tx(){
    uint8_t status = spi_read(CSB, CTL_REG_10);
    if(status==0x82) Serial.println("[send_tx()] Cheap is ready to receive");
    else Serial.println("[send_tx()] Cheap is not ready");
    spi_write(CSB, CTL_REG_1, 0x04);
    delay(500);
    uint8_t final_status = spi_read(CSB,CTL_REG_10);
    if(final_status!=0xA0) Serial.printf("[send_tx()] Tx completed, final state: 0x%02X\n", final_status);
    else {
        Serial.println("[send_tx()] Tx is still active after 500 ms");
    }
}

void cmt2310a_start_tx(){
    if((spi_read(CSB,CTL_REG_10) & 0xFF) != 0x82){
        Serial.println("[cmt2310a_start_tx] READY state not confirmed");
        go_ready();
        delay(5);
    }
    else Serial.println("[cmt2310a_start_tx] Ready state is confirmed");
    clear_fifo();
    delay(5);
    write_fifo();
    delay(5);
    send_tx();
    // go_sleep();
}