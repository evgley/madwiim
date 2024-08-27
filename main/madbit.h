#pragma once

#include <esp_spp_api.h>
#include <esp_gap_bt_api.h>

#include "freertos/FreeRTOS.h"
#include "freertos/message_buffer.h"

#include "madbit_protocol.h"

struct Madbit {
    static Madbit& getInstance();

    enum Volume {
        MAX = 60,
        MIN = 0
    };

    void sendCommand(uint8_t cmd, uint8_t* data=nullptr, int dataLen=0);
    MessageBufferHandle_t getMessageBuf();

    bool connected = false;
    bool targetFound = false;
private:
    void reconnect();
    
    friend void readTask(char* buf, uint16_t size);
    friend void writeTask(void*);

    friend void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
    friend void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
    
    MessageBufferHandle_t messageBuf;
    MessageBufferHandle_t writeBuf;
    TaskHandle_t writeTask;

    int handle;

    Madbit();
};