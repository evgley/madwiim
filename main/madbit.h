#pragma once

#include <esp_spp_api.h>

#include "freertos/FreeRTOS.h"
#include "freertos/message_buffer.h"

struct Madbit {
    static Madbit& getInstance();

    enum Volume {
        MAX = 60,
        MIN = 0
    };

    int getVolume();
    void setVolume(int newVol);

    bool connected = false;
private:
    void enableNotificationMessages();
    void disableNotificationMessages();

    friend void readTask(void*);
    friend void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
    
    MessageBufferHandle_t messageBuf;
    bool sendToMessageBuf = false;
    bool sendToMessageBufChanged = false;

    int fd;

    Madbit();
};