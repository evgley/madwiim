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

    int getVolume();
    int getVolumeCb(const TProtocol& msg);
    
    void setVolume(int newVol);

    int getSource();
    int getSourceCb(const TProtocol& msg);

    void reboot();

    bool connected = false;
    bool targetFound = false;
private:
    void enableNotificationMessages();
    void disableNotificationMessages();
    
    void reconnect();
    
    template <typename Ret>
    Ret runCommand(int cmd, Ret (Madbit::*cb)(const TProtocol& arg));

    friend void readTask(void*);
    friend void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
    friend void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

    
    MessageBufferHandle_t messageBuf;
    bool sendToMessageBuf = false;
    bool sendToMessageBufChanged = false;

    int fd;

    Madbit();
};