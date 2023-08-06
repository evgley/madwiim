#pragma once

#include <esp_spp_api.h>

struct Madbit {
    static Madbit& getInstance();

    int getVolume();
    void setVolume(int newVol);

    int fd;
    bool connected = false;
private:
    friend void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);

    Madbit();
};