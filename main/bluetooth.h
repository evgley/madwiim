#pragma once

#include <map>
#include <array>

#include "unistd.h"
#include "esp_bt_defs.h"

struct Bluetooth {
    static Bluetooth& getInstance();

    using DeviceAddress = std::array<uint8_t, ESP_BD_ADDR_LEN>;
    void addConnection(const DeviceAddress& addr);



    std::map<DeviceAddress, int> targets;
private:
    Bluetooth();
};