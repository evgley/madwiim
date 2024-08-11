#pragma once

#include "nvs_flash.h"
#include <string>

class SettingsStorage {
public:
    SettingsStorage();
    ~SettingsStorage();

    esp_err_t set(const char* key, const char* value);
    esp_err_t get(const char* key, std::string& value);

    
    esp_err_t set(const char* key, int8_t value);
    esp_err_t get(const char* key, int8_t& value);

    esp_err_t erase();

    static SettingsStorage& getInstance();

private:
    nvs_handle_t storage;
};