#include "settings_storage.h"
#include "esp_log.h"
#include "esp_err.h"

#define TAG "SETT"

SettingsStorage::SettingsStorage() {
    ESP_ERROR_CHECK(nvs_open("madwiim", NVS_READWRITE, &storage));
}

SettingsStorage::~SettingsStorage() {
    nvs_close(storage);
}

esp_err_t SettingsStorage::set(const char* key, const char* value) {
    return nvs_set_str(storage, key, value);
}

esp_err_t SettingsStorage::get(const char* key, std::string& value) {
    size_t size = {};
    auto err = nvs_get_str(storage, key, NULL, &size);
    if (err != ESP_OK)
        return err;
    
    value.resize(size);
    err = nvs_get_str(storage, key, &value[0], &size);
    if (err == ESP_OK)
        value.pop_back();

    return err;
}


esp_err_t SettingsStorage::set(const char* key, int8_t value) {
    return nvs_set_i8(storage, key, value);
  
}

esp_err_t SettingsStorage::get(const char* key, int8_t& value) {
    return nvs_get_i8(storage, key, &value);
}

esp_err_t SettingsStorage::erase() {
    return nvs_erase_all(storage);
}

SettingsStorage& SettingsStorage::getInstance() {
    static SettingsStorage ss;
    return ss;
}

