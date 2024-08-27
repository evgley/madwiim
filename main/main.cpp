#include "display.h"
#include "madbit.h"
#include "encoder.h"
#include "init.h"
#include "settings_storage.h"
#include "nvs_flash.h"

#include "esp_task_wdt.h"
#include "esp_log.h"

#include "button_gpio.h"

#include <driver/gpio.h>

#include <algorithm>

#define TAG "APP"


void refreshTask(void*);

class MadWiiM {
public:
    MadWiiM(Madbit* madbit, Display* display)
        : madbit(madbit)
        , display(display) {

    }

    void init() {

        displayInfo.volume = -1;
        displayInfo.source = -1;
        displayInfo.preset = -1;
        displayInfo.initialized = true;
        display->setInfo(displayInfo);

        while (!madbit->connected)
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        auto& settings = SettingsStorage::getInstance();
        std::string shouldSetVolumeOnBoot;
        settings.get("setvolumeonboot", shouldSetVolumeOnBoot);
        if (shouldSetVolumeOnBoot == "on") {
            std::string volumeOnBoot;
            ESP_ERROR_CHECK(settings.get("volumeonboot", volumeOnBoot));
            displayInfo.volume = std::stoi(volumeOnBoot);

            uint8_t volumeBuf[4] = {uint8_t(Madbit::Volume::MAX - displayInfo.volume), 0, 0, 0};
            madbit->sendCommand(PROTOCOL_CMD_RUX_SET_VOLUME, &volumeBuf[0], 4);

        } else
        {
            madbit->sendCommand(PROTOCOL_CMD_RUX_GET_VOLUME);
        }
        madbit->sendCommand(PROTOCOL_CMD_RUX_GET_PRESET);
        madbit->sendCommand(PROTOCOL_CMD_RUX_GET_SOURCE);

        TaskHandle_t taskHandle;
        xTaskCreate(refreshTask, "BTREFRESH", CONFIG_ESP_MAIN_TASK_STACK_SIZE, this, tskIDLE_PRIORITY, &taskHandle);       
    
         while (displayInfo.volume < 0 || displayInfo.preset < 0 || displayInfo.source < 0)
         {
            vTaskDelay(100 / portTICK_PERIOD_MS);
         }
         
        displayInfo.initialized = true;
        displayInfo.connected = true;
        display->setInfo(displayInfo);
    }

    void setVolume(int vol) {
        displayInfo.volume = vol;
        display->setInfo(displayInfo);
        uint8_t newVolume[4] = {uint8_t(Madbit::Volume::MAX - vol), 0, 0, 0};
        madbit->sendCommand(PROTOCOL_CMD_RUX_SET_VOLUME, &newVolume[0], 4);
    }

    int getVolume() {
        return displayInfo.volume;
    }

private:
    int volume = 30;
    friend void refreshTask(void*);

    Madbit* madbit;
    Display* display;
    Display::Info displayInfo;
};

void refreshTask(void* arg) {
    ESP_LOGV("BTREFRESH", "Task started");
    auto madwiim = reinterpret_cast<MadWiiM*>(arg);
    auto messageBuf = madwiim->madbit->getMessageBuf();

    uint8_t buf[128];
    while (xMessageBufferReceive(messageBuf, buf, sizeof(buf), portMAX_DELAY))
    {
        auto msg = reinterpret_cast<TProtocol *>(buf);
        ESP_LOGI(TAG, "Response CMD: %d", msg->cmd);

        bool setDisplayInfo = true;
        switch (msg->cmd)
        {
            case PROTOCOL_CMD_RUX_GET_VOLUME:
                setDisplayInfo = false;
                if (madwiim->displayInfo.volume < 0)
                    madwiim->displayInfo.volume = Madbit::Volume::MAX - msg->data;
                break;

            case PROTOCOL_CMD_RUX_GET_SOURCE:
            case PROTOCOL_CMD_RUX_SET_SOURCE:
                madwiim->displayInfo.source = msg->data;
                break;
            
            case PROTOCOL_CMD_RUX_GET_PRESET:
                madwiim->displayInfo.preset = msg->data;
                break;

            default:
                break;
        }

        if (setDisplayInfo)
            madwiim->display->setInfo(madwiim->displayInfo);
    }
}

static void button_single_click_cb(void *arg,void *usr_data)
{
    Madbit* madbit = static_cast<Madbit*>(usr_data);

    madbit->sendCommand(PROTOCOL_CMD_RUX_INC_SOURCE);
    madbit->sendCommand(PROTOCOL_CMD_OK);
    madbit->sendCommand(PROTOCOL_CMD_RUX_GET_SOURCE);
}

void initButtons(void* arg) {
    button_config_t gpio_btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
        .gpio_button_config = {
            .gpio_num = GPIO_NUM_18,
            .active_level = 0,
    }};

    button_handle_t gpio_btn1 = iot_button_create(&gpio_btn_cfg);
    if(NULL == gpio_btn1) {
        ESP_LOGE(TAG, "Button create failed");
    }

    iot_button_register_cb(gpio_btn1, BUTTON_SINGLE_CLICK, button_single_click_cb, (void*)arg);

    gpio_btn_cfg.gpio_button_config.gpio_num = GPIO_NUM_32;
    button_handle_t gpio_btn2 = iot_button_create(&gpio_btn_cfg);
    if(NULL == gpio_btn2) {
        ESP_LOGE(TAG, "Button create failed");
    }

    iot_button_register_cb(gpio_btn2, BUTTON_PRESS_DOWN, button_single_click_cb, (void*)arg);

    gpio_btn_cfg.gpio_button_config.gpio_num = GPIO_NUM_33;
    button_handle_t gpio_btn3 = iot_button_create(&gpio_btn_cfg);
    if(NULL == gpio_btn3) {
        ESP_LOGE(TAG, "Button create failed");
    }
    iot_button_register_cb(gpio_btn3, BUTTON_PRESS_DOWN, button_single_click_cb, (void*)arg);
};

extern "C" void app_main() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    Display* display = new Display();
    {
        gpio_set_direction(GPIO_NUM_18, GPIO_MODE_INPUT);
        gpio_pullup_en(GPIO_NUM_18);
        auto forceReinit = !gpio_get_level(GPIO_NUM_18);
        ESP_LOGI(TAG, "FORCE REINIT: %d", forceReinit);

        ensure_initialized(*display, forceReinit);
    }

    Madbit& madbit = Madbit::getInstance();
    MadWiiM* madwiim = new MadWiiM(&madbit, display);
    initButtons(&madbit);
  
    auto& settings = SettingsStorage::getInstance();
    std::string encoderSpeedDivStr;
    ESP_ERROR_CHECK(settings.get("encoderspeeddiv", encoderSpeedDivStr));
    auto encoderSpeedDiv = std::stoi(encoderSpeedDivStr);
    ESP_LOGI(TAG, "Encoder speed divider: %d", encoderSpeedDiv);

    madwiim->init();
    Encoder::Config cfg;
    cfg.gpioA = GPIO_NUM_5;
    cfg.gpioB = GPIO_NUM_19;
    Encoder enc(cfg);
    int encoderValue = enc.getValue();
    auto volume = madwiim->getVolume();
    while (true)
    {
        auto newEncoderValue = enc.getValue();
        if (newEncoderValue != encoderValue)
        {
            auto volumeDiff = (newEncoderValue - encoderValue); // TODO / encoderSpeedDiv;
            if (volumeDiff)
            {
                volume += volumeDiff;
                volume = std::min<int>(volume, Madbit::Volume::MAX);
                volume = std::max<int>(volume, Madbit::Volume::MIN);

                madwiim->setVolume(volume);
                encoderValue = newEncoderValue;
            }
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}