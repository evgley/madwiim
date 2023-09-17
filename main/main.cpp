#include "display.h"
#include "madbit.h"
#include "encoder.h"

#include "esp_task_wdt.h"
#include "esp_log.h"

#include "button_gpio.h"

#include <driver/gpio.h>

#include <algorithm>

#define TAG "APP"

class MadWiiM {
public:
    MadWiiM(Madbit* madbit, Display* display)
        : madbit(madbit)
        , display(display) {

        displayInfo.volume = 0;
        display->setInfo(displayInfo);
    }

    int initVolume() {
        int vol = madbit->getVolume();
        displayInfo.volume = vol;
        display->setInfo(displayInfo);

        return vol;
    }

    void setVolume(int vol) {
        displayInfo.volume = vol;
        display->setInfo(displayInfo);
        madbit->setVolume(vol);
    }

    void setConnected(bool c) {
        displayInfo.connected = c;
        display->setInfo(displayInfo);
    }

private:
    int volume = 30;

    Madbit* madbit;
    Display* display;
    Display::Info displayInfo;
};

static void button_single_click_cb(void *arg,void *usr_data)
{
    ESP_LOGI(TAG, "BUTTON_SINGLE_CLICK");

    Madbit* madbit = static_cast<Madbit*>(usr_data);
    madbit->getVolume();
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

    iot_button_register_cb(gpio_btn1, BUTTON_PRESS_DOWN, button_single_click_cb, (void*)arg);

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
    Display* display = new Display();

    Madbit& madbit = Madbit::getInstance();
    MadWiiM* madwiim = new MadWiiM(&madbit, display);

    while (true)
    {
        if (madbit.connected) {
            madwiim->setConnected(true);
            break;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    initButtons(&madbit);

    int volume = madwiim->initVolume();
    
    Encoder::Config cfg;
    cfg.gpioA = GPIO_NUM_5;
    cfg.gpioB = GPIO_NUM_19;

    Encoder enc(cfg);
    int val = enc.getValue();
    while (true)
    {
        auto newVal = enc.getValue();
        if (newVal != val) {

            volume += (newVal - val);
            volume = std::min<int>(volume, Madbit::Volume::MAX);
            volume = std::max<int>(volume, Madbit::Volume::MIN);

            madwiim->setVolume(volume);
            val = newVal;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}