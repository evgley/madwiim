#include "display.h"
#include "encoder.h"
#include "madbit.h"

#include "esp_task_wdt.h"
#include "esp_log.h"

#include "button_gpio.h"

#include <driver/gpio.h>

#include <algorithm>

#define TAG "APP"
#define STARTUP_VOLUME 10
#define MAX_VOLUME 20
#define MIN_VOLUME 0

static void button_single_click_cb(void *arg,void *usr_data)
{
    ESP_LOGI(TAG, "BUTTON_SINGLE_CLICK");
}

void initButtons() {
    button_config_t gpio_btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
        .gpio_button_config = {
            .gpio_num = GPIO_NUM_18,
            .active_level = 0,
    }};

    button_handle_t gpio_btn = iot_button_create(&gpio_btn_cfg);
    if(NULL == gpio_btn) {
        ESP_LOGE(TAG, "Button create failed");
    }

    iot_button_register_cb(gpio_btn, BUTTON_SINGLE_CLICK, button_single_click_cb,NULL);
};

static void _knob_left_cb(void *arg, void *data)
{
    Display* display = static_cast<Display*>(data);
    Display::Info info;
    info.volume =iot_knob_get_count_value((button_handle_t)arg);
    display->setInfo(info);

    ESP_LOGI(TAG, "KONB: KONB_LEFT,count_value:%d",iot_knob_get_count_value((button_handle_t)arg));
}

static void _knob_right_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "KONB: KONB_RIGHT,count_value:%d",iot_knob_get_count_value((button_handle_t)arg));
}

void initEncoder(void* ctx)
{
    knob_config_t cfg = {
        .default_direction = 0,
        .gpio_encoder_a = GPIO_NUM_5,
        .gpio_encoder_b = GPIO_NUM_19,
    };
    knob_handle_t s_knob = iot_knob_create(&cfg);
    if (NULL == s_knob)
    {
        ESP_LOGE(TAG, "knob create failed");
    }

    iot_knob_register_cb(s_knob, KNOB_LEFT, _knob_left_cb, ctx);
    iot_knob_register_cb(s_knob, KNOB_RIGHT, _knob_right_cb, ctx);
}

extern "C" void app_main() {
    Display display;
    Madbit& madbit = Madbit::getInstance();

    initButtons();
    initEncoder(&display);

    
    // Encoder::Config cfg;
    // cfg.gpioA = GPIO_NUM_5;
    // cfg.gpioB = GPIO_NUM_19;
    // cfg.lowLimit = -100;
    // cfg.highLimit = 100;

    // Encoder enc(cfg);
    // auto encoderQueue = enc.getEventQueue();

    // int volumeDiff = STARTUP_VOLUME;
    // Display::Info info;
    // while (true)
    // {
    //     auto newVolume = std::max(std::min(info.volume + volumeDiff, MAX_VOLUME), MIN_VOLUME);
    //     if (newVolume != info.volume)
    //     {
    //         info.volume = newVolume;
    //         display.setInfo(info);
    //     }

    //     rotary_encoder_event_t event = {};
    //     if (xQueueReceive(encoderQueue, &event, 1000 / portTICK_PERIOD_MS) == pdTRUE)
    //     {
    //         switch (event.state.direction)
    //         {
    //         case ROTARY_ENCODER_DIRECTION_CLOCKWISE:
    //             volumeDiff = 1;
    //             break;
    //         case ROTARY_ENCODER_DIRECTION_COUNTER_CLOCKWISE:
    //             volumeDiff = -1;
    //             break;
    //         case ROTARY_ENCODER_DIRECTION_NOT_SET:
    //             volumeDiff = 0;
    //             break;
    //         }
    //     } else {
    //         volumeDiff = 0;

    //     }
    // }
    
}