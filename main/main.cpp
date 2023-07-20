#include "display.h"
#include "encoder.h"
#include "madbit.h"

#include "esp_task_wdt.h"
#include "esp_log.h"

#include <algorithm>

#define TAG "APP"
#define STARTUP_VOLUME 10
#define MAX_VOLUME 20
#define MIN_VOLUME 0

extern "C" void app_main() {
    Display display;
    Madbit& madbit = Madbit::getInstance();
    
    Encoder::Config cfg;
    cfg.gpioA = GPIO_NUM_5;
    cfg.gpioB = GPIO_NUM_19;
    cfg.lowLimit = -100;
    cfg.highLimit = 100;

    Encoder enc(cfg);
    auto encoderQueue = enc.getEventQueue();

    int volumeDiff = STARTUP_VOLUME;
    Display::Info info;
    while (true)
    {
        auto newVolume = std::max(std::min(info.volume + volumeDiff, MAX_VOLUME), MIN_VOLUME);
        if (newVolume != info.volume)
        {
            info.volume = newVolume;
            display.setInfo(info);
        }

        rotary_encoder_event_t event = {};
        if (xQueueReceive(encoderQueue, &event, 1000 / portTICK_PERIOD_MS) == pdTRUE)
        {
            switch (event.state.direction)
            {
            case ROTARY_ENCODER_DIRECTION_CLOCKWISE:
                volumeDiff = 1;
                break;
            case ROTARY_ENCODER_DIRECTION_COUNTER_CLOCKWISE:
                volumeDiff = -1;
                break;
            case ROTARY_ENCODER_DIRECTION_NOT_SET:
                volumeDiff = 0;
                break;
            }
        } else {
            volumeDiff = 0;
        }
    }
    
}