#pragma once

#include "hal/gpio_types.h"
#include "rotary_encoder.h"

struct Encoder
{
    struct Config {
        int highLimit;
        int lowLimit;
        gpio_num_t gpioA;
        gpio_num_t gpioB;
    };

    Encoder(const Config& cfg);
    QueueHandle_t getEventQueue();

private:
    rotary_encoder_info_t encoder;
    Config cfg;
};