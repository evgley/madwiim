#pragma once

#include "driver/pulse_cnt.h"

struct Encoder
{
    struct Config {
        int highLimit;
        int lowLimit;
        int gpioA;
        int gpioB;
    };

    Encoder(const Config& cfg);
    int getValue();

private:
    Config cfg;
    pcnt_unit_handle_t pcntUnit;
};