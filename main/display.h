#pragma once

#include "driver/i2c.h"
#include "hal/gpio_types.h"
#include "esp_lcd_types.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"


struct Display {

    Display();

    struct Info {
        int volume;
    };
    void setInfo(const Info& info);

private:
    static void updateDisplay(void* pvDisplay);
    lv_obj_t *label;
    char buffer[8] = {'0', '0', '\n'};
};
