#pragma once

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
        int volume = 0;
        bool connected = false;
    };
    void setInfo(const Info& info);

private:
    friend void animate(void*);

    lv_obj_t* init();
    void createObjects(lv_obj_t* scr);

    Info info;
    lv_obj_t* volumeLabel;
    lv_obj_t* bluetoothLabel;
};
