#include "display.h"
#include "encoder.h"
#include "madbit.h"

#include "esp_task_wdt.h"
#include "esp_log.h"

#define TAG "APP"

extern "C" void app_main() {
    Display display;
    Madbit& madbit = Madbit::getInstance();
    
    Encoder::Config cfg;
    cfg.gpioA = 5;
    cfg.gpioB = 19;
    cfg.lowLimit = -100;
    cfg.highLimit = 100;

    Encoder enc(cfg);

    Display::Info info;
    while (true)
    {
        int vol = enc.getValue();
        if (vol != info.volume) {
            ESP_LOGI(TAG, "Volume changed %d", vol);

            info.volume = vol;
            display.setInfo(info);
        } else {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            //taskYIELD();
        }

    }
    
}