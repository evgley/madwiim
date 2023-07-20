#include "encoder.h"
#include "esp_log.h"

#define TAG "ENCD"


Encoder::Encoder(const Config& cfg) : cfg(cfg) {
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    ESP_ERROR_CHECK(rotary_encoder_init(&encoder, cfg.gpioA, cfg.gpioB));
    ESP_ERROR_CHECK(rotary_encoder_enable_half_steps(&encoder, false));

    QueueHandle_t queue = rotary_encoder_create_queue();
    ESP_ERROR_CHECK(rotary_encoder_set_queue(&encoder, queue));
}

QueueHandle_t Encoder::getEventQueue() {
    return encoder.queue;
}