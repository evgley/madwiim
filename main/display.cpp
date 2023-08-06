#include "display.h"
#include "esp_log.h"

#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_HW_ADDR 0x3C
#define LCD_CMD_BITS 8
#define LCD_H_RES 128
#define LCD_V_RES 32

#define TAG "DPLY"

extern lv_font_t test_32;

void Display::setInfo(const Info& info) {
    ESP_LOGI(TAG, "setInfo %d", info.volume);
    lv_label_set_text_fmt(label, "%d %d", info.volume, int(info.connected));
    lv_refr_now(NULL);
}

Display::Display()
{
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {I2C_MASTER_FREQ_HZ}, // select frequency specific to your project
        .clk_flags = 0, // optional; you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = I2C_HW_ADDR,
        .control_phase_bytes = 1, // refer to LCD spec
        .dc_bit_offset = 6,       // refer to LCD spec
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_CMD_BITS,
    };
    esp_lcd_panel_io_handle_t io_handle;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_NUM_0, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,
        .bits_per_pixel = 1};

    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    uint8_t args[] = {0x02};
    esp_lcd_panel_io_tx_param(io_handle, 0xDA, &args, 1);

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = LCD_H_RES * LCD_V_RES,
        .double_buffer = true,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        }};
    
    lv_disp_t* lvDisplay = lvgl_port_add_disp(&disp_cfg);
    lv_obj_t *scr = lv_disp_get_scr_act(lvDisplay);
    
    label = lv_label_create(scr);
    lv_label_set_text_fmt(label, "%s", buffer); // LV_SYMBOL_WIFI LV_SYMBOL_BLUETOOTH);

    static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_text_font(&style, &test_32); // <--- you have to enable other font sizes in menuconfig
    lv_obj_add_style(label, &style, 0);      
}

extern "C" void init_display()
{
    static Display d;
}