#include "display.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"

#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_HW_ADDR 0x3C
#define LCD_CMD_BITS 8
#define LCD_H_RES 128
#define LCD_V_RES 32

#define TAG "DPLY"

//#define DUMMY_DISPLAY

extern const uint8_t usb_flash_png_start[] asm("_binary_usb_flash_png_start");
extern const uint8_t usb_flash_png_end[]   asm("_binary_usb_flash_png_end");

extern const uint8_t usb_audio_png_start[] asm("_binary_usb_audio_png_start");
extern const uint8_t usb_audio_png_end[]   asm("_binary_usb_audio_png_end");

extern const uint8_t toslink_png_start[] asm("_binary_toslink_png_start");
extern const uint8_t toslink_png_end[]   asm("_binary_toslink_png_end");

extern const uint8_t coax_png_start[] asm("_binary_coax_png_start");
extern const uint8_t coax_png_end[]   asm("_binary_coax_png_end");

extern const uint8_t analog_png_start[] asm("_binary_analog_png_start");
extern const uint8_t analog_png_end[]   asm("_binary_analog_png_end");

extern const uint8_t bluetooth_png_start[] asm("_binary_bluetooth_png_start");
extern const uint8_t bluetooth_png_end[]   asm("_binary_bluetooth_png_end");

extern lv_font_t lato_44;

void Display::setState(State s) {
    state = s;

    switch (s)
    {
    case State::Configuring:
        lv_label_set_text(customLabel, "INIT");
        lv_obj_clear_flag(customLabel, LV_OBJ_FLAG_HIDDEN);

        lv_obj_add_flag(volumeLabel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(presetLabel, LV_OBJ_FLAG_HIDDEN);
        break;
    case State::Connecting:
        lv_obj_add_flag(volumeLabel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(presetLabel, LV_OBJ_FLAG_HIDDEN);
        lv_anim_start(&animation);
        break;

    case State::Connected:
        lv_obj_clear_flag(volumeLabel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(presetLabel, LV_OBJ_FLAG_HIDDEN);

        lv_anim_del(bluetoothLabel, nullptr);
        lv_obj_add_flag(bluetoothLabel, LV_OBJ_FLAG_HIDDEN);
        break;
    
    default:
        break;
    }
}


void Display::setVolume(int volume)
{
    lv_label_set_text_fmt(volumeLabel, "%d", volume);
}

lv_img_dsc_t getImageBySource(int source) {
    const uint8_t* imgBegin = nullptr;
    const uint8_t* imgEnd = nullptr;

    switch (source)
    {
    case 0:
        imgBegin = analog_png_start;
        imgEnd = analog_png_end;
        break;
    
    case 1:
        imgBegin = usb_flash_png_start;
        imgEnd = usb_flash_png_end;
        break;

    case 2:
        imgBegin = usb_audio_png_start;
        imgEnd = usb_audio_png_end;
        break;

    case 3:
        imgBegin = toslink_png_start;
        imgEnd = toslink_png_end;
        break;

     case 4:
        imgBegin = coax_png_start;
        imgEnd = coax_png_end;
        break;

    case 5:
        imgBegin = bluetooth_png_start;
        imgEnd = bluetooth_png_end;
        break;  

    default:
        ESP_ERROR_CHECK(source);
        break;
    }

    lv_img_dsc_t img_dsc;
    img_dsc.header.always_zero = 0;
    img_dsc.header.w = 64;
    img_dsc.header.h = 32;
    img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA;
    img_dsc.data_size = imgEnd - imgBegin;
    img_dsc.data = imgBegin;
    return img_dsc;
}

void Display::setSource(int source)
{
    sourceIconDsc = getImageBySource(source);
    lv_img_set_src(sourceIcon, &sourceIconDsc);
}

void Display::setPreset(int preset)
{
    lv_label_set_text_fmt(volumeLabel, "%d", preset);
}

lv_obj_t* Display::init() {
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
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v1(I2C_NUM_0, &io_config, &io_handle));

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
    lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_cfg.task_max_sleep_ms = 100;
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
    return lv_disp_get_scr_act(lvDisplay);
}

void Display::createObjects(lv_obj_t* scr) {
    lv_obj_clean(scr);

    customLabel = lv_label_create(scr);
    lv_label_set_text(customLabel, "");

    volumeLabel = lv_label_create(scr);

    presetLabel = lv_label_create(scr);
    lv_label_set_text(presetLabel, "");

    static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_text_font(&style, &lato_44); // <--- you have to enable other font sizes in menuconfig
    lv_obj_add_style(volumeLabel, &style, 0); 
    lv_obj_set_pos(volumeLabel, 67, 0);

    lv_obj_add_style(presetLabel, &style, 0); 

    bluetoothLabel = lv_label_create(scr);
    lv_label_set_text(bluetoothLabel, LV_SYMBOL_BLUETOOTH);
    lv_obj_add_flag(bluetoothLabel, LV_OBJ_FLAG_HIDDEN);

    sourceIcon = lv_img_create(lv_scr_act());
}

void animate_bluetooth(void *arg, int32_t value)
{
    auto obj = (lv_obj_t*)arg;
    if (value % 2)
        lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
}

Display::Display()
{
#ifndef DUMMY_DISPLAY
        createObjects(init());
        lv_anim_init(&animation);
        lv_anim_set_exec_cb(&animation, animate_bluetooth);
        lv_anim_set_var(&animation, bluetoothLabel);
        lv_anim_set_time(&animation, 50000);
        lv_anim_set_values(&animation, 0, 100);
        lv_anim_set_repeat_count(&animation, LV_ANIM_REPEAT_INFINITE);
#endif // DUMMY_DISPLAY
}

extern "C" void init_display()
{
    static Display d;
}