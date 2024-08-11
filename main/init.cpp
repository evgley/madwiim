#include "init.h"
#include "settings_storage.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "nvs_flash.h"
#include "esp_http_server.h"
#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>

#define TAG "INIT"
#define CONFIG_ESP_MAXIMUM_RETRY 5
#define INITIALIZED_KEY "INITIALIZED"

static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

extern const char init_html[] asm("_binary_init_html_start");

StaticSemaphore_t initSemaphoreBuffer;
SemaphoreHandle_t initSemaphore = xSemaphoreCreateBinaryStatic(&initSemaphoreBuffer);

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
		esp_wifi_connect();
	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
		if (s_retry_num < CONFIG_ESP_MAXIMUM_RETRY) {
			esp_wifi_connect();
			s_retry_num++;
			ESP_LOGI(TAG, "retry to connect to the AP");
		} else {
			xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
		}
		ESP_LOGI(TAG,"connect to the AP fail");
	} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
		ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
		s_retry_num = 0;
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
	}
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "MADBIT-ESP32-INIT",
            .password = "madbit32",
            .ssid_len = 17,
            .authmode = WIFI_AUTH_OPEN,
            .max_connection = 4,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished");
}

esp_err_t get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=\"utf-8\"");
    httpd_resp_send(req, init_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t save_init_key_value(const std::string& data) {
    auto delimPos = data.find('=');
    if (delimPos == std::string::npos)
        return ESP_FAIL;
    
    auto dataCopy = data;
    dataCopy[delimPos] = '\0';

    SettingsStorage::getInstance().set(&dataCopy[0], &dataCopy[delimPos+1]);
    return ESP_OK;
}

esp_err_t save_init_data(const std::string& data)
{
    for (auto posBegin = 0; posBegin != std::string::npos; )
    {
        auto posEnd = data.find('&', posBegin);
        if (posEnd == std::string::npos)
            posEnd = data.size();
        
        save_init_key_value(data.substr(posBegin, posEnd - posBegin));
        posBegin = posEnd + 1;

        if (posEnd == data.size())
            break;
    }
    
    return ESP_OK;
}

char from_hex(char ch) {
    return isdigit(ch) ? ch - '0' : tolower(ch) - 'a' + 10;
}

std::string url_decode(const std::string& text) {
    char h;
    std::ostringstream escaped;
    escaped.fill('0');

    for (auto i = text.begin(), n = text.end(); i != n; ++i) {
        std::string::value_type c = (*i);

        if (c == '%') {
            if (i[1] && i[2]) {
                h = from_hex(i[1]) << 4 | from_hex(i[2]);
                escaped << h;
                i += 2;
            }
        } else if (c == '+') {
            escaped << ' ';
        } else {
            escaped << c;
        }
    }

    return escaped.str();
}
esp_err_t post_handler(httpd_req_t *req)
{
    std::string content;
    char buf[128];
    while (content.size() != req->content_len)
    {
        size_t recv_size = std::min(req->content_len - content.size(), sizeof(buf));
        int bytesRed = httpd_req_recv(req, buf, recv_size);
        if (bytesRed > 0) {
            content.append(buf, bytesRed);
        } else {
            if (bytesRed == HTTPD_SOCK_ERR_TIMEOUT) {
                httpd_resp_send_408(req);
                return ESP_OK;
            }
            else
                return ESP_FAIL;
        }
    }
    ESP_LOGI(TAG, "post_handler %s", content.c_str());
    SettingsStorage::getInstance().erase();
    ESP_ERROR_CHECK(save_init_data(url_decode(content)));
    SettingsStorage::getInstance().set(INITIALIZED_KEY, 1);
    
    ESP_ERROR_CHECK(httpd_resp_set_type(req, "text/html; charset=\"utf-8\""));
    const char resp[] = "Настройки сохранены";
    ESP_ERROR_CHECK(httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN));
    ::xSemaphoreGive(initSemaphore);
    return ESP_OK;
}

httpd_handle_t start_http_server() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    httpd_uri_t uri_get = {
        .uri      = "/",
        .method   = HTTP_GET,
        .handler  = get_handler,
        .user_ctx = NULL
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_get));

    httpd_uri_t uri_post = {
        .uri      = "/",
        .method   = HTTP_POST,
        .handler  = post_handler,
        .user_ctx = NULL
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_post));

    return server;
}

void ensure_initialized(Display& display, bool force) {
    ESP_LOGI(TAG, "ensure_initialized");
    auto& settings = SettingsStorage::getInstance();

    int8_t isInitialized = false;
    if (force || ESP_ERR_NVS_NOT_FOUND == settings.get(INITIALIZED_KEY, isInitialized)
     || !isInitialized) {
        wifi_init_softap();
        start_http_server();

        ::xSemaphoreTake(initSemaphore, portMAX_DELAY);
    } else 
        esp_wifi_set_mode(WIFI_MODE_NULL);
}