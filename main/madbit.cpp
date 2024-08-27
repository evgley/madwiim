#include "madbit.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "esp_log.h"
#include "freertos/task.h"
#include <string.h>
#include <string>
#include <vector>
#include <algorithm>
#include <array>
#include <sstream>

#include "nvs.h"
#include "nvs_flash.h"

extern "C" {
#include "console_uart.h"
#include "madbit_protocol.h"
#include "settings_storage.h"
}
#include "unistd.h"
#include "settings_storage.h"

#define TAG "MAD"
#define DEVICE_NAME "MADBIT_ESP32"
#define PIN_LENGTH 4
#define WRITE_NOTIFY_INDEX 1

extern "C" uint8_t calccrc(void* data,int size);

std::vector<uint8_t> packetCompose(uint8_t cmd, uint8_t* data, int dataLen) {
    std::vector<uint8_t> res(8 + dataLen);
    res[0] = '[';
    res[1] = 'C';
    res[2] = 'M';
    res[3] = 'D';
    res[4] = ']';
    res[5] = cmd;
    res[6] = 2 + (((dataLen % 4) == 0) ? dataLen : (((dataLen / 4) * 4) + 4))/4;
    res[7] = '\0';
    memcpy(&res[8], data, dataLen);

    res[7] = calccrc(&res[0], res.size());

    ESP_LOG_BUFFER_HEXDUMP("BTWRITE", &res[0], res.size(), ESP_LOG_DEBUG);
    return res;
}

std::vector<uint8_t> packetCompose(int cmd) {
    return packetCompose(cmd, NULL, 0);
}

static uint8_t spp_data[ESP_SPP_MAX_MTU];
static uint8_t *s_p_data = NULL; /* data pointer of spp_data */
static uint16_t spp_data_length;

static char *bda2str(uint8_t * bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

esp_err_t read_peer_bd_addr(esp_bd_addr_t& res) {
    std::string targetBtAddr;
    ESP_ERROR_CHECK(SettingsStorage::getInstance().get("btaddr", targetBtAddr));
    std::stringstream ss(targetBtAddr);
    
    std::string tmp;
    for (auto& bit : res) {
        std::getline(ss, tmp, ':');
        bit = std::stoul(tmp, nullptr, 16);

    }
        
    ESP_LOGI(TAG, "Read Peer BD Addr from settings: %s parsed: %02x:%02x:%02x:%02x:%02x:%02x", targetBtAddr.c_str(), res[0], res[1], res[2], res[3], res[4], res[5]);
    return ESP_OK;
}

esp_err_t read_peer_pin_code(esp_bt_pin_code_t& res) {
    std::string targetPinCode;
    ESP_ERROR_CHECK(SettingsStorage::getInstance().get("btpin", targetPinCode));

    memcpy(&res[0], &targetPinCode[0], PIN_LENGTH);

    ESP_LOGI(TAG, "Read Peer PIN Code from settings: %s", targetPinCode.c_str());
    return ESP_OK;
}

esp_bd_addr_t& get_peer_bd_addr() {
    static esp_bd_addr_t res = {};
    if (!res[0]) // not initialized
        read_peer_bd_addr(res);
    return res;
}

std::string get_name_from_eir(uint8_t *eir)
{
    if (!eir)
        return {};

    uint8_t rmt_bdname_len = 0;
    uint8_t *rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    std::string name(reinterpret_cast<char*>(rmt_bdname), std::min<int>(rmt_bdname_len, ESP_BT_GAP_MAX_BDNAME_LEN));
    return name.c_str();
}

void writeTask(void*) {
    auto& madbit = Madbit::getInstance();
    while ((spp_data_length = xMessageBufferReceive(madbit.writeBuf, spp_data, sizeof(spp_data),  portMAX_DELAY)))
    {
        s_p_data = spp_data;
        uint32_t needSend = true;
        while (needSend)
        {
            ESP_ERROR_CHECK(esp_spp_write(madbit.handle, spp_data_length, spp_data));
            xTaskNotifyWaitIndexed(WRITE_NOTIFY_INDEX, 0, ULONG_MAX, &needSend, portMAX_DELAY);
        }
    }
}

void readTask(char* buf, uint16_t size) {
    auto& madbit = Madbit::getInstance();
    static std::string data;

    if (size > 0)
    {
        ESP_LOG_BUFFER_HEXDUMP("BTREAD", buf, size, ESP_LOG_DEBUG);
        data.append(buf, size);

        auto cmdBeginIdx = 0;
        while ((cmdBeginIdx = data.find("[CMD]")) != std::string::npos)
        {
            // ESP_LOGE(TAG, "MSG RCV offset:%d size:%d", cmdBeginIdx, data.size());
            // ESP_LOG_BUFFER_HEXDUMP("DATA", data.c_str(), data.size(), ESP_LOG_ERROR);

            auto cmdLength = data.length() - cmdBeginIdx;
            if (cmdLength < sizeof(TProtocol))
                break;

            auto res = reinterpret_cast<const TProtocol *>(&data[cmdBeginIdx]);
            auto cmdLengthMust = res->sizediv4 * 4;
            if (cmdLength < cmdLengthMust)
                break;

            std::string line = data.substr(cmdBeginIdx, cmdLengthMust);
            ESP_LOG_BUFFER_HEXDUMP("BTRESP", line.c_str(), line.size(), ESP_LOG_VERBOSE);
            xMessageBufferSend(madbit.messageBuf, &line[0], cmdLengthMust, portMAX_DELAY);

            data.erase(0, cmdBeginIdx + cmdLengthMust);
        }
    }
}

void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    uint8_t i = 0;
    char bda_str[18] = {0};

    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(TAG, "ESP_SPP_INIT_EVT");
            esp_bt_gap_set_device_name(DEVICE_NAME);
            esp_spp_start_discovery(get_peer_bd_addr());

        } else {
            ESP_LOGE(TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
        }
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        if (param->disc_comp.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(TAG, "ESP_SPP_DISCOVERY_COMP_EVT scn_num:%d", param->disc_comp.scn_num);
            for (i = 0; i < param->disc_comp.scn_num; i++) {
                ESP_LOGI(TAG, "-- [%d] scn:%d service_name:%s", i, param->disc_comp.scn[i],
                         param->disc_comp.service_name[i]);
            }
            /* We only connect to the first found server on the remote SPP acceptor here */
            esp_spp_connect(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_MASTER, param->disc_comp.scn[0], get_peer_bd_addr());
        } else {
            ESP_LOGE(TAG, "ESP_SPP_DISCOVERY_COMP_EVT status=%d", param->disc_comp.status);
            esp_spp_start_discovery(get_peer_bd_addr());

        }
        break;
    case ESP_SPP_OPEN_EVT:
        if (param->open.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(TAG, "ESP_SPP_OPEN_EVT handle:%" PRIu32 " rem_bda:[%s]", param->open.handle,
                     bda2str(param->open.rem_bda, bda_str, sizeof(bda_str)));
            auto& madbit = Madbit::getInstance();
            madbit.handle = param->open.handle;
            madbit.connected = true;
            xTaskCreate(writeTask, "BTWRITE", CONFIG_ESP_MAIN_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &madbit.writeTask);
        } else {
            ESP_LOGE(TAG, "ESP_SPP_OPEN_EVT status:%d", param->open.status);
        }
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%" PRIu32 " close_by_remote:%d", param->close.status,
                 param->close.handle, param->close.async);
        // restart esp32 on disconnect
        esp_restart();

        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        if (param->cl_init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(TAG, "ESP_SPP_CL_INIT_EVT handle:%" PRIu32 " sec_id:%d", param->cl_init.handle, param->cl_init.sec_id);
        } else {
            ESP_LOGE(TAG, "ESP_SPP_CL_INIT_EVT status:%d", param->cl_init.status);
        }
        break;
    case ESP_SPP_DATA_IND_EVT:
        readTask(reinterpret_cast<char*>(param->data_ind.data), param->data_ind.len);
        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGD(TAG, "ESP_SPP_WRITE_EVT status: %d length: %d", param->write.status, param->write.len);
        if (param->write.status == ESP_SPP_SUCCESS) {
            if (s_p_data + param->write.len == spp_data + spp_data_length) {
                /* Means the previous data packet be sent completely, send a new data packet */
                xTaskNotifyIndexed(Madbit::getInstance().writeTask, WRITE_NOTIFY_INDEX, 0, eSetValueWithOverwrite);
            } else {
                /*
                 * Means the previous data packet only be sent partially due to the lower layer congestion, resend the
                 * remainning data.
                 */
                s_p_data += param->write.len;
            }
        } else {
            /* Means the prevous data packet is not sent at all, need to send the whole data packet again. */
            xTaskNotifyIndexed(Madbit::getInstance().writeTask, WRITE_NOTIFY_INDEX, 1, eSetValueWithOverwrite);
        }

        if (!param->write.cong) {
            /* The lower layer is not congested, you can send the next data packet now. */
            // esp_spp_write(param->write.handle, spp_data + SPP_DATA_LEN - s_p_data, s_p_data);
        } else {
            /*
             * The lower layer is congested now, don't send the next data packet until receiving the
             * ESP_SPP_CONG_EVT with param->cong.cong == 0.
             */
            ;
        }

        /*
         * If you don't want to manage this complicated process, we also provide the SPP VFS mode that hides the
         * implementation details. However, it is less efficient and will block the caller until all data has been sent.
         */
        break;
    case ESP_SPP_CONG_EVT:
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
        ESP_LOGI(TAG, "ESP_SPP_CONG_EVT cong:%d", param->cong.cong);
#endif
        if (param->cong.cong == 0) {
            /* Send the privous (partial) data packet or the next data packet. */
            // esp_spp_write(param->write.handle, spp_data + SPP_DATA_LEN - s_p_data, s_p_data);
        }
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_SPP_SRV_OPEN_EVT");
        break;
    case ESP_SPP_UNINIT_EVT:
        ESP_LOGI(TAG, "ESP_SPP_UNINIT_EVT");
        break;

    case ESP_SPP_VFS_REGISTER_EVT:
        ESP_LOGI(TAG, "ESP_SPP_VFS_REGISTER_EVT %d", param->vfs_register.status);
        break;

    default:
        break;
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    ESP_LOGV(TAG, "esp_bt_gap_cb %d", event);
    switch(event){
    // case ESP_BT_GAP_DISC_RES_EVT:
    //     /* Find the target peer device name in the EIR data */
    //     for (int i = 0; i < param->disc_res.num_prop; i++){
    //         if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_EIR)
    //         {
    //             auto name = get_name_from_eir(static_cast<uint8_t*>(param->disc_res.prop[i].val));
    //             std::string targetBtAddr;
    //             ESP_ERROR_CHECK(SettingsStorage::getInstance().get("btaddr", targetBtAddr));
    //             ESP_LOGE(TAG, "Target addr: %s", targetBtAddr.c_str());

    //             char bda_str[18] = {0};
    //             bda2str(param->disc_res.bda, bda_str, sizeof(bda_str));

    //             if (name == TARGET_DEVICE_NAME) {
    //                 memcpy(peer_bd_addr, param->disc_res.bda, ESP_BD_ADDR_LEN);
    //                 /* Have found the target peer device, cancel the previous GAP discover procedure. And go on
    //                  * dsicovering the SPP service on the peer device */
    //                 esp_bt_gap_cancel_discovery();


    //                 Madbit::getInstance().targetFound = true;
    //                 ESP_LOGI(TAG, "Target device '%s' found. Bd addr: %s", TARGET_DEVICE_NAME, bda_str);

    //                 esp_spp_start_discovery(peer_bd_addr);
    //             }
    //         }
    //     }
    //     break;
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_DISC_STATE_CHANGED_EVT %d", param->disc_st_chg.state);

        // if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED && !Madbit::getInstance().targetFound)
        //     Madbit::getInstance().reconnect();

        break;
    case ESP_BT_GAP_RMT_SRVCS_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_RMT_SRVCS_EVT");
        break;
    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_RMT_SRVC_REC_EVT");
        break;
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        break;
    }

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d", param->mode_chg.mode);
        break;

    default:
        break;
    }
}

Madbit& Madbit::getInstance() {
    static Madbit obj;
    return obj;
}

Madbit::Madbit(void)
{
    constexpr auto messageBufSize = 1000;
    messageBuf = xMessageBufferCreate(messageBufSize);
    writeBuf =  xMessageBufferCreate(messageBufSize);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bredr_tx_power_set(ESP_PWR_LVL_P9,ESP_PWR_LVL_P9));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(esp_bt_gap_cb));
    ESP_ERROR_CHECK(esp_spp_register_callback(esp_spp_cb));

    esp_spp_cfg_t bt_spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
    };
    ESP_ERROR_CHECK(esp_spp_enhanced_init(&bt_spp_cfg));
    ESP_ERROR_CHECK(esp_spp_vfs_register());

    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t targetPinCode = {};
    ESP_ERROR_CHECK(read_peer_pin_code(targetPinCode));
    esp_bt_gap_set_pin(pin_type, PIN_LENGTH, targetPinCode);

    char bda_str[18] = {0};
    ESP_LOGI(TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
}

void Madbit::reconnect(void) {
    ESP_LOGI(TAG, "Reconnect start");
    connected = false;
    targetFound = false;

    esp_restart();
}

void Madbit::sendCommand(uint8_t cmd, uint8_t* data, int dataLen) {
    ESP_LOGI(TAG, "sendCommand %d", cmd);
    auto packet = packetCompose(cmd, data, dataLen);

    xMessageBufferSend(writeBuf, &packet[0], packet.size(), portMAX_DELAY);
}

MessageBufferHandle_t Madbit::getMessageBuf() {
    return messageBuf;
}