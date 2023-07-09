#include "madbit_protocol.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"

extern uint8_t calccrc(void* data,int size);

void protocol_send_cmd(int cmd) {
    return protocol_send_data(cmd, NULL, 0);
}

void protocol_send_data(int cmd, uint8_t* data, int dataLen) {
    const char* fmt = "[CMD]%c%c";
    static char buf[255];

    int cmdDataLen = ((dataLen % 4) == 0) ? dataLen : (((dataLen / 4) * 4) + 4);

    int offset = sprintf(buf, fmt, cmd, (char)(cmdDataLen / 4 + 2));
    memcpy(buf + offset + 1, data, dataLen);
    buf[offset] = calccrc(buf, offset + dataLen + 1);


    ESP_LOG_BUFFER_HEX("TST", buf, offset + dataLen + 1);
   
}