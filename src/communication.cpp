#include "communication.h"
#include <WiFi.h>

void initEspNow(esp_now_peer_info_t& peer) {
    memset(&peer, 0, sizeof(esp_now_peer_info_t));
  
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("esp32", "esp32");
    WiFi.disconnect();
    ESP_ERROR_CHECK(esp_now_init());
  
    // set mac address of the peer
    // C0:49:EF:F9:9A:48 -------------
    // cc:db:a7:00:10:80
    uint8_t peer_address[] = { 0xcc, 0xdb, 0xa7, 0x00, 0x10, 0x80 };
    for (int i = 0; i < 6; i++) {
      peer.peer_addr[i] = peer_address[i];
    }
  
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

void espNowSend(esp_now_peer_info_t& peer, uint8_t* data, size_t data_len) {
    Serial.printf("sending: %s\n", data);
    int index = 0;
    while (index < data_len) {
        int sentSize = min(data_len - index, (unsigned int)ESP_NOW_MAX_DATA_LEN);
        ESP_ERROR_CHECK(esp_now_send(peer.peer_addr, data + index, sentSize));
        index += sentSize;
        delay(10);
    }
}
