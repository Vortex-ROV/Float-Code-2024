#if 0

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <string>
#include <queue>
#include "communication.h"

// mac address: cc:db:a7:00:10:80
// peer address: cc:7b:5c:a7:7f:cc

esp_now_peer_info_t peer;

std::queue<String> q;

// lightweight function
void onSend(const uint8_t *mac_addr, esp_now_send_status_t status) {

}

// lightweight function
void onReceive(const uint8_t *esp_now_info, const uint8_t *data, int data_len) {
  q.emplace(String((char*)data));
}

void send(const String& data) {
  int index = 0;
  while (index < data.length()) {
    int sentSize = min(data.length() - index, (unsigned int)ESP_NOW_MAX_DATA_LEN);
    ESP_ERROR_CHECK(esp_now_send(NULL, (uint8_t*)(data.c_str() + index), sentSize));
    index += sentSize;
  }
}

void setup() {
  Serial.begin(115200);

  memset(&peer, 0, sizeof(esp_now_peer_info_t));

  WiFi.mode(WIFI_STA);
  WiFi.softAP("esp32");
  WiFi.disconnect();
  ESP_ERROR_CHECK(esp_now_init());

  uint8_t mac[6] = {0};
  ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));

  Serial.println("MAC Address:");
  Serial.println(WiFi.macAddress());

  esp_now_register_send_cb(onSend);
  esp_now_register_recv_cb(onReceive);

  // set mac address of the peer
  // cc:7b:5c:a7:7f:cc
  uint8_t peerAddress[] = { 0xcc, 0x7b, 0x5c, 0xa7, 0x7f, 0xcc  };
  for (int i = 0; i < 6; i++) {
    peer.peer_addr[i] = peerAddress[i];
  }

  esp_now_add_peer(&peer);

  while (!Serial.available()) {
    continue;
  }

  String command = Serial.readStringUntil('\n');
  Serial.printf("Command: %s\n", command);
  espNowSend(peer, (uint8_t*)command.c_str(), command.length());
}

void loop() {
  while (q.size()) {
    String msg = q.front();
    Serial.println(msg);
    q.pop();
  }
}

#endif