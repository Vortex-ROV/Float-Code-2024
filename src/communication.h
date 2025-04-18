#include <esp_now.h>
#include <WString.h>

// float mac address: cc:7b:5c:a7:7f:cc
// topside mac address: cc:db:a7:00:10:80

void initEspNow(esp_now_peer_info_t& peer);
void espNowSend(esp_now_peer_info_t& peer, uint8_t* data, size_t data_len);
