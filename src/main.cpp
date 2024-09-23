#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "Arduino.h"
#include "EEPROM.h"

// REPLACE WITH THE MAC Address of your receiver
static uint8_t mac_list[2][6] = {
    {0x24, 0x62, 0xAB, 0xE1, 0xAF, 0x34},   // 24:62:AB:E1:AF:34
    {0x24, 0x62, 0xAB, 0xE1, 0xB1, 0x5C}};  // 24:62:AB:E1:B1:5C
static const uint8_t *mac_to_send = mac_list[0];
static uint8_t my_mac[6];

static esp_now_peer_info_t peer_info;

#define BUFFER_SIZE 1

static bool data_arrived = false;
static uint8_t received_data[BUFFER_SIZE];
static size_t received_data_length = 0;

const int led_pin = LED_BUILTIN;
const int key_pin = 0;

enum led_state_t : bool { LED_OFF = false, LED_ON = true };
static led_state_t led_state = LED_OFF;

void printMAC(const uint8_t *mac) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2],
                mac[3], mac[4], mac[5]);
}

void initMAC(void) {
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, my_mac);
  if (ret == ESP_OK) {
    if (memcmp(my_mac, mac_list[0], sizeof(mac_list[0])) == 0) {
      mac_to_send = mac_list[1];
    }
  } else {
    Serial.println("Error reading MAC Address");
  }

  Serial.print("My MAC: ");
  printMAC(my_mac);
  Serial.print("Sending to MAC: ");
  printMAC(mac_to_send);
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success"
                                                : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int data_len) {
  memcpy(received_data, data, sizeof(received_data));
  received_data_length = data_len;
  data_arrived = true;
  // Recebeu os dados e salvou em receivedData
}

void initEspNow(void) {
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peer_info.peer_addr, mac_to_send, 6);
  peer_info.channel = 0;
  peer_info.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peer_info) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is
  // received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void setup() {
  Serial.begin(115200);

  pinMode(key_pin, INPUT_PULLUP);
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, led_state);

  WiFi.mode(WIFI_STA);
  initMAC();
  initEspNow();
}

bool clickedKey(void) {
  static bool released = false;
  static uint8_t filter = 0x55;

  filter *= 2;
  filter += digitalRead(key_pin);  // 0 = pressed, 1 = released

  if (filter == 0) {
    if (released) {
      released = false;
      return true;
    }
  } else if (filter == 0xff) {
    released = true;
  }
  return false;
}

void loop() {
  // Set values to send
  static uint8_t data_to_send[BUFFER_SIZE];

  if (clickedKey()) {
    data_to_send[0]++;

    Serial.print("Sending: ");
    Serial.println(data_to_send[0]);

    // Send message via ESP-NOW
    esp_err_t result =
        esp_now_send(mac_to_send, data_to_send, sizeof(data_to_send));

    if (result == ESP_OK) {
      Serial.println("Sent with success");
    } else {
      Serial.println("Error sending the data");
    }
  }
  if (data_arrived) {
    data_arrived = false;

    Serial.print("Received: ");
    Serial.println(received_data[0]);

    if (led_state == LED_OFF)
      led_state = LED_ON;
    else
      led_state = LED_OFF;

    digitalWrite(led_pin, led_state);
  }
}
