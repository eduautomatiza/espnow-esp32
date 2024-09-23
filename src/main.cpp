/*****************************************************************************
 * @file main.cpp
 * @author Eduardo Guse
 * @brief Demostração de um projeto usando o protocolo esp-now bidirecional
 * @version 0.1
 * @date 2024-09-23
 *
 * @copyright Copyright (c) 2024
 *
 *****************************************************************************/
#include <WiFi.h>
#include <esp_now.h>
#include <esp_timer.h>
#include <esp_types.h>
#include <esp_wifi.h>

#include "Arduino.h"
#include "freertos/queue.h"

/*****************************************************************************
 * Define os pinos usados na placa
 *****************************************************************************/
const int led_pin = LED_BUILTIN;
const int key_pin = 0;

/*****************************************************************************
 * define tipos de variáveis
 *****************************************************************************/
enum led_state_t : bool { LED_OFF = false, LED_ON = true };

typedef struct {
  float_t value_1;
  float_t value_2;
  int32_t value_3;
  int32_t value_4;
} exchange_data_t;

typedef struct {
  int64_t time;
  uint8_t from[6];
  exchange_data_t data;
} queue_value_t;

/*****************************************************************************
 * Define as variáveis globais
 *****************************************************************************/
// REPLACE WITH THE MAC Address of your receiver
static uint8_t mac_list[2][6] = {
    {0x24, 0x62, 0xAB, 0xE1, 0xAF, 0x34},   // 24:62:AB:E1:AF:34
    {0x24, 0x62, 0xAB, 0xE1, 0xB1, 0x5C}};  // 24:62:AB:E1:B1:5C
static const uint8_t *mac_to_send = mac_list[0];
static uint8_t my_mac[6];

static esp_now_peer_info_t peer_info;
static led_state_t led_state = LED_OFF;
static QueueHandle_t received_values_queue;

/*****************************************************************************
 * Guarda as mensagens recebidas por callback para serem consumidas no loop
 *****************************************************************************/
void initEventsQueue(size_t n_elements) {
  received_values_queue = xQueueCreate(n_elements, sizeof(queue_value_t));
}

bool getEvent(queue_value_t *value, TickType_t xTicksToWait) {
  return received_values_queue
             ? xQueueReceive(received_values_queue, value, xTicksToWait)
             : false;
}

bool pushEvent(queue_value_t *value, TickType_t xTicksToWait) {
  return received_values_queue
             ? xQueueSend(received_values_queue, (void *)value, xTicksToWait)
             : false;
}

/*****************************************************************************
 *
 *****************************************************************************/

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
  if (data_len == sizeof(exchange_data_t)) {
    queue_value_t event;
    event.time = esp_timer_get_time();
    memcpy(&event.from, mac, sizeof(event.from));
    memcpy(&event.data, data, sizeof(exchange_data_t));
    pushEvent(&event, 0);
  }
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

  initEventsQueue(20);  // armazena até 20 valores na fila

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

void printExchangeData(const exchange_data_t &data) {
  Serial.print("Value1: ");
  Serial.println(data.value_1, 6);
  Serial.print("Value2: ");
  Serial.println(data.value_2, 6);
  Serial.print("Value3: ");
  Serial.println(data.value_3);
  Serial.print("Value4: ");
  Serial.println(data.value_4);
}

void loop() {
  if (clickedKey()) {
    static exchange_data_t data_to_send = {
        .value_1 = 3141.59265359,
        .value_2 = 0,
        .value_3 = 1000,
        .value_4 = -1000,
    };
    data_to_send.value_1 += 3.14159265359;
    data_to_send.value_2 -= 3.14159265359;
    data_to_send.value_3--;
    data_to_send.value_4++;

    Serial.println("Sending: ");
    printExchangeData(data_to_send);

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(mac_to_send, (uint8_t *)&data_to_send,
                                    sizeof(data_to_send));

    if (result == ESP_OK) {
      Serial.println("Sent with success");
    } else {
      Serial.println("Error sending the data");
    }
  }
  queue_value_t event;
  if (getEvent(&event, 0)) {
    int64_t now = esp_timer_get_time();

    Serial.print("From: ");
    printMAC(event.from);

    Serial.print("Delay: ");
    Serial.print(now - event.time);
    Serial.println("us");

    printExchangeData(event.data);

    if (led_state == LED_OFF)
      led_state = LED_ON;
    else
      led_state = LED_OFF;

    digitalWrite(led_pin, led_state);
  }
}
