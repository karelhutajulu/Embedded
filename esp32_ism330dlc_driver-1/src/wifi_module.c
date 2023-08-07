#include "wifi_module.h"
#include "esp_event.h"
#include "esp_wifi.h"

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  }
}

esp_err_t wifi_init_sta() {
  // WiFi initialization code
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler,
                             NULL);

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = "s4tunol10",       // Modify this to your WiFi SSID
              .password = "brownie1967", // Modify this to your WiFi password
          },
  };

  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
  esp_wifi_start();

  return ESP_OK;
}
