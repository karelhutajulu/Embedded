#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <stdio.h>

#define LED_PIN 2

#define GATTS_TAG "GATTS_DEMO"

esp_gatt_char_prop_t character_properties =
    ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;

static uint8_t char_value[1] = {0}; // Value of the characteristic

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// GATT server callback function
static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param) {
  switch (event) {
  case ESP_GATTS_REG_EVT: {
    esp_ble_gap_set_device_name("ESP32 LED");
    esp_ble_gap_config_adv_data(&adv_data);
    break;
  }
  case ESP_GATTS_WRITE_EVT: {
    if (param->write.handle == 0x14) { // Handle of the characteristic
      if (param->write.len == 1) {
        char_value[0] = param->write.value[0];
        if (char_value[0] == 1) {
          gpio_set_level(LED_PIN, 1); // Turn on the LED
        } else {
          gpio_set_level(LED_PIN, 0); // Turn off the LED
        }
      }
    }
    break;
  }
  default:
    break;
  }
}

void gap_event_handler(esp_gap_ble_cb_event_t event,
                       esp_ble_gap_cb_param_t *param) {
  if (event == ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT) {
    esp_ble_gap_start_advertising(&adv_params);
  }
}

void app_main(void) {
  nvs_flash_init();
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  esp_bt_controller_init(&bt_cfg);
  esp_bt_controller_enable(ESP_BT_MODE_BLE);
  esp_bluedroid_init();
  esp_bluedroid_enable();
  esp_ble_gap_register_callback(gap_event_handler);
  esp_ble_gatts_register_callback(gatts_profile_event_handler);
  esp_ble_gatts_app_register(0);

  gpio_pad_select_gpio(LED_PIN);
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

  while (1) {
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
