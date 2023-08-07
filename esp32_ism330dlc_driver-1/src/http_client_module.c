#include "http_client_module.h"
#include "esp_http_client.h"

esp_err_t send_data_to_server(float pitch, float roll, float pitchAverage,
                              float rollAverage, float tilt_left,
                              float tilt_right, float avgTilt,
                              float filteredTilt) {
  esp_http_client_config_t config = {
      .url = "http://your-server-url.com", // Replace with your server URL
  };

  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == NULL) {
    return ESP_FAIL;
  }

  char post_data[100]; // Adjust the buffer size as needed
  snprintf(post_data, sizeof(post_data),
           "pitch=%.2f&roll=%.2f&pitch_avg=%.2f&roll_avg=%.2f&tilt_left=%.2f&"
           "tilt_right=%.2f&avg_tilt=%.2f&filtered_tilt=%.2f",
           pitch, roll, pitchAverage, rollAverage, tilt_left, tilt_right,
           avgTilt, filteredTilt);

  esp_http_client_set_method(client, HTTP_METHOD_POST);
  esp_http_client_set_post_field(client, post_data, strlen(post_data));

  esp_err_t err = esp_http_client_perform(client);

  esp_http_client_cleanup(client);

  return err;
}
