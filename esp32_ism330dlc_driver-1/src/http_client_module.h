#ifndef HTTP_CLIENT_MODULE_H
#define HTTP_CLIENT_MODULE_H

#include "esp_err.h"

esp_err_t send_data_to_server(float pitch, float roll, float pitchAverage,
                              float rollAverage, float tilt_left,
                              float tilt_right, float avgTilt,
                              float filteredTilt);

#endif
