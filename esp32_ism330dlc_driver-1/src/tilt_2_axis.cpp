#include "tilt_2_axis.h"
#include "esp_log.h"

Tilt2Axis::Tilt2Axis() {}

tilt_2_axis_retval_t Tilt2Axis::init(i2c_port_t i2c, uint8_t address) {
  _pSensor = new Ism330dlcDriver(i2c, address);
  if (_pSensor->start() == ISM330DLC_OK) {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    return TILT_2_OK;
  } else {
    return TILT_2_ERROR;
  }
}

tilt_2_axis_retval_t Tilt2Axis::init(spi_device_handle_t *spi, int cs_pin) {
  _pSensor = new Ism330dlcDriver(spi, cs_pin);
  if (_pSensor->start() == ISM330DLC_OK) {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    return TILT_2_OK;
  } else {
    return TILT_2_ERROR;
  }
}

tilt_2_axis_retval_t Tilt2Axis::stop() {
  if (_pSensor->stop() == ISM330DLC_OK) {
    return TILT_2_OK;
  } else {
    return TILT_2_ERROR;
  }
}

tilt_2_axis_retval_t Tilt2Axis::getSensorId(uint8_t &id) {
  if (_pSensor->readId(&id) == ISM330DLC_OK) {
    return TILT_2_OK;
  } else {
    return TILT_2_ERROR;
  }
}

tilt_2_axis_retval_t Tilt2Axis::getTemperature(float &temperature) {
  if (_pSensor->getTemperature(&temperature) == ISM330DLC_OK) {
    return TILT_2_OK;
  } else {
    return TILT_2_ERROR;
  }
}

tilt_2_axis_retval_t Tilt2Axis::getTiltAngle(float &angle,
                                             tilt_2_installation_t side) {
  float acceleration[3] = {0};

  data_stat_t data_stat = _pSensor->getDataReadyStatus();
  if (data_stat.xlda) {
    _pSensor->get_X_Axes(acceleration);

    if (side == TILT_RIGHT) {
      angle = 90.0 + atan2(acceleration[0], acceleration[1]) * 180 / M_PI -
              _tilt_offset;
    } else {
      angle = -90 + atan2(-acceleration[0], acceleration[1]) * 180 / M_PI -
              _tilt_offset;
    }

    return TILT_2_OK;
  } else {
    return TILT_2_ERROR;
  }
}

tilt_2_axis_retval_t Tilt2Axis::calibrate(float set_value,
                                          tilt_2_installation_t side) {
  /* collect mean of each axis for 5s */
  uint8_t delay_in_sec = 5, data_num = delay_in_sec * 10;
  float tilt_angle = 0, sum_tilt = 0.0;

  for (uint8_t i = 0; i < data_num;) {
    if (getTiltAngle(tilt_angle, side) == TILT_2_OK) {
      i++;
      sum_tilt += tilt_angle;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }

  /* value = measurement - offset */
  /* offset = measurement - value */
  _tilt_offset = sum_tilt / data_num - set_value;

  return TILT_2_OK;
}

float Tilt2Axis::getOffset() { return _tilt_offset; }

void Tilt2Axis::setOffset(float offset) { _tilt_offset = offset; }