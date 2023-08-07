#include "tilt_3_axis.h"
#include "esp_log.h"

Tilt3Axis::Tilt3Axis() {}

tilt_3_axis_retval_t Tilt3Axis::init(i2c_port_t i2c, uint8_t address) {
  _pSensor = new Ism330dlcDriver(i2c, address);
  if (_pSensor->start() == ISM330DLC_OK) {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    return TILT_3_OK;
  } else {
    return TILT_3_ERROR;
  }
}

tilt_3_axis_retval_t Tilt3Axis::init(spi_device_handle_t *spi, int cs_pin) {
  _pSensor = new Ism330dlcDriver(spi, cs_pin);
  if (_pSensor->start() == ISM330DLC_OK) {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    return TILT_3_OK;
  } else {
    return TILT_3_ERROR;
  }
}

tilt_3_axis_retval_t Tilt3Axis::stop() {
  if (_pSensor->stop() == ISM330DLC_OK) {
    return TILT_3_OK;
  } else {
    return TILT_3_ERROR;
  }
}

tilt_3_axis_retval_t Tilt3Axis::getSensorId(uint8_t &id) {
  if (_pSensor->readId(&id) == ISM330DLC_OK) {
    return TILT_3_OK;
  } else {
    return TILT_3_ERROR;
  }
}

tilt_3_axis_retval_t Tilt3Axis::getTemperature(float &temperature) {
  if (_pSensor->getTemperature(&temperature) == ISM330DLC_OK) {
    return TILT_3_OK;
  } else {
    return TILT_3_ERROR;
  }
}

tilt_3_axis_retval_t Tilt3Axis::getPitchRollAngle(float &pitch, float &roll) {
  float acceleration[3] = {0};

  data_stat_t data_stat = _pSensor->getDataReadyStatus();
  if (data_stat.xlda) {
    _pSensor->get_X_Axes(acceleration);

    if (acceleration[2] < 0) {
      calcPitchRollUpDown(acceleration, pitch, roll);
      pitch -= _tilt_offset.pitch;
      roll -= _tilt_offset.roll;
    } else {
      calcPitchRoll(acceleration, pitch, roll);
      pitch -= _tilt_offset.pitch;
      roll -= _tilt_offset.roll;
    }

    return TILT_3_OK;
  } else {
    return TILT_3_ERROR;
  }
}

void Tilt3Axis::calcPitchRoll(float *accel, float &pitch, float &roll) {

  accel[0] = -accel[0];
  accel[1] = -accel[1];

  pitch = -1 * atan2(accel[1], accel[2]) / M_PI * 180;
  roll = atan2(accel[0], accel[2]) / M_PI * 180;
}

void Tilt3Axis::calcPitchRollUpDown(float *accel, float &pitch, float &roll) {

  accel[1] = -accel[1];
  accel[2] = -accel[2];

  pitch = -1 * atan2(accel[1], accel[2]) / M_PI * 180;
  roll = atan2(accel[0], accel[2]) / M_PI * 180;
}

tilt_3_axis_retval_t Tilt3Axis::calibrate(float set_pitch, float set_roll) {
  /* collect mean of each axis for 5s */
  uint8_t delay_in_sec = 5, data_num = delay_in_sec * 10;
  float pitch = 0, roll = 0, sum_pitch = 0.0, sum_roll = 0.0;

  for (uint8_t i = 0; i < data_num;) {
    if (getPitchRollAngle(pitch, roll) == TILT_3_OK) {
      i++;
      sum_pitch += pitch;
      sum_roll += roll;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }

  /* value = measurement - offset */
  /* offset = measurement - value */
  _tilt_offset.pitch = sum_pitch / data_num - set_pitch;
  _tilt_offset.roll = sum_roll / data_num - set_roll;

  return TILT_3_OK;
}

tilt_3_axis_offset_t Tilt3Axis::getOffset() { return _tilt_offset; }

void Tilt3Axis::setOffset(tilt_3_axis_offset_t offset) {
  _tilt_offset = offset;
}