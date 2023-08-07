#include "ism330dlc_driver.h"
#include "esp_log.h"
#include <bits/stdc++.h>
#include <string.h>

#define BOOT_TIME 15 // ms
static const char *TAG = "ISM330DLC Driver";

/* Class Implementation ------------------------------------------------------*/
/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
Ism330dlcDriver::Ism330dlcDriver(i2c_port_t i2c, uint8_t address)
    : _pI2C(i2c), _address(address) {
  _pSPI = NULL;
  _is_x_enabled = false;
  _is_g_enabled = false;

  _regCtx.write_reg = ism330dlc_io_write;
  _regCtx.read_reg = ism330dlc_io_read;
  _regCtx.handle = (void *)this;

  /* Wait sensor boot time */
  vTaskDelay(BOOT_TIME / portTICK_PERIOD_MS);
}

/** Constructor
 * @param spi object of an helper class which handles the SPI peripheral
 * @param cs_pin the chip select pin
 * @param spi_speed the SPI speed
 */
Ism330dlcDriver::Ism330dlcDriver(spi_device_handle_t *spi, int cs_pin)
    : _pSPI(spi) {
  _csPin = (gpio_num_t)cs_pin;
  _pI2C = -1;
  _address = 0U;
  _is_x_enabled = false;
  _is_g_enabled = false;

  _regCtx.write_reg = ism330dlc_io_write;
  _regCtx.read_reg = ism330dlc_io_read;
  _regCtx.handle = (void *)this;

  /* Wait sensor boot time */
  vTaskDelay(BOOT_TIME / portTICK_PERIOD_MS);
}

/**
 * @brief  Read ID of ISM330DLC Accelerometer and Gyroscope
 * @param  id the pointer where the ID of the device is stored
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::readId(uint8_t *id) {
  if (!id) {
    return ISM330DLC_ERROR;
  }

  if (ism330dlc_device_id_get(&_regCtx, id) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief  Configure the sensor in order to be used
 * @retval 0 in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::start() {
  if (_pSPI) {
    // Configure CS pin
    gpio_pad_select_gpio(_csPin);
    gpio_set_direction(_csPin, GPIO_MODE_OUTPUT);
    gpio_set_level(_csPin, 1);
  }

  _x_last_odr = ISM330DLC_XL_ODR_12Hz5;
  _g_last_odr = ISM330DLC_GY_ODR_12Hz5;
  _is_x_enabled = true;
  _is_g_enabled = true;

  /* Restore default configuration */
  if (ism330dlc_reset_set(&_regCtx, PROPERTY_ENABLE) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  uint8_t rst;
  do {
    ism330dlc_reset_get(&_regCtx, &rst);
  } while (rst);

  /* FIFO mode selection */
  if (ism330dlc_fifo_mode_set(&(_regCtx), ISM330DLC_BYPASS_MODE) !=
      ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  /* Enable register address automatically incremented during a multiple byte
   * access */
  if (ism330dlc_auto_increment_set(&(_regCtx), PROPERTY_ENABLE) !=
      ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  /* Enable Block Data Update */
  if (ism330dlc_block_data_update_set(&(_regCtx), PROPERTY_ENABLE) !=
      ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  /* Set Output Data Rate */
  if (ism330dlc_xl_data_rate_set(&(_regCtx), ISM330DLC_XL_ODR_104Hz) !=
      ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }
  if (ism330dlc_gy_data_rate_set(&(_regCtx), ISM330DLC_GY_ODR_104Hz) !=
      ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  /* Set full scale */
  if (ism330dlc_xl_full_scale_set(&(_regCtx), ISM330DLC_2g) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }
  if (ism330dlc_gy_full_scale_set(&(_regCtx), ISM330DLC_500dps) !=
      ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  /* Configure filtering chain(No aux interface) */
  /* Accelerometer - analog filter */
  if (ism330dlc_xl_filter_analog_set(&_regCtx, ISM330DLC_XL_ANA_BW_400Hz) !=
      ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }
  /* Accelerometer - LPF1 path ( LPF2 not used )*/
  if (ism330dlc_xl_lp1_bandwidth_set(&_regCtx, ISM330DLC_XL_LP1_ODR_DIV_4) !=
      ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  /* Accelerometer - LPF1 + LPF2 path */
  if (ism330dlc_xl_lp2_bandwidth_set(
          &_regCtx, ISM330DLC_XL_LOW_NOISE_LP_ODR_DIV_100) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }
  /* Accelerometer - High Pass / Slope path */
  // ism330dlc_xl_reference_mode_set(&_regCtx, PROPERTY_DISABLE);
  // ism330dlc_xl_hp_bandwidth_set(&_regCtx, ISM330DLC_XL_HP_ODR_DIV_100);
  /* Gyroscope - filtering chain */
  if (ism330dlc_gy_band_pass_set(&_regCtx, ISM330DLC_HP_260mHz_LP1_STRONG) !=
      ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief  Disable the sensor and relative resources
 * @retval 0 in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::stop() {
  /* Disable both acc and gyro */
  if (disable_X() != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  if (disable_G() != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  /* Reset CS configuration */
  if (_pSPI) {
    // Configure CS pin
    gpio_set_direction(_csPin, GPIO_MODE_INPUT);
  }

  return ISM330DLC_OK;
}

/**
 * @brief  Enable ISM330DLC Accelerator
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::enable_X(void) {
  /* Check if the component is already enabled */
  if (_is_x_enabled) {
    return ISM330DLC_OK;
  }

  /* Output data rate selection. */
  if (ism330dlc_xl_data_rate_set(&(_regCtx), _x_last_odr) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  _is_x_enabled = true;

  return ISM330DLC_OK;
}

/**
 * @brief  Enable ISM330DLC Gyroscope
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::enable_G(void) {
  /* Check if the component is already enabled */
  if (_is_g_enabled) {
    return ISM330DLC_OK;
  }

  /* Output data rate selection. */
  if (ism330dlc_gy_data_rate_set(&(_regCtx), _g_last_odr) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  _is_g_enabled = true;

  return ISM330DLC_OK;
}

/**
 * @brief  Disable ISM330DLC Accelerator
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::disable_X(void) {
  /* Check if the component is already disabled */
  if (!_is_x_enabled) {
    return ISM330DLC_OK;
  }

  /* Store actual output data rate. */
  if (ism330dlc_xl_data_rate_get(&(_regCtx), &_x_last_odr) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  /* Output data rate selection - power down. */
  if (ism330dlc_xl_data_rate_set(&(_regCtx), ISM330DLC_XL_ODR_OFF) !=
      ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  _is_x_enabled = false;

  return ISM330DLC_OK;
}

/**
 * @brief  Disable ISM330DLC Gyroscope
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::disable_G(void) {
  /* Check if the component is already disabled */
  if (!_is_g_enabled) {
    return ISM330DLC_OK;
  }

  /* Store actual output data rate. */
  if (ism330dlc_gy_data_rate_get(&(_regCtx), &_g_last_odr) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  /* Output data rate selection - power down */
  if (ism330dlc_gy_data_rate_set(&(_regCtx), ISM330DLC_GY_ODR_OFF) !=
      ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  _is_g_enabled = false;

  return ISM330DLC_OK;
}

/**
 * @brief  Read Accelerometer Sensitivity
 * @param  pfData the pointer where the accelerometer sensitivity is stored
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::get_X_Sensitivity(float *pfData) {
  ism330dlc_fs_xl_ois_t fullScale;

  /* Read actual full scale selection from sensor. */
  if (ism330dlc_aux_xl_full_scale_get(&_regCtx, &fullScale) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch (fullScale) {
  case ISM330DLC_AUX_2g:
    *pfData = 0.061f;
    break;
  case ISM330DLC_AUX_4g:
    *pfData = 0.122f;
    break;
  case ISM330DLC_AUX_8g:
    *pfData = 0.244f;
    break;
  case ISM330DLC_AUX_16g:
    *pfData = 0.488f;
    break;
  default:
    *pfData = 0.061f;
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief  Read Gyroscope Sensitivity
 * @param  pfData the pointer where the gyroscope sensitivity is stored
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::get_G_Sensitivity(float *pfData) {
  ism330dlc_fs_g_t fullScale;

  /* Read full scale 125 selection from sensor. */
  if (ism330dlc_gy_full_scale_get(&_regCtx, &fullScale) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch (fullScale) {
  case ISM330DLC_250dps:
    *pfData = 8.75f;
    break;
  case ISM330DLC_125dps:
    *pfData = 4.375f;
    break;
  case ISM330DLC_500dps:
    *pfData = 17.5f;
    break;
  case ISM330DLC_1000dps:
    *pfData = 35.0f;
    break;
  case ISM330DLC_2000dps:
    *pfData = 70.0f;
    break;
  default:
    *pfData = 8.75;
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief  Read ISM330DLC Accelerometer output data rate
 * @param  odr the pointer to the output data rate
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::get_X_ODR(float *odr) {
  ism330dlc_odr_xl_t odr_low_level;

  if (ism330dlc_xl_data_rate_get(&_regCtx, &odr_low_level) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  ism330dlc_xl_hm_mode_t xl_hm_mode;
  if (ism330dlc_xl_power_mode_get(&_regCtx, &xl_hm_mode) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  switch (odr_low_level) {
  case ISM330DLC_XL_ODR_OFF:
    *odr = 0.0f;
    break;
  case ISM330DLC_XL_ODR_1Hz6:
    if (xl_hm_mode == ISM330DLC_XL_HIGH_PERFORMANCE) {
      *odr = 12.5f;
    } else {
      *odr = 1.6f;
    }
    break;
  case ISM330DLC_XL_ODR_12Hz5:
    *odr = 12.5f;
    break;
  case ISM330DLC_XL_ODR_26Hz:
    *odr = 26.0f;
    break;
  case ISM330DLC_XL_ODR_52Hz:
    *odr = 52.0f;
    break;
  case ISM330DLC_XL_ODR_104Hz:
    *odr = 104.0f;
    break;
  case ISM330DLC_XL_ODR_208Hz:
    *odr = 208.0f;
    break;
  case ISM330DLC_XL_ODR_416Hz:
    *odr = 416.0f;
    break;
  case ISM330DLC_XL_ODR_833Hz:
    *odr = 833.0f;
    break;
  case ISM330DLC_XL_ODR_1k66Hz:
    *odr = 1660.0f;
    break;
  case ISM330DLC_XL_ODR_3k33Hz:
    *odr = 3330.0f;
    break;
  case ISM330DLC_XL_ODR_6k66Hz:
    *odr = 6660.0f;
    break;
  default:
    *odr = 12.5f;
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief  Read ISM330DLC Gyroscope output data rate
 * @param  odr the pointer to the output data rate
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::get_G_ODR(float *odr) {
  ism330dlc_odr_g_t odr_low_level;

  if (ism330dlc_gy_data_rate_get(&_regCtx, &odr_low_level) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  switch (odr_low_level) {
  case ISM330DLC_GY_ODR_OFF:
    *odr = 0.0f;
    break;
  case ISM330DLC_GY_ODR_12Hz5:
    *odr = 12.5f;
    break;
  case ISM330DLC_GY_ODR_26Hz:
    *odr = 26.0f;
    break;
  case ISM330DLC_GY_ODR_52Hz:
    *odr = 52.0f;
    break;
  case ISM330DLC_GY_ODR_104Hz:
    *odr = 104.0f;
    break;
  case ISM330DLC_GY_ODR_208Hz:
    *odr = 208.0f;
    break;
  case ISM330DLC_GY_ODR_416Hz:
    *odr = 416.0f;
    break;
  case ISM330DLC_GY_ODR_833Hz:
    *odr = 833.0f;
    break;
  case ISM330DLC_GY_ODR_1k66Hz:
    *odr = 1660.0f;
    break;
  case ISM330DLC_GY_ODR_3k33Hz:
    *odr = 3330.0f;
    break;
  case ISM330DLC_GY_ODR_6k66Hz:
    *odr = 6660.0f;
    break;
  default:
    *odr = -1.0f;
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief  Set ISM330DLC Accelerometer output data rate
 * @param  odr the output data rate to be set
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::set_X_ODR(float odr) {
  if (_is_x_enabled == 1) {
    if (set_X_ODR_When_Enabled(odr) == ISM330DLC_ERROR) {
      return ISM330DLC_ERROR;
    }
  } else {
    if (set_X_ODR_When_Disabled(odr) == ISM330DLC_ERROR) {
      return ISM330DLC_ERROR;
    }
  }

  return ISM330DLC_OK;
}

/**
 * @brief  Set ISM330DLC Accelerometer output data rate when enabled
 * @param  odr the output data rate to be set
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::set_X_ODR_When_Enabled(float odr) {
  ism330dlc_odr_xl_t new_odr;

  new_odr = (odr <= 1.6f)      ? ISM330DLC_XL_ODR_1Hz6
            : (odr <= 12.5f)   ? ISM330DLC_XL_ODR_12Hz5
            : (odr <= 26.0f)   ? ISM330DLC_XL_ODR_26Hz
            : (odr <= 52.0f)   ? ISM330DLC_XL_ODR_52Hz
            : (odr <= 104.0f)  ? ISM330DLC_XL_ODR_104Hz
            : (odr <= 208.0f)  ? ISM330DLC_XL_ODR_208Hz
            : (odr <= 416.0f)  ? ISM330DLC_XL_ODR_416Hz
            : (odr <= 833.0f)  ? ISM330DLC_XL_ODR_833Hz
            : (odr <= 1660.0f) ? ISM330DLC_XL_ODR_1k66Hz
            : (odr <= 3330.0f) ? ISM330DLC_XL_ODR_3k33Hz
                               : ISM330DLC_XL_ODR_6k66Hz;

  if (ism330dlc_xl_data_rate_set(&_regCtx, new_odr) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  _x_last_odr = new_odr;

  return ISM330DLC_OK;
}

/**
 * @brief  Set ISM330DLC Accelerometer output data rate when disabled
 * @param  odr the output data rate to be set
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::set_X_ODR_When_Disabled(float odr) {
  _x_last_odr = (odr <= 1.6f)      ? ISM330DLC_XL_ODR_1Hz6
                : (odr <= 12.5f)   ? ISM330DLC_XL_ODR_12Hz5
                : (odr <= 26.0f)   ? ISM330DLC_XL_ODR_26Hz
                : (odr <= 52.0f)   ? ISM330DLC_XL_ODR_52Hz
                : (odr <= 104.0f)  ? ISM330DLC_XL_ODR_104Hz
                : (odr <= 208.0f)  ? ISM330DLC_XL_ODR_208Hz
                : (odr <= 416.0f)  ? ISM330DLC_XL_ODR_416Hz
                : (odr <= 833.0f)  ? ISM330DLC_XL_ODR_833Hz
                : (odr <= 1660.0f) ? ISM330DLC_XL_ODR_1k66Hz
                : (odr <= 3330.0f) ? ISM330DLC_XL_ODR_3k33Hz
                                   : ISM330DLC_XL_ODR_6k66Hz;

  return ISM330DLC_OK;
}

/**
 * @brief  Set ISM330DLC Gyroscope output data rate
 * @param  odr the output data rate to be set
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::set_G_ODR(float odr) {
  if (_is_g_enabled == 1) {
    if (set_G_ODR_When_Enabled(odr) == ISM330DLC_ERROR) {
      return ISM330DLC_ERROR;
    }
  } else {
    if (set_G_ODR_When_Disabled(odr) == ISM330DLC_ERROR) {
      return ISM330DLC_ERROR;
    }
  }

  return ISM330DLC_OK;
}

/**
 * @brief  Set ISM330DLC Gyroscope output data rate when enabled
 * @param  odr the output data rate to be set
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::set_G_ODR_When_Enabled(float odr) {
  ism330dlc_odr_g_t new_odr;

  new_odr = (odr <= 12.5f)     ? ISM330DLC_GY_ODR_12Hz5
            : (odr <= 26.0f)   ? ISM330DLC_GY_ODR_26Hz
            : (odr <= 52.0f)   ? ISM330DLC_GY_ODR_52Hz
            : (odr <= 104.0f)  ? ISM330DLC_GY_ODR_104Hz
            : (odr <= 208.0f)  ? ISM330DLC_GY_ODR_208Hz
            : (odr <= 416.0f)  ? ISM330DLC_GY_ODR_416Hz
            : (odr <= 833.0f)  ? ISM330DLC_GY_ODR_833Hz
            : (odr <= 1660.0f) ? ISM330DLC_GY_ODR_1k66Hz
            : (odr <= 3330.0f) ? ISM330DLC_GY_ODR_3k33Hz
                               : ISM330DLC_GY_ODR_6k66Hz;

  if (ism330dlc_gy_data_rate_set(&_regCtx, new_odr) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  _g_last_odr = new_odr;

  return ISM330DLC_OK;
}

/**
 * @brief  Set ISM330DLC Gyroscope output data rate when disabled
 * @param  odr the output data rate to be set
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::set_G_ODR_When_Disabled(float odr) {
  _g_last_odr = (odr <= 12.5f)     ? ISM330DLC_GY_ODR_12Hz5
                : (odr <= 26.0f)   ? ISM330DLC_GY_ODR_26Hz
                : (odr <= 52.0f)   ? ISM330DLC_GY_ODR_52Hz
                : (odr <= 104.0f)  ? ISM330DLC_GY_ODR_104Hz
                : (odr <= 208.0f)  ? ISM330DLC_GY_ODR_208Hz
                : (odr <= 416.0f)  ? ISM330DLC_GY_ODR_416Hz
                : (odr <= 833.0f)  ? ISM330DLC_GY_ODR_833Hz
                : (odr <= 1660.0f) ? ISM330DLC_GY_ODR_1k66Hz
                : (odr <= 3330.0f) ? ISM330DLC_GY_ODR_3k33Hz
                                   : ISM330DLC_GY_ODR_6k66Hz;

  return ISM330DLC_OK;
}

/**
 * @brief  Read data from ISM330DLC Accelerometer
 * @param  pData the pointer where the accelerometer data are stored
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::get_X_Axes(float *pData) {
  int16_t dataRaw[3];
  float sensitivity = 0;

  /* Read raw data from ISM330DLC output register. */
  if (ism330dlc_acceleration_raw_get(&_regCtx, dataRaw) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  /* Get ISM330DLC actual sensitivity. */
  if (get_X_Sensitivity(&sensitivity) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  /* Calculate the data. */
  pData[0] = (float)(dataRaw[0] * sensitivity);
  pData[1] = (float)(dataRaw[1] * sensitivity);
  pData[2] = (float)(dataRaw[2] * sensitivity);

  if (_isUseCalibration) {
    pData[0] = (pData[0] - _calibration.offset_x) * _calibration.scale_x;
    pData[1] = (pData[1] - _calibration.offset_y) * _calibration.scale_y;
    pData[2] = (pData[2] - _calibration.offset_z) * _calibration.scale_z;
  }

  return ISM330DLC_OK;
}

/**
 * @brief  Read data from ISM330DLC Gyroscope
 * @param  pData the pointer where the gyroscope data are stored
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::get_G_Axes(float *pData) {
  int16_t dataRaw[3];
  float sensitivity = 0;

  /* Read raw data from ISM330DLC output register. */
  if (ism330dlc_angular_rate_raw_get(&_regCtx, dataRaw) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  /* Get ISM330DLC actual sensitivity. */
  if (get_G_Sensitivity(&sensitivity) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Calculate the data. */
  pData[0] = (float)(dataRaw[0] * sensitivity);
  pData[1] = (float)(dataRaw[1] * sensitivity);
  pData[2] = (float)(dataRaw[2] * sensitivity);

  return ISM330DLC_OK;
}

/**
 * @brief  Read raw data from ISM330DLC Accelerometer
 * @param  pData the pointer where the accelerometer raw data are stored
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::get_X_AxesRaw(int16_t *pData) {
  int16_t dataRaw[3];

  /* Read raw data values. */
  if (ism330dlc_acceleration_raw_get(&_regCtx, dataRaw) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  /* Format the data. */
  pData[0] = dataRaw[0];
  pData[1] = dataRaw[1];
  pData[2] = dataRaw[2];

  return ISM330DLC_OK;
}

/**
 * @brief  Read raw data from ISM330DLC Gyroscope
 * @param  pData the pointer where the gyroscope raw data are stored
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::get_G_AxesRaw(int16_t *pData) {
  int16_t dataRaw[3];

  /* Read raw data values. */
  if (ism330dlc_angular_rate_raw_get(&_regCtx, dataRaw) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  /* Format the data. */
  pData[0] = dataRaw[0];
  pData[1] = dataRaw[1];
  pData[2] = dataRaw[2];

  return ISM330DLC_OK;
}

/**
 * @brief  Read ISM330DLC Accelerometer full scale
 * @param  fullScale the pointer to the full scale
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::get_X_FS(float *fullScale) {
  ism330dlc_fs_xl_t fs_low_level;

  if (ism330dlc_xl_full_scale_get(&_regCtx, &fs_low_level) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  switch (fs_low_level) {
  case ISM330DLC_2g:
    *fullScale = 2.0f;
    break;
  case ISM330DLC_4g:
    *fullScale = 4.0f;
    break;
  case ISM330DLC_8g:
    *fullScale = 8.0f;
    break;
  case ISM330DLC_16g:
    *fullScale = 16.0f;
    break;
  default:
    *fullScale = -1.0f;
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief  Read ISM330DLC Gyroscope full scale
 * @param  fullScale the pointer to the full scale
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::get_G_FS(float *fullScale) {
  ism330dlc_fs_g_t fs_low_level;

  if (ism330dlc_gy_full_scale_get(&_regCtx, &fs_low_level) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  switch (fs_low_level) {
  case ISM330DLC_125dps:
    *fullScale = 125.0f;
    break;
  case ISM330DLC_250dps:
    *fullScale = 250.0f;
    break;
  case ISM330DLC_500dps:
    *fullScale = 500.0f;
    break;
  case ISM330DLC_1000dps:
    *fullScale = 1000.0f;
    break;
  case ISM330DLC_2000dps:
    *fullScale = 2000.0f;
    break;
  default:
    *fullScale = -1.0f;
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief  Set ISM330DLC Accelerometer full scale
 * @param  fullScale the full scale to be set
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::set_X_FS(float fullScale) {
  ism330dlc_fs_xl_t new_fs;

  new_fs = (fullScale <= 2.0f)   ? ISM330DLC_2g
           : (fullScale <= 4.0f) ? ISM330DLC_4g
           : (fullScale <= 8.0f) ? ISM330DLC_8g
                                 : ISM330DLC_16g;

  if (ism330dlc_xl_full_scale_set(&_regCtx, new_fs) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief  Set ISM330DLC Gyroscope full scale
 * @param  fullScale the full scale to be set
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::set_G_FS(float fullScale) {
  ism330dlc_fs_g_t new_fs;

  new_fs = (fullScale <= 125.0f)    ? ISM330DLC_125dps
           : (fullScale <= 250.0f)  ? ISM330DLC_250dps
           : (fullScale <= 500.0f)  ? ISM330DLC_500dps
           : (fullScale <= 1000.0f) ? ISM330DLC_1000dps
                                    : ISM330DLC_2000dps;

  if (ism330dlc_gy_full_scale_set(&_regCtx, new_fs) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

void Ism330dlcDriver::calcPitchRoll(float *accel, float *output) {
  if (accel[2] < 0) { /* upside down */
    accel[1] = -accel[1];
    accel[2] = -accel[2];

    output[0] = -1 * atan2(accel[1], accel[2]) / M_PI * 180;
    output[1] = atan2(accel[0], accel[2]) / M_PI * 180;
  } else { /* upside up */
    accel[0] = -accel[0];
    accel[1] = -accel[1];

    output[0] = -1 * atan2(accel[1], accel[2]) / M_PI * 180;
    output[1] = atan2(accel[0], accel[2]) / M_PI * 180;
  }
}

void accel_calibration_task(void *arg) {
  Ism330dlcDriver *pIsm330dlcDriver = (Ism330dlcDriver *)arg;
  pIsm330dlcDriver->x_buffer.clear();
  pIsm330dlcDriver->y_buffer.clear();
  pIsm330dlcDriver->z_buffer.clear();
  float accel[3] = {0};
  data_stat_t data_stat;

  /* clear FIFO */
  for (uint8_t i = 0; i < 3; i++) {
    pIsm330dlcDriver->get_X_Axes(accel);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }

  /* collect all data of each axis until  */
  while (!pIsm330dlcDriver->isStopCalibration) {
    data_stat = pIsm330dlcDriver->getDataReadyStatus();
    if (data_stat.xlda) {
      pIsm330dlcDriver->get_X_Axes(accel);
      pIsm330dlcDriver->x_buffer.push_back(accel[0]);
      pIsm330dlcDriver->y_buffer.push_back(accel[1]);
      pIsm330dlcDriver->z_buffer.push_back(accel[2]);
      if (pIsm330dlcDriver->x_buffer.size() >= 1200) {
        ESP_LOGE(TAG, "########## TIMEOUT ##########\n");
        break; /* stop collect data to prevent stack overflow */ /* 800 / 20 =
                                                                    40s */
      }
      // printf("%0.3f,%0.3f,%0.3f\n", accel[0], accel[1], accel[2]);
      vTaskDelay(50 / portTICK_PERIOD_MS);
    } else {
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }
  vTaskDelete(NULL);
}

ism330dlc_retval_t Ism330dlcDriver::startCalibrate() {
  isStopCalibration = false;
  _isUseCalibration = false;
  xTaskCreate(accel_calibration_task, "accel_calibration_task", 2 * 1024, this,
              10, &(this->taskHandle));

  return ISM330DLC_OK;
}

ism330dlc_retval_t Ism330dlcDriver::stopCalibrate() {
  isStopCalibration = true;
  uint8_t counter = 0;
  vTaskDelay(100 / portTICK_PERIOD_MS);

  eTaskState task_state;
  do {
    counter++;
    if (counter > 10) {
      break;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    task_state = eTaskGetState(taskHandle);
  } while (task_state != 4);

  float x_max = *std::max_element(std::begin(x_buffer), std::end(x_buffer));
  float x_min = *std::min_element(std::begin(x_buffer), std::end(x_buffer));
  float y_max = *std::max_element(std::begin(y_buffer), std::end(y_buffer));
  float y_min = *std::min_element(std::begin(y_buffer), std::end(y_buffer));
  float z_max = *std::max_element(std::begin(z_buffer), std::end(z_buffer));
  float z_min = *std::min_element(std::begin(z_buffer), std::end(z_buffer));

  _calibration.offset_x = (x_max + x_min) / 2;
  _calibration.offset_y = (y_max + y_min) / 2;
  _calibration.offset_z = (z_max + z_min) / 2;

  float avg_delta_x = (x_max - x_min) / 2;
  float avg_delta_y = (y_max - y_min) / 2;
  float avg_delta_z = (z_max - z_min) / 2;

  float avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3;

  _calibration.scale_x = avg_delta / avg_delta_x;
  _calibration.scale_y = avg_delta / avg_delta_y;
  _calibration.scale_z = avg_delta / avg_delta_z;

  // printf("max_x: %0.3f\tmin_x: %0.3f\tmax_y: %0.3f\tmin_y: %0.3f\n", x_max,
  // x_min, y_max, y_min); printf("offset_x: %0.3f\toffset_y: %0.3f\n",
  // _x_offset, _y_offset);

  return ISM330DLC_OK;
}

void Ism330dlcDriver::setUseCalibration(bool use) { _isUseCalibration = use; }

void Ism330dlcDriver::getCalibrationNum(ism330dlc_calibration_t *calibration) {
  *calibration = _calibration;
}

void Ism330dlcDriver::setCalibrationNum(ism330dlc_calibration_t calibration) {
  _calibration = calibration;
}

/**
 * @brief Enable free fall detection
 * @note This function sets the ISM330DLC accelerometer ODR to 416Hz and the
 * ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::enable_Free_Fall_Detection(void) {
  return enable_Free_Fall_Detection(ISM330DLC_INT1_PIN);
}

/**
 * @brief Enable free fall detection
 * @param int_pin the interrupt pin to be used
 * @note This function sets the ISM330DLC accelerometer ODR to 416Hz and the
 * ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t
Ism330dlcDriver::enable_Free_Fall_Detection(ISM330DLC_Interrupt_Pin_t int_pin) {
  /* Output Data Rate selection */
  if (set_X_ODR(416.0f) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Full scale selection */
  if (ISM330DLC_ACC_GYRO_W_FS_XL((void *)this, ISM330DLC_ACC_GYRO_FS_XL_2g) ==
      MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* FF_DUR setting */
  if (ISM330DLC_ACC_GYRO_W_FF_Duration((void *)this, 0x06) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* WAKE_DUR setting */
  if (ISM330DLC_ACC_GYRO_W_WAKE_DUR((void *)this, 0x00) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* TIMER_HR setting */
  if (ISM330DLC_ACC_GYRO_W_TIMER_HR(
          (void *)this, ISM330DLC_ACC_GYRO_TIMER_HR_6_4ms) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* SLEEP_DUR setting */
  if (ISM330DLC_ACC_GYRO_W_SLEEP_DUR((void *)this, 0x00) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* FF_THS setting */
  if (ISM330DLC_ACC_GYRO_W_FF_THS(
          (void *)this, ISM330DLC_ACC_GYRO_FF_THS_312mg) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Enable basic Interrupts */
  if (ISM330DLC_ACC_GYRO_W_BASIC_INT(
          (void *)this, ISM330DLC_ACC_GYRO_BASIC_INT_ENABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Enable free fall event on either INT1 or INT2 pin */
  switch (int_pin) {
  case ISM330DLC_INT1_PIN:
    if (ISM330DLC_ACC_GYRO_W_FFEvOnInt1(
            (void *)this, ISM330DLC_ACC_GYRO_INT1_FF_ENABLED) == MEMS_ERROR) {
      return ISM330DLC_ERROR;
    }
    break;

  case ISM330DLC_INT2_PIN:
    if (ISM330DLC_ACC_GYRO_W_FFEvOnInt2(
            (void *)this, ISM330DLC_ACC_GYRO_INT2_FF_ENABLED) == MEMS_ERROR) {
      return ISM330DLC_ERROR;
    }
    break;

  default:
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Disable free fall detection
 * @param None
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::disable_Free_Fall_Detection(void) {
  /* Disable free fall event on INT1 pin */
  if (ISM330DLC_ACC_GYRO_W_FFEvOnInt1(
          (void *)this, ISM330DLC_ACC_GYRO_INT1_FF_DISABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Disable free fall event on INT2 pin */
  if (ISM330DLC_ACC_GYRO_W_FFEvOnInt2(
          (void *)this, ISM330DLC_ACC_GYRO_INT2_FF_DISABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Disable basic Interrupts */
  if (ISM330DLC_ACC_GYRO_W_BASIC_INT(
          (void *)this, ISM330DLC_ACC_GYRO_BASIC_INT_DISABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* FF_DUR setting */
  if (ISM330DLC_ACC_GYRO_W_FF_Duration((void *)this, 0x00) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* FF_THS setting */
  if (ISM330DLC_ACC_GYRO_W_FF_THS(
          (void *)this, ISM330DLC_ACC_GYRO_FF_THS_156mg) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Set the free fall detection threshold for ISM330DLC accelerometer
 * sensor
 * @param thr the threshold to be set
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::set_Free_Fall_Threshold(uint8_t thr) {

  if (ISM330DLC_ACC_GYRO_W_FF_THS(
          (void *)this, (ISM330DLC_ACC_GYRO_FF_THS_t)thr) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Enable the tilt detection for ISM330DLC accelerometer sensor
 * @note This function sets the ISM330DLC accelerometer ODR to 26Hz and the
 * ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::enable_Tilt_Detection(void) {
  return enable_Tilt_Detection(ISM330DLC_INT1_PIN);
}

/**
 * @brief Enable the tilt detection for ISM330DLC accelerometer sensor
 * @param int_pin the interrupt pin to be used
 * @note This function sets the ISM330DLC accelerometer ODR to 26Hz and the
 * ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t
Ism330dlcDriver::enable_Tilt_Detection(ISM330DLC_Interrupt_Pin_t int_pin) {
  /* Output Data Rate selection */
  if (set_X_ODR(26.0f) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Full scale selection. */
  if (set_X_FS(2.0f) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Enable embedded functionalities */
  if (ISM330DLC_ACC_GYRO_W_FUNC_EN(
          (void *)this, ISM330DLC_ACC_GYRO_FUNC_EN_ENABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Enable tilt calculation. */
  if (ISM330DLC_ACC_GYRO_W_TILT(
          (void *)this, ISM330DLC_ACC_GYRO_TILT_ENABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Enable tilt detection on either INT1 or INT2 pin */
  switch (int_pin) {
  case ISM330DLC_INT1_PIN:
    if (ISM330DLC_ACC_GYRO_W_TiltEvOnInt1(
            (void *)this, ISM330DLC_ACC_GYRO_INT1_TILT_ENABLED) == MEMS_ERROR) {
      return ISM330DLC_ERROR;
    }
    break;

  case ISM330DLC_INT2_PIN:
    if (ISM330DLC_ACC_GYRO_W_TiltEvOnInt2(
            (void *)this, ISM330DLC_ACC_GYRO_INT2_TILT_ENABLED) == MEMS_ERROR) {
      return ISM330DLC_ERROR;
    }
    break;

  default:
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Disable the tilt detection for ISM330DLC accelerometer sensor
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::disable_Tilt_Detection(void) {
  /* Disable tilt event on INT1. */
  if (ISM330DLC_ACC_GYRO_W_TiltEvOnInt1(
          (void *)this, ISM330DLC_ACC_GYRO_INT1_TILT_DISABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Disable tilt event on INT2. */
  if (ISM330DLC_ACC_GYRO_W_TiltEvOnInt2(
          (void *)this, ISM330DLC_ACC_GYRO_INT2_TILT_DISABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Disable tilt calculation. */
  if (ISM330DLC_ACC_GYRO_W_TILT(
          (void *)this, ISM330DLC_ACC_GYRO_TILT_DISABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Disable embedded functionalities */
  if (ISM330DLC_ACC_GYRO_W_FUNC_EN(
          (void *)this, ISM330DLC_ACC_GYRO_FUNC_EN_DISABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Enable the wake up detection for ISM330DLC accelerometer sensor
 * @note  This function sets the ISM330DLC accelerometer ODR to 416Hz and the
 * ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::enable_Wake_Up_Detection(void) {
  return enable_Wake_Up_Detection(ISM330DLC_INT2_PIN);
}

/**
 * @brief Enable the wake up detection for ISM330DLC accelerometer sensor
 * @param int_pin the interrupt pin to be used
 * @note  This function sets the ISM330DLC accelerometer ODR to 416Hz and the
 * ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t
Ism330dlcDriver::enable_Wake_Up_Detection(ISM330DLC_Interrupt_Pin_t int_pin) {
  /* Output Data Rate selection */
  if (set_X_ODR(416.0f) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Full scale selection. */
  if (set_X_FS(2.0f) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* WAKE_DUR setting */
  if (ISM330DLC_ACC_GYRO_W_WAKE_DUR((void *)this, 0x00) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Set wake up threshold. */
  if (ISM330DLC_ACC_GYRO_W_WK_THS((void *)this, 0x02) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Enable basic Interrupts */
  if (ISM330DLC_ACC_GYRO_W_BASIC_INT(
          (void *)this, ISM330DLC_ACC_GYRO_BASIC_INT_ENABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Enable wake up detection on either INT1 or INT2 pin */
  switch (int_pin) {
  case ISM330DLC_INT1_PIN:
    if (ISM330DLC_ACC_GYRO_W_WUEvOnInt1(
            (void *)this, ISM330DLC_ACC_GYRO_INT1_WU_ENABLED) == MEMS_ERROR) {
      return ISM330DLC_ERROR;
    }
    break;

  case ISM330DLC_INT2_PIN:
    if (ISM330DLC_ACC_GYRO_W_WUEvOnInt2(
            (void *)this, ISM330DLC_ACC_GYRO_INT2_WU_ENABLED) == MEMS_ERROR) {
      return ISM330DLC_ERROR;
    }
    break;

  default:
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Disable the wake up detection for ISM330DLC accelerometer sensor
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::disable_Wake_Up_Detection(void) {
  /* Disable wake up event on INT1 */
  if (ISM330DLC_ACC_GYRO_W_WUEvOnInt1(
          (void *)this, ISM330DLC_ACC_GYRO_INT1_WU_DISABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Disable wake up event on INT2 */
  if (ISM330DLC_ACC_GYRO_W_WUEvOnInt2(
          (void *)this, ISM330DLC_ACC_GYRO_INT2_WU_DISABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Disable basic Interrupts */
  if (ISM330DLC_ACC_GYRO_W_BASIC_INT(
          (void *)this, ISM330DLC_ACC_GYRO_BASIC_INT_DISABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* WU_DUR setting */
  if (ISM330DLC_ACC_GYRO_W_WAKE_DUR((void *)this, 0x00) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* WU_THS setting */
  if (ISM330DLC_ACC_GYRO_W_WK_THS((void *)this, 0x00) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Set the wake up threshold for ISM330DLC accelerometer sensor
 * @param thr the threshold to be set
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::set_Wake_Up_Threshold(uint8_t thr) {
  if (ISM330DLC_ACC_GYRO_W_WK_THS((void *)this, thr) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Enable the single tap detection for ISM330DLC accelerometer sensor
 * @note  This function sets the ISM330DLC accelerometer ODR to 416Hz and the
 * ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::enable_Single_Tap_Detection(void) {
  return enable_Single_Tap_Detection(ISM330DLC_INT1_PIN);
}

/**
 * @brief Enable the single tap detection for ISM330DLC accelerometer sensor
 * @param int_pin the interrupt pin to be used
 * @note  This function sets the ISM330DLC accelerometer ODR to 416Hz and the
 * ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::enable_Single_Tap_Detection(
    ISM330DLC_Interrupt_Pin_t int_pin) {
  /* Output Data Rate selection */
  if (set_X_ODR(416.0f) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Full scale selection. */
  if (set_X_FS(2.0f) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Enable X direction in tap recognition. */
  if (ISM330DLC_ACC_GYRO_W_TAP_X_EN(
          (void *)this, ISM330DLC_ACC_GYRO_TAP_X_EN_ENABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Enable Y direction in tap recognition. */
  if (ISM330DLC_ACC_GYRO_W_TAP_Y_EN(
          (void *)this, ISM330DLC_ACC_GYRO_TAP_Y_EN_ENABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Enable Z direction in tap recognition. */
  if (ISM330DLC_ACC_GYRO_W_TAP_Z_EN(
          (void *)this, ISM330DLC_ACC_GYRO_TAP_Z_EN_ENABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Set tap threshold. */
  if (set_Tap_Threshold(ISM330DLC_TAP_THRESHOLD_MID_LOW) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Set tap shock time window. */
  if (set_Tap_Shock_Time(ISM330DLC_TAP_SHOCK_TIME_MID_HIGH) ==
      ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Set tap quiet time window. */
  if (set_Tap_Quiet_Time(ISM330DLC_TAP_QUIET_TIME_MID_LOW) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* _NOTE_: Tap duration time window - don't care for single tap. */

  /* _NOTE_: Single/Double Tap event - don't care of this flag for single tap.
   */

  /* Enable basic Interrupts */
  if (ISM330DLC_ACC_GYRO_W_BASIC_INT(
          (void *)this, ISM330DLC_ACC_GYRO_BASIC_INT_ENABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Enable single tap on either INT1 or INT2 pin */
  switch (int_pin) {
  case ISM330DLC_INT1_PIN:
    if (ISM330DLC_ACC_GYRO_W_SingleTapOnInt1(
            (void *)this, ISM330DLC_ACC_GYRO_INT1_SINGLE_TAP_ENABLED) ==
        MEMS_ERROR) {
      return ISM330DLC_ERROR;
    }
    break;

  case ISM330DLC_INT2_PIN:
    if (ISM330DLC_ACC_GYRO_W_SingleTapOnInt2(
            (void *)this, ISM330DLC_ACC_GYRO_INT2_SINGLE_TAP_ENABLED) ==
        MEMS_ERROR) {
      return ISM330DLC_ERROR;
    }
    break;

  default:
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Disable the single tap detection for ISM330DLC accelerometer sensor
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::disable_Single_Tap_Detection(void) {
  /* Disable single tap interrupt on INT1 pin. */
  if (ISM330DLC_ACC_GYRO_W_SingleTapOnInt1(
          (void *)this, ISM330DLC_ACC_GYRO_INT1_SINGLE_TAP_DISABLED) ==
      MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Disable single tap interrupt on INT2 pin. */
  if (ISM330DLC_ACC_GYRO_W_SingleTapOnInt2(
          (void *)this, ISM330DLC_ACC_GYRO_INT2_SINGLE_TAP_DISABLED) ==
      MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Disable basic Interrupts */
  if (ISM330DLC_ACC_GYRO_W_BASIC_INT(
          (void *)this, ISM330DLC_ACC_GYRO_BASIC_INT_DISABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Reset tap threshold. */
  if (set_Tap_Threshold(0x0) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Reset tap shock time window. */
  if (set_Tap_Shock_Time(0x0) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Reset tap quiet time window. */
  if (set_Tap_Quiet_Time(0x0) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* _NOTE_: Tap duration time window - don't care for single tap. */

  /* _NOTE_: Single/Double Tap event - don't care of this flag for single tap.
   */

  /* Disable Z direction in tap recognition. */
  if (ISM330DLC_ACC_GYRO_W_TAP_Z_EN(
          (void *)this, ISM330DLC_ACC_GYRO_TAP_Z_EN_DISABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Disable Y direction in tap recognition. */
  if (ISM330DLC_ACC_GYRO_W_TAP_Y_EN(
          (void *)this, ISM330DLC_ACC_GYRO_TAP_Y_EN_DISABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Disable X direction in tap recognition. */
  if (ISM330DLC_ACC_GYRO_W_TAP_X_EN(
          (void *)this, ISM330DLC_ACC_GYRO_TAP_X_EN_DISABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Enable the double tap detection for ISM330DLC accelerometer sensor
 * @note  This function sets the ISM330DLC accelerometer ODR to 416Hz and the
 * ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::enable_Double_Tap_Detection(void) {
  return enable_Double_Tap_Detection(ISM330DLC_INT1_PIN);
}

/**
 * @brief Enable the double tap detection for ISM330DLC accelerometer sensor
 * @param int_pin the interrupt pin to be used
 * @note  This function sets the ISM330DLC accelerometer ODR to 416Hz and the
 * ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::enable_Double_Tap_Detection(
    ISM330DLC_Interrupt_Pin_t int_pin) {
  /* Output Data Rate selection */
  if (set_X_ODR(416.0f) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Full scale selection. */
  if (set_X_FS(2.0f) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Enable X direction in tap recognition. */
  if (ISM330DLC_ACC_GYRO_W_TAP_X_EN(
          (void *)this, ISM330DLC_ACC_GYRO_TAP_X_EN_ENABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Enable Y direction in tap recognition. */
  if (ISM330DLC_ACC_GYRO_W_TAP_Y_EN(
          (void *)this, ISM330DLC_ACC_GYRO_TAP_Y_EN_ENABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Enable Z direction in tap recognition. */
  if (ISM330DLC_ACC_GYRO_W_TAP_Z_EN(
          (void *)this, ISM330DLC_ACC_GYRO_TAP_Z_EN_ENABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Set tap threshold. */
  if (set_Tap_Threshold(ISM330DLC_TAP_THRESHOLD_MID_LOW) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Set tap shock time window. */
  if (set_Tap_Shock_Time(ISM330DLC_TAP_SHOCK_TIME_HIGH) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Set tap quiet time window. */
  if (set_Tap_Quiet_Time(ISM330DLC_TAP_QUIET_TIME_HIGH) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Set tap duration time window. */
  if (set_Tap_Duration_Time(ISM330DLC_TAP_DURATION_TIME_MID) ==
      ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Single and double tap enabled. */
  if (ISM330DLC_ACC_GYRO_W_SINGLE_DOUBLE_TAP_EV(
          (void *)this, ISM330DLC_ACC_GYRO_SINGLE_DOUBLE_TAP_DOUBLE_TAP) ==
      MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Enable basic Interrupts */
  if (ISM330DLC_ACC_GYRO_W_BASIC_INT(
          (void *)this, ISM330DLC_ACC_GYRO_BASIC_INT_ENABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Enable double tap on either INT1 or INT2 pin */
  switch (int_pin) {
  case ISM330DLC_INT1_PIN:
    if (ISM330DLC_ACC_GYRO_W_TapEvOnInt1(
            (void *)this, ISM330DLC_ACC_GYRO_INT1_DOUBLE_TAP_ENABLED) ==
        MEMS_ERROR) {
      return ISM330DLC_ERROR;
    }
    break;

  case ISM330DLC_INT2_PIN:
    if (ISM330DLC_ACC_GYRO_W_TapEvOnInt2(
            (void *)this, ISM330DLC_ACC_GYRO_INT2_DOUBLE_TAP_ENABLED) ==
        MEMS_ERROR) {
      return ISM330DLC_ERROR;
    }
    break;

  default:
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Disable the double tap detection for ISM330DLC accelerometer sensor
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::disable_Double_Tap_Detection(void) {
  /* Disable double tap interrupt on INT1 pin. */
  if (ISM330DLC_ACC_GYRO_W_TapEvOnInt1(
          (void *)this, ISM330DLC_ACC_GYRO_INT1_DOUBLE_TAP_DISABLED) ==
      MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Disable double tap interrupt on INT2 pin. */
  if (ISM330DLC_ACC_GYRO_W_TapEvOnInt2(
          (void *)this, ISM330DLC_ACC_GYRO_INT2_DOUBLE_TAP_DISABLED) ==
      MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Disable basic Interrupts */
  if (ISM330DLC_ACC_GYRO_W_BASIC_INT(
          (void *)this, ISM330DLC_ACC_GYRO_BASIC_INT_DISABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Reset tap threshold. */
  if (set_Tap_Threshold(0x0) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Reset tap shock time window. */
  if (set_Tap_Shock_Time(0x0) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Reset tap quiet time window. */
  if (set_Tap_Quiet_Time(0x0) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Reset tap duration time window. */
  if (set_Tap_Duration_Time(0x0) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Only single tap enabled. */
  if (ISM330DLC_ACC_GYRO_W_SINGLE_DOUBLE_TAP_EV(
          (void *)this, ISM330DLC_ACC_GYRO_SINGLE_DOUBLE_TAP_SINGLE_TAP) ==
      MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Disable Z direction in tap recognition. */
  if (ISM330DLC_ACC_GYRO_W_TAP_Z_EN(
          (void *)this, ISM330DLC_ACC_GYRO_TAP_Z_EN_DISABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Disable Y direction in tap recognition. */
  if (ISM330DLC_ACC_GYRO_W_TAP_Y_EN(
          (void *)this, ISM330DLC_ACC_GYRO_TAP_Y_EN_DISABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Disable X direction in tap recognition. */
  if (ISM330DLC_ACC_GYRO_W_TAP_X_EN(
          (void *)this, ISM330DLC_ACC_GYRO_TAP_X_EN_DISABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Set the tap threshold for ISM330DLC accelerometer sensor
 * @param thr the threshold to be set
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::set_Tap_Threshold(uint8_t thr) {
  if (ISM330DLC_ACC_GYRO_W_TAP_THS((void *)this, thr) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Set the tap shock time window for ISM330DLC accelerometer sensor
 * @param time the shock time window to be set
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::set_Tap_Shock_Time(uint8_t time) {
  if (ISM330DLC_ACC_GYRO_W_SHOCK_Duration((void *)this, time) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Set the tap quiet time window for ISM330DLC accelerometer sensor
 * @param time the quiet time window to be set
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::set_Tap_Quiet_Time(uint8_t time) {
  if (ISM330DLC_ACC_GYRO_W_QUIET_Duration((void *)this, time) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Set the tap duration of the time window for ISM330DLC accelerometer
 * sensor
 * @param time the duration of the time window to be set
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::set_Tap_Duration_Time(uint8_t time) {
  if (ISM330DLC_ACC_GYRO_W_DUR((void *)this, time) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Enable the 6D orientation detection for ISM330DLC accelerometer sensor
 * @note  This function sets the ISM330DLC accelerometer ODR to 416Hz and the
 * ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::enable_6D_Orientation(void) {
  return enable_6D_Orientation(ISM330DLC_INT1_PIN);
}

/**
 * @brief Enable the 6D orientation detection for ISM330DLC accelerometer sensor
 * @param int_pin the interrupt pin to be used
 * @note  This function sets the ISM330DLC accelerometer ODR to 416Hz and the
 * ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t
Ism330dlcDriver::enable_6D_Orientation(ISM330DLC_Interrupt_Pin_t int_pin) {
  /* Output Data Rate selection */
  if (set_X_ODR(416.0f) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Full scale selection. */
  if (set_X_FS(2.0f) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Set 6D threshold. */
  if (ISM330DLC_ACC_GYRO_W_SIXD_THS(
          (void *)this, ISM330DLC_ACC_GYRO_SIXD_THS_60_degree) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Enable basic Interrupts */
  if (ISM330DLC_ACC_GYRO_W_BASIC_INT(
          (void *)this, ISM330DLC_ACC_GYRO_BASIC_INT_ENABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Enable 6D orientation on either INT1 or INT2 pin */
  switch (int_pin) {
  case ISM330DLC_INT1_PIN:
    if (ISM330DLC_ACC_GYRO_W_6DEvOnInt1(
            (void *)this, ISM330DLC_ACC_GYRO_INT1_6D_ENABLED) == MEMS_ERROR) {
      return ISM330DLC_ERROR;
    }
    break;

  case ISM330DLC_INT2_PIN:
    if (ISM330DLC_ACC_GYRO_W_6DEvOnInt2(
            (void *)this, ISM330DLC_ACC_GYRO_INT2_6D_ENABLED) == MEMS_ERROR) {
      return ISM330DLC_ERROR;
    }
    break;

  default:
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Disable the 6D orientation detection for ISM330DLC accelerometer
 * sensor
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::disable_6D_Orientation(void) {
  /* Disable 6D orientation interrupt on INT1 pin. */
  if (ISM330DLC_ACC_GYRO_W_6DEvOnInt1(
          (void *)this, ISM330DLC_ACC_GYRO_INT1_6D_DISABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Disable 6D orientation interrupt on INT2 pin. */
  if (ISM330DLC_ACC_GYRO_W_6DEvOnInt2(
          (void *)this, ISM330DLC_ACC_GYRO_INT2_6D_DISABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Disable basic Interrupts */
  if (ISM330DLC_ACC_GYRO_W_BASIC_INT(
          (void *)this, ISM330DLC_ACC_GYRO_BASIC_INT_DISABLED) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  /* Reset 6D threshold. */
  if (ISM330DLC_ACC_GYRO_W_SIXD_THS(
          (void *)this, ISM330DLC_ACC_GYRO_SIXD_THS_80_degree) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Get the 6D orientation XL axis for ISM330DLC accelerometer sensor
 * @param xl the pointer to the 6D orientation XL axis
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::get_6D_Orientation_XL(uint8_t *xl) {
  ISM330DLC_ACC_GYRO_DSD_XL_t xl_raw;

  if (ISM330DLC_ACC_GYRO_R_DSD_XL((void *)this, &xl_raw) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  switch (xl_raw) {
  case ISM330DLC_ACC_GYRO_DSD_XL_DETECTED:
    *xl = 1;
    break;
  case ISM330DLC_ACC_GYRO_DSD_XL_NOT_DETECTED:
    *xl = 0;
    break;
  default:
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Get the 6D orientation XH axis for ISM330DLC accelerometer sensor
 * @param xh the pointer to the 6D orientation XH axis
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::get_6D_Orientation_XH(uint8_t *xh) {
  ISM330DLC_ACC_GYRO_DSD_XH_t xh_raw;

  if (ISM330DLC_ACC_GYRO_R_DSD_XH((void *)this, &xh_raw) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  switch (xh_raw) {
  case ISM330DLC_ACC_GYRO_DSD_XH_DETECTED:
    *xh = 1;
    break;
  case ISM330DLC_ACC_GYRO_DSD_XH_NOT_DETECTED:
    *xh = 0;
    break;
  default:
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Get the 6D orientation YL axis for ISM330DLC accelerometer sensor
 * @param yl the pointer to the 6D orientation YL axis
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::get_6D_Orientation_YL(uint8_t *yl) {
  ISM330DLC_ACC_GYRO_DSD_YL_t yl_raw;

  if (ISM330DLC_ACC_GYRO_R_DSD_YL((void *)this, &yl_raw) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  switch (yl_raw) {
  case ISM330DLC_ACC_GYRO_DSD_YL_DETECTED:
    *yl = 1;
    break;
  case ISM330DLC_ACC_GYRO_DSD_YL_NOT_DETECTED:
    *yl = 0;
    break;
  default:
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Get the 6D orientation YH axis for ISM330DLC accelerometer sensor
 * @param yh the pointer to the 6D orientation YH axis
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::get_6D_Orientation_YH(uint8_t *yh) {
  ISM330DLC_ACC_GYRO_DSD_YH_t yh_raw;

  if (ISM330DLC_ACC_GYRO_R_DSD_YH((void *)this, &yh_raw) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  switch (yh_raw) {
  case ISM330DLC_ACC_GYRO_DSD_YH_DETECTED:
    *yh = 1;
    break;
  case ISM330DLC_ACC_GYRO_DSD_YH_NOT_DETECTED:
    *yh = 0;
    break;
  default:
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Get the 6D orientation ZL axis for ISM330DLC accelerometer sensor
 * @param zl the pointer to the 6D orientation ZL axis
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::get_6D_Orientation_ZL(uint8_t *zl) {
  ISM330DLC_ACC_GYRO_DSD_ZL_t zl_raw;

  if (ISM330DLC_ACC_GYRO_R_DSD_ZL((void *)this, &zl_raw) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  switch (zl_raw) {
  case ISM330DLC_ACC_GYRO_DSD_ZL_DETECTED:
    *zl = 1;
    break;
  case ISM330DLC_ACC_GYRO_DSD_ZL_NOT_DETECTED:
    *zl = 0;
    break;
  default:
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Get the 6D orientation ZH axis for ISM330DLC accelerometer sensor
 * @param zh the pointer to the 6D orientation ZH axis
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::get_6D_Orientation_ZH(uint8_t *zh) {
  ISM330DLC_ACC_GYRO_DSD_ZH_t zh_raw;

  if (ISM330DLC_ACC_GYRO_R_DSD_ZH((void *)this, &zh_raw) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  switch (zh_raw) {
  case ISM330DLC_ACC_GYRO_DSD_ZH_DETECTED:
    *zh = 1;
    break;
  case ISM330DLC_ACC_GYRO_DSD_ZH_NOT_DETECTED:
    *zh = 0;
    break;
  default:
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Get the status of all hardware events for ISM330DLC accelerometer
 * sensor
 * @param status the pointer to the status of all hardware events
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t
Ism330dlcDriver::get_Event_Status(ISM330DLC_Event_Status_t *status) {
  uint8_t Wake_Up_Src = 0, Tap_Src = 0, D6D_Src = 0, Func_Src = 0, Md1_Cfg = 0,
          Md2_Cfg = 0, Int1_Ctrl = 0;

  memset((void *)status, 0x0, sizeof(ISM330DLC_Event_Status_t));

  if (readReg(ISM330DLC_ACC_GYRO_WAKE_UP_SRC, &Wake_Up_Src) ==
      ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  if (readReg(ISM330DLC_ACC_GYRO_TAP_SRC, &Tap_Src) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  if (readReg(ISM330DLC_ACC_GYRO_D6D_SRC, &D6D_Src) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  if (readReg(ISM330DLC_ACC_GYRO_FUNC_SRC1, &Func_Src) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  if (readReg(ISM330DLC_ACC_GYRO_MD1_CFG, &Md1_Cfg) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  if (readReg(ISM330DLC_ACC_GYRO_MD2_CFG, &Md2_Cfg) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  if (readReg(ISM330DLC_ACC_GYRO_INT1_CTRL, &Int1_Ctrl) == ISM330DLC_ERROR) {
    return ISM330DLC_ERROR;
  }

  if ((Md1_Cfg & ISM330DLC_ACC_GYRO_INT1_FF_MASK) ||
      (Md2_Cfg & ISM330DLC_ACC_GYRO_INT2_FF_MASK)) {
    if ((Wake_Up_Src & ISM330DLC_ACC_GYRO_FF_EV_STATUS_MASK)) {
      status->FreeFallStatus = 1;
    }
  }

  if ((Md1_Cfg & ISM330DLC_ACC_GYRO_INT1_WU_MASK) ||
      (Md2_Cfg & ISM330DLC_ACC_GYRO_INT2_WU_MASK)) {
    if ((Wake_Up_Src & ISM330DLC_ACC_GYRO_WU_EV_STATUS_MASK)) {
      status->WakeUpStatus = 1;
    }
  }

  if ((Md1_Cfg & ISM330DLC_ACC_GYRO_INT1_SINGLE_TAP_MASK) ||
      (Md2_Cfg & ISM330DLC_ACC_GYRO_INT2_SINGLE_TAP_MASK)) {
    if ((Tap_Src & ISM330DLC_ACC_GYRO_SINGLE_TAP_EV_STATUS_MASK)) {
      status->TapStatus = 1;
    }
  }

  if ((Md1_Cfg & ISM330DLC_ACC_GYRO_INT1_DOUBLE_TAP_MASK) ||
      (Md2_Cfg & ISM330DLC_ACC_GYRO_INT2_DOUBLE_TAP_MASK)) {
    if ((Tap_Src & ISM330DLC_ACC_GYRO_DOUBLE_TAP_EV_STATUS_MASK)) {
      status->DoubleTapStatus = 1;
    }
  }

  if ((Md1_Cfg & ISM330DLC_ACC_GYRO_INT1_6D_MASK) ||
      (Md2_Cfg & ISM330DLC_ACC_GYRO_INT2_6D_MASK)) {
    if ((D6D_Src & ISM330DLC_ACC_GYRO_D6D_EV_STATUS_MASK)) {
      status->D6DOrientationStatus = 1;
    }
  }

  if ((Md1_Cfg & ISM330DLC_ACC_GYRO_INT1_TILT_MASK) ||
      (Md2_Cfg & ISM330DLC_ACC_GYRO_INT2_TILT_MASK)) {
    if ((Func_Src & ISM330DLC_ACC_GYRO_TILT_EV_STATUS_MASK)) {
      status->TiltStatus = 1;
    }
  }

  return ISM330DLC_OK;
}

ism330dlc_retval_t Ism330dlcDriver::getTemperature(float *temperature) {
  int16_t data_raw;

  /* Read raw data values. */
  if (ism330dlc_temperature_raw_get(&_regCtx, &data_raw) != ISM330DLC_OK) {
    return ISM330DLC_ERROR;
  }

  /* Convert raw data to celcius */
  *temperature = ism330dlc_from_lsb_to_celsius(data_raw);

  return ISM330DLC_OK;
}

data_stat_t Ism330dlcDriver::getDataReadyStatus() {
  data_stat_t retVal;
  ism330dlc_status_reg_t status_reg;
  ism330dlc_status_reg_get(&_regCtx, &status_reg);

  retVal.gda = status_reg.gda;
  retVal.tda = status_reg.tda;
  retVal.xlda = status_reg.xlda;

  return retVal;
}

/**
 * @brief Read the data from register
 * @param reg register address
 * @param data register data
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::readReg(uint8_t reg, uint8_t *data) {

  if (ISM330DLC_ACC_GYRO_ReadReg((void *)this, reg, data, 1) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

/**
 * @brief Write the data to register
 * @param reg register address
 * @param data register data
 * @retval ISM330DLC_OK in case of success, an error code otherwise
 */
ism330dlc_retval_t Ism330dlcDriver::writeReg(uint8_t reg, uint8_t data) {

  if (ISM330DLC_ACC_GYRO_WriteReg((void *)this, reg, &data, 1) == MEMS_ERROR) {
    return ISM330DLC_ERROR;
  }

  return ISM330DLC_OK;
}

uint8_t Ism330dlcDriver::ioRead(uint8_t *pBuffer, uint8_t regAddr,
                                uint16_t len) {
  if (_pSPI) {
    spi_transaction_t t = {};
    t.addr = regAddr | 0x80;
    t.rxlength = len * 8; // receive data length is in bits
    t.rx_buffer = pBuffer;

    gpio_set_level(_csPin, 0);
    esp_err_t err = spi_device_transmit(*_pSPI, &t);
    gpio_set_level(_csPin, 1);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "sx126x_hal_read failed, err = %d", err);
      return 1;
    }

    return 0;
  }

  if (_pI2C != -1) {
    if (len == 0) {
      return 0;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, regAddr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
      i2c_master_read(cmd, pBuffer, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, pBuffer + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(_pI2C, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
      return 0;
    } else {
      return 1;
    }
  }

  return 1;
}

uint8_t Ism330dlcDriver::ioWrite(uint8_t *pBuffer, uint8_t regAddr,
                                 uint16_t len) {
  if (_pSPI) {
    spi_transaction_t t = {};
    t.addr = regAddr & 0x7F;
    t.rxlength = len * 8; // receive data length is in bits
    t.rx_buffer = pBuffer;

    gpio_set_level(_csPin, 0);
    esp_err_t err = spi_device_transmit(*_pSPI, &t);
    gpio_set_level(_csPin, 1);

    if (err != ESP_OK) {
      ESP_LOGE(TAG, "sx126x_hal_write failed, err = %d", err);
      return 1;
    }

    return 0;
  }

  if (_pI2C != -1) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, regAddr, true);
    i2c_master_write(cmd, pBuffer, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(_pI2C, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
      return 0;
    } else {
      return 1;
    }
  }

  return 1;
}

// uint8_t ISM330DLC_IO_Write(void* handle, uint8_t WriteAddr, uint8_t* pBuffer,
// uint16_t nBytesToWrite)
// {
//     return ((Ism330dlcDriver*)handle)->ioWrite(pBuffer, WriteAddr,
//     nBytesToWrite);
// }

// uint8_t ISM330DLC_IO_Read(void* handle, uint8_t ReadAddr, uint8_t* pBuffer,
// uint16_t nBytesToRead)
// {
//     return ((Ism330dlcDriver*)handle)->ioRead(pBuffer, ReadAddr,
//     nBytesToRead);
// }

int32_t ism330dlc_io_write(void *handle, uint8_t regAddr,
                           const uint8_t *pBuffer, uint16_t len) {
  return ((Ism330dlcDriver *)handle)->ioWrite((uint8_t *)pBuffer, regAddr, len);
}

int32_t ism330dlc_io_read(void *handle, uint8_t regAddr, uint8_t *pBuffer,
                          uint16_t len) {
  return ((Ism330dlcDriver *)handle)->ioRead(pBuffer, regAddr, len);
}