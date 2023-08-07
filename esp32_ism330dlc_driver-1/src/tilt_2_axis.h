#ifndef __TILT_2_AXIS_H__
#define __TILT_2_AXIS_H__

#include "ism330dlc_driver.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"

typedef enum {
    TILT_2_OK = 0,
    TILT_2_ERROR = -1
} tilt_2_axis_retval_t;

typedef enum {
    TILT_LEFT = 0,
    TILT_RIGHT
} tilt_2_installation_t;

class Tilt2Axis {
public:
    Tilt2Axis();
	
    tilt_2_axis_retval_t init(i2c_port_t i2c, uint8_t address);
    tilt_2_axis_retval_t init(spi_device_handle_t* spi, int cs_pin);
    tilt_2_axis_retval_t stop();
    tilt_2_axis_retval_t getSensorId(uint8_t& id);
    tilt_2_axis_retval_t getTemperature(float& temperature);
    tilt_2_axis_retval_t getTiltAngle(float& angle, tilt_2_installation_t side);
    tilt_2_axis_retval_t calibrate(float set_value, tilt_2_installation_t side);
    float getOffset();
    void setOffset(float offset);

private:
    Ism330dlcDriver* _pSensor;
	float _tilt_offset = 0.0f;
};

#endif
