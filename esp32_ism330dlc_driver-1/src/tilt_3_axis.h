#ifndef __TILT_3_AXIS_H__
#define __TILT_3_AXIS_H__

#include "ism330dlc_driver.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"

typedef enum {
    TILT_3_OK = 0,
    TILT_3_ERROR = -1
} tilt_3_axis_retval_t;

typedef struct {
    float pitch = 0.0f;
    float roll  = 0.0f;
} tilt_3_axis_offset_t;

class Tilt3Axis {
public:
    Tilt3Axis();
	
    tilt_3_axis_retval_t init(i2c_port_t i2c, uint8_t address);
    tilt_3_axis_retval_t init(spi_device_handle_t* spi, int cs_pin);
    tilt_3_axis_retval_t stop();
    tilt_3_axis_retval_t getSensorId(uint8_t& id);
    tilt_3_axis_retval_t getTemperature(float& temperature);
    tilt_3_axis_retval_t getPitchRollAngle(float& pitch, float& roll);
    tilt_3_axis_retval_t calibrate(float set_pitch, float set_roll);
    tilt_3_axis_offset_t getOffset();
    void setOffset(tilt_3_axis_offset_t offset);

private:
	void calcPitchRoll(float* accel, float& pitch, float& roll);
	void calcPitchRollUpDown(float* accel, float& pitch, float& roll);

    Ism330dlcDriver* _pSensor;
	tilt_3_axis_offset_t _tilt_offset;
};

#endif
