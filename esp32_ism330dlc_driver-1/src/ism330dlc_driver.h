#ifndef __ISM330DLC_DRIVER_H__
#define __ISM330DLC_DRIVER_H__

#include "ISM330DLC_ACC_GYRO_Driver.h"
#include "ism330dlc_reg.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"

#include <vector>

#define ISM330DLC_TAP_THRESHOLD_MID_LOW 0x08

#define ISM330DLC_TAP_SHOCK_TIME_LOW 0x00 /**< Lowest  value of wake up threshold */
#define ISM330DLC_TAP_SHOCK_TIME_MID_LOW 0x01
#define ISM330DLC_TAP_SHOCK_TIME_MID_HIGH 0x02
#define ISM330DLC_TAP_SHOCK_TIME_HIGH 0x03 /**< Highest value of wake up threshold */

#define ISM330DLC_TAP_QUIET_TIME_LOW 0x00 /**< Lowest  value of wake up threshold */
#define ISM330DLC_TAP_QUIET_TIME_MID_LOW 0x01
#define ISM330DLC_TAP_QUIET_TIME_MID_HIGH 0x02
#define ISM330DLC_TAP_QUIET_TIME_HIGH 0x03 /**< Highest value of wake up threshold */

#define ISM330DLC_TAP_DURATION_TIME_LOW 0x00 /**< Lowest  value of wake up threshold */
#define ISM330DLC_TAP_DURATION_TIME_MID_LOW 0x04
#define ISM330DLC_TAP_DURATION_TIME_MID 0x08
#define ISM330DLC_TAP_DURATION_TIME_MID_HIGH 0x0C
#define ISM330DLC_TAP_DURATION_TIME_HIGH 0x0F /**< Highest value of wake up threshold */

typedef enum {
    ISM330DLC_OK = 0,
    ISM330DLC_ERROR = -1
} ism330dlc_retval_t;

typedef struct {
    float offset_x;
    float offset_y;
    float offset_z;
    float scale_x;
    float scale_y;
    float scale_z;
} ism330dlc_calibration_t;

typedef enum {
    ISM330DLC_INT1_PIN,
    ISM330DLC_INT2_PIN
} ISM330DLC_Interrupt_Pin_t;

typedef struct
{
    unsigned int FreeFallStatus : 1;
    unsigned int TapStatus : 1;
    unsigned int DoubleTapStatus : 1;
    unsigned int WakeUpStatus : 1;
    unsigned int TiltStatus : 1;
    unsigned int D6DOrientationStatus : 1;
} ISM330DLC_Event_Status_t;

typedef struct
{
	bool xlda;
	bool gda;
	bool tda;
} data_stat_t;

class Ism330dlcDriver {
public:
    Ism330dlcDriver(i2c_port_t i2c, uint8_t address);
    Ism330dlcDriver(spi_device_handle_t* spi, int cs_pin);
	
    ism330dlc_retval_t readId(uint8_t* id);
    ism330dlc_retval_t start(void);
    ism330dlc_retval_t stop(void);
    ism330dlc_retval_t enable_X(void);
    ism330dlc_retval_t enable_G(void);
    ism330dlc_retval_t disable_X(void);
    ism330dlc_retval_t disable_G(void);
    ism330dlc_retval_t get_X_Sensitivity(float* pfData);
    ism330dlc_retval_t get_G_Sensitivity(float* pfData);
    ism330dlc_retval_t get_X_ODR(float* odr);
    ism330dlc_retval_t get_G_ODR(float* odr);
    ism330dlc_retval_t set_X_ODR(float odr);
    ism330dlc_retval_t set_G_ODR(float odr);
    ism330dlc_retval_t get_X_Axes(float* pData);
    ism330dlc_retval_t get_G_Axes(float* pData);
    ism330dlc_retval_t get_X_AxesRaw(int16_t* pData);
    ism330dlc_retval_t get_G_AxesRaw(int16_t* pData);
    ism330dlc_retval_t get_X_FS(float* fullScale);
    ism330dlc_retval_t get_G_FS(float* fullScale);
    ism330dlc_retval_t set_X_FS(float fullScale);
    ism330dlc_retval_t set_G_FS(float fullScale);
	ism330dlc_retval_t startCalibrate();
	ism330dlc_retval_t stopCalibrate();
	void setUseCalibration(bool use);
	void getCalibrationNum(ism330dlc_calibration_t* calibration);
    void setCalibrationNum(ism330dlc_calibration_t calibration);
	
    ism330dlc_retval_t enable_Free_Fall_Detection(void);
    ism330dlc_retval_t enable_Free_Fall_Detection(ISM330DLC_Interrupt_Pin_t int_pin);
    ism330dlc_retval_t disable_Free_Fall_Detection(void);
    ism330dlc_retval_t set_Free_Fall_Threshold(uint8_t thr);
    ism330dlc_retval_t enable_Tilt_Detection(void);
    ism330dlc_retval_t enable_Tilt_Detection(ISM330DLC_Interrupt_Pin_t int_pin);
    ism330dlc_retval_t disable_Tilt_Detection(void);
    ism330dlc_retval_t enable_Wake_Up_Detection(void);
    ism330dlc_retval_t enable_Wake_Up_Detection(ISM330DLC_Interrupt_Pin_t int_pin);
    ism330dlc_retval_t disable_Wake_Up_Detection(void);
    ism330dlc_retval_t set_Wake_Up_Threshold(uint8_t thr);
    ism330dlc_retval_t enable_Single_Tap_Detection(void);
    ism330dlc_retval_t enable_Single_Tap_Detection(ISM330DLC_Interrupt_Pin_t int_pin);
    ism330dlc_retval_t disable_Single_Tap_Detection(void);
    ism330dlc_retval_t enable_Double_Tap_Detection(void);
    ism330dlc_retval_t enable_Double_Tap_Detection(ISM330DLC_Interrupt_Pin_t int_pin);
    ism330dlc_retval_t disable_Double_Tap_Detection(void);
    ism330dlc_retval_t set_Tap_Threshold(uint8_t thr);
    ism330dlc_retval_t set_Tap_Shock_Time(uint8_t time);
    ism330dlc_retval_t set_Tap_Quiet_Time(uint8_t time);
    ism330dlc_retval_t set_Tap_Duration_Time(uint8_t time);
    ism330dlc_retval_t enable_6D_Orientation(void);
    ism330dlc_retval_t enable_6D_Orientation(ISM330DLC_Interrupt_Pin_t int_pin);
    ism330dlc_retval_t disable_6D_Orientation(void);
    ism330dlc_retval_t get_6D_Orientation_XL(uint8_t* xl);
    ism330dlc_retval_t get_6D_Orientation_XH(uint8_t* xh);
    ism330dlc_retval_t get_6D_Orientation_YL(uint8_t* yl);
    ism330dlc_retval_t get_6D_Orientation_YH(uint8_t* yh);
    ism330dlc_retval_t get_6D_Orientation_ZL(uint8_t* zl);
    ism330dlc_retval_t get_6D_Orientation_ZH(uint8_t* zh);
    ism330dlc_retval_t get_Event_Status(ISM330DLC_Event_Status_t* status);

    ism330dlc_retval_t getTemperature(float* temperature);
    data_stat_t getDataReadyStatus();
    ism330dlc_retval_t readReg(uint8_t reg, uint8_t* data);
    ism330dlc_retval_t writeReg(uint8_t reg, uint8_t data);

    uint8_t ioRead(uint8_t* pBuffer, uint8_t regAddr, uint16_t len);
	uint8_t ioWrite(uint8_t* pBuffer, uint8_t regAddr, uint16_t len);

	bool isStopCalibration { true };
	std::vector<float> x_buffer;
	std::vector<float> y_buffer;
	std::vector<float> z_buffer;

private:
    ism330dlc_retval_t set_X_ODR_When_Enabled(float odr);
    ism330dlc_retval_t set_G_ODR_When_Enabled(float odr);
    ism330dlc_retval_t set_X_ODR_When_Disabled(float odr);
    ism330dlc_retval_t set_G_ODR_When_Disabled(float odr);
	void calcPitchRoll(float* accel, float* output);

    /* Helper classes. */
    i2c_port_t _pI2C;
    spi_device_handle_t* _pSPI;

    /* Configuration */
    uint8_t _address;
    gpio_num_t _csPin;

    bool _is_x_enabled;
    bool _is_g_enabled;
    ism330dlc_odr_xl_t _x_last_odr;
    ism330dlc_odr_g_t _g_last_odr;

	ism330dlc_calibration_t _calibration;
	bool _isUseCalibration { false };
	TaskHandle_t taskHandle = NULL;

    stmdev_ctx_t _regCtx;
};

#ifdef __cplusplus
extern "C" {
#endif
// uint8_t ISM330DLC_IO_Write(void* handle, uint8_t WriteAddr, uint8_t* pBuffer, uint16_t nBytesToWrite);
// uint8_t ISM330DLC_IO_Read(void* handle, uint8_t ReadAddr, uint8_t* pBuffer, uint16_t nBytesToRead);

int32_t ism330dlc_io_write(void* handle, uint8_t regAddr, const uint8_t* pBuffer, uint16_t len);
int32_t ism330dlc_io_read(void* handle, uint8_t regAddr, uint8_t* pBuffer, uint16_t len);
#ifdef __cplusplus
}
#endif

#endif