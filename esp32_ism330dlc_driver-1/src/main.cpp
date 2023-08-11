#include "tilt_2_axis.h"
#include "tilt_3_axis.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#include <string.h>

#define I2C_MASTER_NUM              I2C_NUM_1					/*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000						/*!< I2C master clock frequency */
#define I2C_SLAVE_TX_BUF_DISABLE    0							/*!< I2C master doesn't need buffer */
#define I2C_SLAVE_RX_BUF_DISABLE    0							/*!< I2C master doesn't need buffer */
#define I2C_SLAVE_ADDR 				0b1101011					/* SDA -> HIGH */

#define I2C_SDA_PIN (GPIO_NUM_21)
#define I2C_SCL_PIN (GPIO_NUM_19)

static const char* TAG = "Main";

#define MEAN_SIZE 10
float meanBuffer[MEAN_SIZE];
size_t meanIndex = 0;

esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_PIN;
    conf.scl_io_num = I2C_SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE; /* must be enabled */
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE; /* must be enabled */
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_SLAVE_RX_BUF_DISABLE, I2C_SLAVE_TX_BUF_DISABLE, 0);
}

float mean(float inpVal, size_t& index, float* buffer, size_t length)
{
    buffer[index] = inpVal;
    index++;
    index %= length;

    float sum = 0;
    for (size_t i = 0; i < length; i++) {
        sum += buffer[i];
    }

    return sum / length;
}

float lpf(float oldVal, float newVal)
{
    return 0.8 * oldVal + 0.2 * newVal;
}

void calcPitchRoll(float* accel, float* output) {

	accel[0] = -accel[0];
	accel[1] = -accel[1];

	output[0] = -1 * atan2(accel[1], accel[2]) / M_PI * 180;
	output[1] = atan2(accel[0], accel[2]) / M_PI * 180;
}

void calcPitchRollUpDown(float* accel, float* output) {

	accel[1] = -accel[1];
	accel[2] = -accel[2];

	output[0] = -1 * atan2(accel[1], accel[2]) / M_PI * 180;
	output[1] = atan2(accel[0], accel[2]) / M_PI * 180;
} 

extern "C" void app_main()
{

        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(i2c_master_init());
        ESP_LOGI(TAG, "I2C initialized successfully");

        Tilt2Axis tilt2Axis;
	tilt2Axis.init(I2C_MASTER_NUM, I2C_SLAVE_ADDR);

	Tilt3Axis tilt3Axis;
	tilt3Axis.init(I2C_MASTER_NUM, I2C_SLAVE_ADDR);

    uint8_t sensorId = 0;
	tilt2Axis.getSensorId(sensorId);
    printf("ISM330DLC ID: %02X\n", sensorId);

	float temperature = 0;
    tilt2Axis.getTemperature(temperature);
	printf("temperature: %f 'C\n", temperature);

    float rollAverage = 0, pitchAverage = 0;
	float pitch = 0.0f, roll = 0.0f;

    float rollMeanBuffer[MEAN_SIZE];
    float pitchMeanBuffer[MEAN_SIZE];
    size_t roolMeanIndex = 0;
    size_t pitchMeanIndex = 0;

	float tilt_right, tilt_left, filteredTilt = 0, avgTilt = 0, oldTilt = 0;
	int counter = 0, counter_calibrate = 0;

    while (1) {
		if (tilt3Axis.getPitchRollAngle(pitch, roll) == TILT_3_OK) {
			pitchAverage = mean(pitch, pitchMeanIndex, pitchMeanBuffer, MEAN_SIZE);
            rollAverage = mean(roll, roolMeanIndex, rollMeanBuffer, MEAN_SIZE);
            vTaskDelay(100 / portTICK_PERIOD_MS);
                } else {
            vTaskDelay(10 / portTICK_PERIOD_MS);
                }

		printf("3 axis: %0.2f\t%0.2f\t%0.2f\t%0.2f\n", pitch, roll, pitchAverage, rollAverage);

        if (tilt2Axis.getTiltAngle(tilt_right, TILT_RIGHT) == TILT_2_OK) {
			avgTilt = mean(tilt_right, meanIndex, meanBuffer, MEAN_SIZE);
			filteredTilt = lpf(oldTilt, tilt_right);
			oldTilt = filteredTilt;
                        vTaskDelay(100 / portTICK_PERIOD_MS);
                } else {
                        vTaskDelay(10 / portTICK_PERIOD_MS);
                }

		if (tilt2Axis.getTiltAngle(tilt_left, TILT_LEFT) == TILT_2_OK) {
                        vTaskDelay(100 / portTICK_PERIOD_MS);
                } else {
                        vTaskDelay(10 / portTICK_PERIOD_MS);
                }

		// printf("2 axis: %0.3f\t%0.3f\t%0.3f\t%0.3f\n", tilt_left, tilt_right, avgTilt, filteredTilt);

		if (counter_calibrate == 50) {
			// tilt2Axis.calibrate(0.0, TILT_LEFT);
			tilt3Axis.calibrate(45.0, 45.0);
		}

		if (counter_calibrate == 100) {
			printf("offset value: %0.3f\n", tilt2Axis.getOffset());
			// tilt2Axis.setOffset(0.0f);
			tilt_3_axis_offset_t offset;
			offset.pitch = 0.0f;
			offset.roll = 0.0f;
			tilt3Axis.setOffset(offset);
		}


		counter++;
		counter_calibrate++;
    }
}