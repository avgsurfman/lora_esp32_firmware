// Copyright Franciszek Moszczuk, MIT License
/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdint.h>

/*********************************************************************/
/* ESP32-IDF header files */
/*********************************************************************/
#include "sdkconfig.h"
#include "esp_types.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*********************************************************************/
/* own header files */
/*********************************************************************/

//#include "driver/i2c_slave.h"
#include "i2c.h"
#include "bmi160_esp32.h"

/*********************************************************************/
/* Macro definitions and config variables */
/*********************************************************************/
#define	SCL_PIN		CONFIG_BMI160_I2C_SCL
#define SDA_PIN		CONFIG_BMI160_I2C_SDA

// logging

/*********************************************************************/
/* global variables */
/*********************************************************************/
//TODO:wyrzucić do typedefa?

/*! @brief variable to hold the bmi160 accel data */
struct bmi160_sensor_data bmi160_accel;

/*! @brief variable to hold the bmi160 gyro data */
struct bmi160_sensor_data bmi160_gyro;


const char* BMI_TAG = "BMI160";

/*********************************************************************/
/* Static Function declarations */
/*********************************************************************/
 

/*********************************************************************/
/* Function declarations */
/*********************************************************************/


/*!
 * @brief   Mock-up function, the SDO pin is not soldered yet. This would 
 * normally be for selecting the default 0x68 address via changing 
 * the SDO pin to low (0).
 */
void init_sensor_interface(void);


/**
 * @brief Delay function used by the struct.
 *
 * @param ms - miliseconds of delay.
 */
void bmi160_delay_msec(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

/**
 * @brief Component I2C wrapper write function.
 *
 * @param dev_addr Device Adress
 * @param reg_addr Register Adress
 * @param data Data Pointer
 * @param len Lenght of the message (in bytes?)
 *
 * @return Status of the read/write function. 
 */
int8_t bmi160_i2c_w(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

/**
 * @brief Component I2C wrapper read function.
 *
 * @param dev_addr Device Adress
 * @param reg_addr Register Adress
 * @param data Data Pointer
 * @param len Lenght of the message (in bytes?)
 *
 * @return Status of the read/write function. 
 */
int8_t bmi160_i2c_r(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);


/********************************************************************/
/* Device setup */
/********************************************************************/

/*!
 *  @brief Call this to initialize the chip.
 *
 *  @param bmi160dev The device struct.
 *
 *  @return esp_err_t Error status of the operation.
 *
 */
esp_err_t init_bmi160(struct bmi160_dev* bmi160dev, struct bmi160_cfg* accelcfg, struct bmi160_cfg* gyrocfg)
{
    int8_t rslt;

    ESP_LOGD(BMI_TAG ,"Debug: started. Chip adress: BMI160_DEV_ADDR= %x \n", BMI160_DEV_ADDR);
    ESP_LOGD(BMI_TAG,"Null pointer check...");
    if (bmi160dev) {
	    ESP_LOGD(BMI_TAG ,"Clear. Reading from the struct: DEV_ADDR=%x \n", bmi160dev->id);
    }
    else {
	    ESP_LOGE(BMI_TAG,"NULL POINTER EXCEPTION \n");
	    return ESP_ERR_NOT_FOUND;
    }
    rslt = bmi160_init(bmi160dev);

    // I2C's the default for now, Check struct's protocol field
    if (!bmi160dev->intf); else return ESP_FAIL;
    if (rslt == BMI160_OK)
    {
        ESP_LOGI(BMI_TAG,"BMI160 initialization success ! ");
        ESP_LOGI(BMI_TAG,"Chip ID 0x%X\n", bmi160dev->chip_id);
    }
    else
    {
        ESP_LOGE(BMI_TAG,"BMI160 initialization failure !\n ERROR: %d \n", rslt);
	return ESP_FAIL;
    }

    // Full config is passed
    if (accelcfg && gyrocfg) {
	    	/* ACCEL CONFIG */
    		/* Select the Output data rate, range of accelerometer sensor */
    		bmi160dev->accel_cfg.odr = accelcfg->odr;
    		bmi160dev->accel_cfg.range = accelcfg->range;
    		bmi160dev->accel_cfg.bw = accelcfg->bw;

    		/* Select the power mode of a */
    		bmi160dev->accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

		/* GYRO CONFIG */
    		/* Select the Output data rate, range of Gyroscope sensor */
    		bmi160dev->gyro_cfg.odr = gyrocfg->odr;
    		bmi160dev->gyro_cfg.range = gyrocfg->range;
    		bmi160dev->gyro_cfg.bw = gyrocfg->bw;

    		/* Select the power mode of Gyroscope sensor */
    		bmi160dev->gyro_cfg.power = gyrocfg->power;
    }
    else {
    	ESP_LOGW(BMI_TAG,"Config not passed, using default params...");

    	/* Select the Output data rate, range of accelerometer sensor */
    	bmi160dev->accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    	bmi160dev->accel_cfg.range = BMI160_ACCEL_RANGE_16G;
    	bmi160dev->accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    	/* Select the power mode of accelerometer sensor */
    	bmi160dev->accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    	/* Select the Output data rate, range of Gyroscope sensor */
    	bmi160dev->gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    	bmi160dev->gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    	bmi160dev->gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    	/* Select the power mode of Gyroscope sensor */
    	bmi160dev->gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    	/* Set the sensor configuration */
    	rslt = bmi160_set_sens_conf(bmi160dev);
    }
    // add interrupt
    return ESP_OK;
}

int8_t bmi160_i2c_w(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	return (int8_t)i2c_write(dev_addr, reg_addr, data, (size_t) len);
	// const doesn't work the same way as it does in cxx so it's unnecessary to re-cast
}


int8_t bmi160_i2c_r(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	return (int8_t)i2c_read(dev_addr, reg_addr, data, (size_t) len);
	// const doesn't work the same way as it does in cxx so it's unnecessary to re-cast
}


// Garbage as I2C went elsewhere, can be reused in the future 
void bmi160_i2c_init(void){
	ESP_LOGD(BMI_TAG, "Initializing I2C...");
	ESP_LOGE(BMI_TAG, "NOT IMPLEMENTED.");
	//ESP_ERROR_CHECK(i2c_new_slave_device(&bmi160_i2c_slv_config, &bmi160_i2c_slave));

}

static esp_err_t bmi160_i2c_end(void){ // TODO:
	//return i2c_del_slave_device(bmi160_i2c_slave);
	return ESP_OK;
}

void configure_bmi160(struct bmi160_dev* bmi160dev)
{
    /* link read/write/delay function of host system to appropriate
     * bmi160 function call prototypes */
    bmi160dev->write = bmi160_i2c_w; 
    bmi160dev->read = bmi160_i2c_r; 
    bmi160dev->delay_ms = bmi160_delay_msec; // Copied — hopefully works...

    bmi160dev->id = BMI160_DEV_ADDR;
    ESP_LOGD(BMI_TAG, "I2C Address: %x", BMI160_DEV_ADDR);
    bmi160dev->intf = BMI160_I2C_INTF;
}
