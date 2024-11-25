

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


/*! @brief Stores config in the balls */
/*
i2c_slave_config_t bmi160_i2c_slv_config = {
    		.addr_bit_len = I2C_ADDR_BIT_LEN_7,
    		.clk_source = I2C_CLK_SRC_DEFAULT,
		.i2c_port = -1, // autoselect the i2c controller
    		.send_buf_depth = 256,
    		.scl_io_num = SCL_PIN,
    		.sda_io_num = SDA_PIN,
    		.slave_addr = BMI160_DEV_ADDR,
};

*/

//allocate memory
//i2c_slave_dev_handle_t bmi160_i2c_slave;

static const char* TAG = "BMI160";

/*********************************************************************/
/* Static Function declarations */
/*********************************************************************/
 

/*********************************************************************/
/* Function declarations */
/*********************************************************************/


/*!
 * @brief   Mock-up function, SDO is not soldered yet. This would normally be for
 * setting up the Interrupts on the chip.
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
 *  @param[in] void
 *
 *  @return esp_err_t
 *
 */


esp_err_t init_bmi160(struct bmi160_dev* bmi160dev)
{
    int8_t rslt;

    ESP_LOGD(TAG ,"Debug: started. Chip adress: BMI160_DEV_ADDR: %x \n", BMI160_DEV_ADDR);
    ESP_LOGD(TAG,"Null pointer check...");
    if (bmi160dev) {
	    ESP_LOGD(TAG ,"Clear. Reading from the struct: DEV_ADDR=%x \n", bmi160dev->id);
    }
    else {
	    ESP_LOGE(TAG,"NULL POINTER EXCEPTION \n");
	    return ESP_ERR_NOT_FOUND;
    }
    rslt = bmi160_init(bmi160dev);

    // I2C's the default for now
    // Check struct's protocol field
    if (!bmi160dev->intf) bmi160_i2c_init(); else return ESP_FAIL;
    if (rslt == BMI160_OK)
    {
        ESP_LOGI(TAG,"BMI160 initialization success ! ");
        ESP_LOGI(TAG,"Chip ID 0x%X\n", bmi160dev->chip_id);
    }
    else
    {
        ESP_LOGE(TAG,"BMI160 initialization failure !\n ERROR: %d \n", rslt);
	return ESP_FAIL;
    }

    // TODO:ZMIENIĆ TO I PODLINKOWAĆ POD LOW-POWER MODE
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


// Garbage as I2C went elsewhere, can be reused as the main component 
/*! @brief: i2c init */
void bmi160_i2c_init(void){
	ESP_LOGD(TAG, "Initializing I2C...");

	//ESP_ERROR_CHECK(i2c_new_slave_device(&bmi160_i2c_slv_config, &bmi160_i2c_slave));

}

esp_err_t bmi160_i2c_end(void){ // TODO:
	//return i2c_del_slave_device(bmi160_i2c_slave);
	return ESP_OK;
}

