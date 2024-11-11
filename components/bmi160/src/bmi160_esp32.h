#ifndef BMI160_ESP32_H_
#define BMI160_ESP32_H_

#include "esp_err.h"
#include "bmi160.h"
#include "bmi160_esp32_defs.h"

void init_sensor_interface(void);
/*!
 * \ingroup bmi160-esp32API
 * \page 
 * \code
 * void init_sensor_interface(void) 
 * \endcode
 * @details 
 * 
 *
 *
 * @return Result of API execution status
 * @retval Zero Success
 * @retval Negative Error
 */

esp_err_t init_bmi160(struct bmi160_dev* bmi160dev); 

/*!
 * \ingroup bmi160-esp32API
 * \page 
 * \code
 * void init_sensor_interface(void) 
 * \endcode
 * @details This API configures the sensor via I2C. 
 *  
 * @param Device Adress.
 *
 * @return Result of API execution status
 * @retval Zero Success - esp_err_t struct with field OK
 * @retval esp_err_t with anything else than OK in its enum.
 */

/**
 * \ingroup bmi16-esp32API
 * \page
 * \code 
 * void bmi160_i2c_w
 * \encode
 * @details Write wrapper function for i2c.
 *
 * @param dev_addr
 * @param reg_addr
 * @param data
 * @param len
 *
 * @return 
 */
int8_t bmi160_i2c_w(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);



/**
 * \ingroup bmi16-esp32API
 * \page
 * \code 
 * void bmi160_i2c_w
 * \encode
 * @details Read wrapper function for i2c.
 * @param dev_addr
 * @param reg_addr
 * @param data
 * @param len
 *
 * @return 
 */
int8_t bmi160_i2c_r(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);


#endif 
