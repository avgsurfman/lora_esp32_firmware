#ifndef BMI160_ESP32_H_
#define BMI160_ESP32_H_

#include "esp_err.h"
#include "bmi160.h"
#include "bmi160_esp32_defs.h"

void init_sensor_interface(void);
/*!
 * \ingroup bmi160-esp32API
 * \page 
 * \code init_sensor_interface(void);
 * \endcode
 * @details Pin set up for address. 
 * 
 *
 *
 */
void bmi160_delay_msec(uint32_t ms);
/*!
 * \ingroup bmi160-esp32API
 * \page 
 * \code bmi160_delay_msec(uint32_t ms);
 * void 
 * \endcode
 * @details Delay function pointer that acts as a callback whenever
 * the chip needs to rest.
 *
 *
 */

esp_err_t init_bmi160(struct bmi160_dev* bmi160dev, struct bmi160_cfg* accelcfg, struct bmi160_cfg* gyrocfg)

/*!
 * \ingroup bmi160-esp32API
 * \page 
 * \code
 * esp_err_t init_bmi160(struct bmi160_dev* bmi160dev); 
 * \endcode
 * @details Device initializer function. 
 *  
 * @param bmi160dev Device Struct Adress.
 *
 * @return Result of API execution status
 * @retval Zero Success - esp_err_t struct with field OK
 * @retval esp_err_t with anything else than OK in its enum.
 */



/**
 * \ingroup bmi16-esp32API
 * \page
 * \code 
 * int8_t bmi160_i2c_w
 * \encode
 * @details Write wrapper function for i2c.
 *
 * @param dev_addr
 * @param reg_addr
 * @param data
 * @param len
 *
 * @return Status of the write. 
 */
int8_t bmi160_i2c_w(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);



/**
 * \ingroup bmi16-esp32API
 * \page
 * \code 
 * int8_t bmi160_i2c_r
 * \encode
 * @details Read wrapper function for i2c.
 * @param dev_addr
 * @param reg_addr
 * @param data
 * @param len
 *
 * @return Dunno actually 
 */
int8_t bmi160_i2c_r(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);


void bmi160_i2c_init();

esp_err_t bmi160_i2c_end();

#endif 
