#ifndef LOW_POWER_MODE_H
#define LOW_POWER_MODE_H

/**
 * @defgroup low_power_mode Low Power Mode
 *  @{
 */
void low_power_mode_enter_deep_sleep(void);

void low_power_mode_set_sleep_time(int sleep_time_sec);

#endif // LOW_POWER_MODE_H

/** @}*/
