#include "lowPowerMode.h"
#include <esp_sleep.h>

void lowPowerMode_set_sleep_time(int sleep_time_sec)
{
    printf("Enabling timer wakeup, %ds\n", sleep_time_sec);
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(sleep_time_sec * 1000000));
}

void lowPowerMode_enter_deep_sleep(void)
{
    printf("Entering low power mode\n");
    lowPowerMode_set_sleep_time(CONFIG_LOW_POWER_MODE_SLEEP_TIME_SEC);
    esp_deep_sleep_start();
}