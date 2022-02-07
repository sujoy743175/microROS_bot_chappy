#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include <esp_log.h>
#include "sdkconfig.h"

#define WHEEL_STATE_GPIO GPIO_NUM_13


int Wheel_State()
{
  //Configure button
 
  gpio_set_direction(WHEEL_STATE_GPIO, GPIO_MODE_INPUT);
  printf("wheel_state pin configured\n");

  int wheel_state = gpio_get_level(WHEEL_STATE_GPIO);
  return wheel_state;  
}
