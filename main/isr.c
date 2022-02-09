#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include <esp_log.h>
#include "sdkconfig.h"

#define LEFT_LIMIT_GPIO GPIO_NUM_19
#define RIGHT_LIMIT_GPIO GPIO_NUM_15


void left_limit_switch()
{
  //Configure button
  gpio_config_t left_limit;
  left_limit.intr_type = GPIO_INTR_ANYEDGE; 	//Enable interrupt on both rising and falling edges
  left_limit.mode = GPIO_MODE_INPUT;        	//Set as Input
  left_limit.pin_bit_mask = (1 << LEFT_LIMIT_GPIO); //Bitmask
  left_limit.pull_up_en = GPIO_PULLUP_ENABLE; 	//Disable pullup DISABLE
  left_limit.pull_down_en = GPIO_PULLDOWN_DISABLE; //Enable pulldown ENABLE
  gpio_config(&left_limit);

  printf("Left limit pin configured\n");

}

int get_left_limit_state(){
  int left_limit_state = gpio_get_level(LEFT_LIMIT_GPIO);
  return left_limit_state; 

}


void right_limit_switch()
{
    //Configure button
  gpio_config_t right_limit;
  right_limit.intr_type = GPIO_INTR_ANYEDGE; 	//Enable interrupt on both rising and falling edges
  right_limit.mode = GPIO_MODE_INPUT;        	//Set as Input
  right_limit.pin_bit_mask = (1 << RIGHT_LIMIT_GPIO); //Bitmask
  right_limit.pull_up_en = GPIO_PULLUP_ENABLE; 	//Disable pullup DISABLE
  right_limit.pull_down_en = GPIO_PULLDOWN_DISABLE; //Enable pulldown ENABLE
  gpio_config(&right_limit);
  printf("Right limit pin configured\n"); 
}


int get_right_limit_state(){
  int right_limit_state = gpio_get_level(RIGHT_LIMIT_GPIO);
  return right_limit_state; 

}


/*
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include <esp_log.h>
#include "sdkconfig.h"

#define LED_GPIO GPIO_NUM_2
#define BUTTON_GPIO GPIO_NUM_0


int limit_switch()
{
  //Configure button
  gpio_config_t btn_config;
  btn_config.intr_type = GPIO_INTR_ANYEDGE; 	//Enable interrupt on both rising and falling edges
  btn_config.mode = GPIO_MODE_INPUT;        	//Set as Input
  btn_config.pin_bit_mask = (1 << BUTTON_GPIO); //Bitmask
  btn_config.pull_up_en = GPIO_PULLUP_DISABLE; 	//Disable pullup
  btn_config.pull_down_en = GPIO_PULLDOWN_ENABLE; //Enable pulldown
  gpio_config(&btn_config);
  printf("Button configured\n");

  int btn_state = gpio_get_level(BUTTON_GPIO);
  return btn_state;  
}*/
