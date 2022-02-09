/*
    IoT Simplified (iotsimplified.com)
    
    Sensor Driver for the HC-SR04 Ultrasonic Range Finding Sensor

    Written by Ahmed Al Bayati
    


    Unless required by applicable law or agreed to in writing, this
    software is distributed on an "AS IS" BASIS, WITHOUT Wfwd_errANTIES OR
    CONDITIONS OF ANY KIND, either express or implied.
    
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"

#include <driver/ledc.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#define LED_BUILTIN 2
#define HCSR_HIGH 1
#define HCSR_LOW 0


static const char *TAG = "hc-sr04_driver";

// PINS
//#define LED_BUILTIN 33
#define PIN_LEFT_FORWARD 26
#define PIN_LEFT_BACKWARD 27
#define PIN_RIGHT_FORWARD 14
#define PIN_RIGHT_BACKWARD 25

// PWM Channels (Reserve channel 0 and 1 for camera)
#define PWM_LEFT_FORWARD LEDC_CHANNEL_2
#define PWM_LEFT_BACKWARD LEDC_CHANNEL_3
#define PWM_RIGHT_FORWARD LEDC_CHANNEL_4
#define PWM_RIGHT_BACKWARD LEDC_CHANNEL_5

// Other PWM settings
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION LEDC_TIMER_12_BIT
#define PWM_TIMER LEDC_TIMER_1
#define PWM_MODE LEDC_HIGH_SPEED_MODE

/*// Function Prototypes
int setup_ledc(); //Setup all the pins and such to get the GPIO ready to be used
esp_err_t hcsr_send_trig_signal(); //Send a 10uS trig signal HIGH to the sensor
int hcsr_echo_pulse_read(); //Read the returned pulse and update a variable*/



// Function Def's

void cmd_vel_callback(const void *msgin);

int motor_pin_setup()
{
    ESP_LOGI(TAG,"Setting up ledc");

    ledc_channel_config_t ledc_channel[4] = {
        {
            .channel    = PWM_LEFT_FORWARD,
            .duty       = 0,
            .gpio_num   = PIN_LEFT_FORWARD,
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
        {
            .channel    = PWM_LEFT_BACKWARD,
            .duty       = 0,
            .gpio_num   = PIN_LEFT_BACKWARD,
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
        {
            .channel    = PWM_RIGHT_FORWARD,
            .duty       = 0,
            .gpio_num   = PIN_RIGHT_FORWARD,
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
        {
            .channel    = PWM_RIGHT_BACKWARD,
            .duty       = 0,
            .gpio_num   = PIN_RIGHT_BACKWARD,
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
    };

    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&ledc_channel[i]);
    }
    // Configure timer
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    

    ESP_LOGI(TAG,"ledc Setup Completed NOW!");

    return 0;
}











