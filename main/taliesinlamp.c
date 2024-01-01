/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/ledc.h"
#include <math.h>
#include "esp_adc/adc_oneshot.h"


#define LEDC_TIMER_L1              LEDC_TIMER_0
#define LEDC_MODE_L1               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO_L1          (1) // Define the output GPIO
#define LEDC_CHANNEL_L1            LEDC_CHANNEL_0
#define LEDC_TIMER_L2              LEDC_TIMER_1
#define LEDC_MODE_L2               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO_L2          (2) // Define the output GPIO
#define LEDC_CHANNEL_L2            LEDC_CHANNEL_1
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY_MAX               (8191) // Set duty to 50%. (2 ** 13) * 50% = 4096
// #define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz


static const char *TAG = "taliesin-lamp";

static void lamp1_ledc_init(void)
{

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE_L1,
        .channel        = LEDC_CHANNEL_L1,
        .timer_sel      = LEDC_TIMER_L1,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_L1,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE_L1,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER_L1,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

}

static void lamp2_ledc_init(void)
{

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE_L2,
        .channel        = LEDC_CHANNEL_L2,
        .timer_sel      = LEDC_TIMER_L2,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_L2,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE_L2,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER_L2,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

}

static void set_lamp1_pwm(size_t pwm1)
{
//    uint16_t new_duty1 = pow((((double) pwm1)/4095),4)*8191;
    uint16_t new_duty1 = 8191*(tan((1.54*pwm1)/4095)/tan(1.54));
//    ESP_LOGI(TAG,"in actual code, input pwm1 to %i out new is %i",pwm1,new_duty1);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE_L1,LEDC_CHANNEL_L1,new_duty1));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE_L1,LEDC_CHANNEL_L1));
}

static void set_lamp2_pwm(size_t pwm2)
{
//    ESP_LOGI(TAG,"in actual code, set pwm2 to %i",pwm2);
    uint16_t new_duty2 = 8191*(tan((1.54*pwm2)/4095)/tan(1.54));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE_L2,LEDC_CHANNEL_L2,new_duty2));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE_L2,LEDC_CHANNEL_L2));
}



void app_main(void)
{

    // Initialize PWM for lamp1 (warm) and lamp2 (cool). Turn both off
    lamp1_ledc_init();
    lamp2_ledc_init();
    set_lamp1_pwm(0);
    set_lamp2_pwm(0);

    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config));

    static int brightness_raw;
    static int temperature_raw;

    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &brightness_raw));
//        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_3, brightness_raw);
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &temperature_raw));
//        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_4, temperature_raw);

        ESP_LOGI(TAG,"temp raw %d",temperature_raw);

//        double brightness_factor = brightness_raw / 4095;
        double warm_factor = ((double)temperature_raw/4095);
//            double warm_factor = 2300/4095;
//        warm_factor = .5;

        ESP_LOGI(TAG,"warm %f",warm_factor);

        size_t bulb_base_duty = 8191*(tan((1.54*brightness_raw)/4095)/tan(1.54));
        size_t warm_bulb_base = bulb_base_duty * warm_factor;
        size_t cool_bulb_base = bulb_base_duty * (1 - warm_factor);

        set_lamp1_pwm(warm_bulb_base);
        set_lamp2_pwm(cool_bulb_base);

        ESP_LOGI(TAG, "BrightRaw: %d TempRaw: %d WarmFactor: %6f%% Warm: %d Cool: %d", brightness_raw,temperature_raw,warm_factor,warm_bulb_base,cool_bulb_base);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

