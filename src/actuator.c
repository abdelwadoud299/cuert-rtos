/**
 * @file actuator.c
 * @brief Hardware PWM actuator driver using ESP32 LEDC peripheral.
 */

#include "actuator.h"
#include "config.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          ACTUATOR_LED_GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY          ACTUATOR_PWM_FREQ_HZ

void Actuator_Init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void Actuator_SetOutput(uint8_t percent)
{
    if (percent > 100) percent = 100;

    uint32_t duty = ((uint32_t)percent * ACTUATOR_PWM_MAX_DUTY) / 100UL;

#if (ACTUATOR_LED_ACTIVE_LEVEL == 0)
    duty = ACTUATOR_PWM_MAX_DUTY - duty;
#endif

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void Actuator_SetDigitalLevel(bool is_on)
{
    uint32_t duty = is_on ? ACTUATOR_PWM_MAX_DUTY : 0;

#if (ACTUATOR_LED_ACTIVE_LEVEL == 0)
    duty = ACTUATOR_PWM_MAX_DUTY - duty;
#endif

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}
