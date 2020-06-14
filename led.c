/*
  Copyright 2019, Awesome Audio Apparatus.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

      https://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
      limitations under the License.
*/

#include <driver/ledc.h>
#include <esp_task.h>
#include <esp_log.h>
#include "led.h"
#include "demiurge.h"


const int LED_GPIO[] = {GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_25, GPIO_NUM_26};
const ledc_timer_t LED_TIMER[] = {LEDC_TIMER_0, LEDC_TIMER_1, LEDC_TIMER_2, LEDC_TIMER_3};
const ledc_channel_t LED_CHANNEL[] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};

// TODO: This code should be refactored so that all the hardware specifics goes to Demiurge::runtime(), and we can later devise a "driver" system if there are more boards being made.

void led_init(led_t *handle, int position) {
   configASSERT(position > 0 && position <= 4)
   handle->me.read_fn = led_read;
   handle->me.data = handle;
   handle->channel = LED_CHANNEL[position - 1];
   handle->input = NULL;
   handle->fade = NULL;
   handle->dutycycle = NULL;

   // TODO: make the fade and dutycycle dynamic

   ledc_timer_config_t timer_config;
   timer_config.timer_num = LED_TIMER[position - 1];
   timer_config.duty_resolution = LEDC_TIMER_13_BIT;
   timer_config.freq_hz = 5000;
   timer_config.speed_mode = LEDC_HIGH_SPEED_MODE;
   esp_err_t err = ledc_timer_config(&timer_config);
   ESP_ERROR_CHECK(err);

   ledc_channel_config_t conf;
   conf.gpio_num = LED_GPIO[position - 1];;
   conf.timer_sel = LED_TIMER[position - 1];
   conf.speed_mode = LEDC_HIGH_SPEED_MODE;
   conf.channel = LED_CHANNEL[position - 1];
   conf.hpoint = 4095;
   conf.intr_type = LEDC_INTR_DISABLE;
   conf.duty = 0;
   ESP_LOGD("LED", "Channel: %d", conf.channel);
   ESP_LOGD("LED", "Timer: %d", conf.timer_sel);
   ESP_LOGD("LED", "GPIO: %d", conf.gpio_num);
   err = ledc_channel_config(&conf);
   ESP_ERROR_CHECK(err);

   ledc_fade_func_install(0);
}

void led_configure_input(led_t *handle, signal_t *input) {
   if (!handle->registered) {
      demiurge_registerSink(&handle->me);
      handle->registered = true;
   }
   handle->input = input;
}

void led_configure_fade(led_t *handle, signal_t *fade) {
   handle->fade = fade;
}

void led_configure_dutycycle(led_t *handle, signal_t *dutycycle) {
   handle->dutycycle = dutycycle;
}

static void IRAM_ATTR led_set(ledc_channel_t channel, uint32_t duty, int fade_time_ms) {
   ledc_set_fade_with_time(LEDC_HIGH_SPEED_MODE, channel, duty, fade_time_ms);
   ledc_fade_start(LEDC_HIGH_SPEED_MODE, channel, LEDC_FADE_NO_WAIT);
};

float IRAM_ATTR led_read(signal_t *handle, uint64_t time) {
   led_t *led = (led_t *) handle->data;
   if (time > handle->last_calc) {
      handle->last_calc = time;
      signal_t *upstream = led->input;
      float result = upstream->read_fn(upstream, time);

// TODO: Fix the dynamic LEDs
//      uint32_t duty = (uint32_t) (result * 819.0f);
//      led_set(led->channel, duty, led->fade_time_ms);
//      led->duty = duty;
//      handle->cached = duty;
//      return duty;
   }
   return handle->cached;
}
