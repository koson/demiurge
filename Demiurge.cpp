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

#include <esp_log.h>
#include <esp_task_wdt.h>
#include <soc/timer_group_reg.h>
#include <driver/ledc.h>
#include <driver/i2c.h>
#include <mcp4822/MCP4822.h>

#include "Demiurge.h"

#define TAG "DEMI"
#define DEMIURGE_TIMER_GROUP 0

static uint16_t micros_per_tick;

static void IRAM_ATTR initialize_tick_timer(int ticks_per_second) {
   micros_per_tick = 1000000 / ticks_per_second;
   WRITE_PERI_REG(TIMG_T0CONFIG_REG(DEMIURGE_TIMER_GROUP),
                  (1 << TIMG_T0_DIVIDER_S) | TIMG_T0_EN | TIMG_T0_AUTORELOAD | TIMG_T0_INCREASE | TIMG_T0_ALARM_EN);

   WRITE_PERI_REG(TIMG_T0LOADHI_REG(DEMIURGE_TIMER_GROUP), 0);
   WRITE_PERI_REG(TIMG_T0LOADLO_REG(DEMIURGE_TIMER_GROUP), 0);
   WRITE_PERI_REG(TIMG_T0ALARMHI_REG(DEMIURGE_TIMER_GROUP), 0);
   WRITE_PERI_REG(TIMG_T0ALARMLO_REG(DEMIURGE_TIMER_GROUP), 40 * micros_per_tick);

   WRITE_PERI_REG(TIMG_T0LOAD_REG(DEMIURGE_TIMER_GROUP), 1);
}

static const uint32_t timerRetrig = (1 << TIMG_T0_DIVIDER_S) | TIMG_T0_EN | TIMG_T0_AUTORELOAD | TIMG_T0_INCREASE;
static uint32_t overrun = 0;

static void IRAM_ATTR wait_timer_alarm() {
   int counter = 0;
   while (READ_PERI_REG(TIMG_T0CONFIG_REG(DEMIURGE_TIMER_GROUP)) & TIMG_T0_ALARM_EN)
      counter++;
   if (counter > 0)
      overrun++;
   WRITE_PERI_REG(TIMG_T0CONFIG_REG(DEMIURGE_TIMER_GROUP), timerRetrig);
   WRITE_PERI_REG(TIMG_T0LOAD_REG(DEMIURGE_TIMER_GROUP), 1);
}

void IRAM_ATTR startInfiniteTask(void *parameter) {
   ESP_LOGD(TAG, "Starting audio algorithm in Core %d", xTaskGetAffinity(nullptr));
   auto *demiurge = static_cast<Demiurge *>(parameter);
   demiurge->initialize();
   initialize_tick_timer(demiurge->_ticks_per_second);
   while (true) {
      wait_timer_alarm();
      demiurge->tick();
   }

}

Demiurge::Demiurge() {
   ESP_LOGI(TAG, "Starting Demiurge...\n");
   for (int i = 0; i < DEMIURGE_MAX_SINKS; i++)
      _sinks[i] = nullptr;
   _started = false;
   _gpios = 0;
   esp_err_t err = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
   ETS_ASSERT(err == ESP_OK);
   initializeSinks();
};

static gpio_num_t gpio_output[] = {GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_33, GPIO_NUM_27};
static uint32_t gpio_output_level[] = {1, 1, 1, 1, 0, 0};
static gpio_num_t gpio_input[] = {GPIO_NUM_32, GPIO_NUM_36, GPIO_NUM_37, GPIO_NUM_38, GPIO_NUM_39};

void Demiurge::initialize() {
   for (int i = 0; i < sizeof(gpio_output) / sizeof(gpio_num_t); i++) {
      ESP_LOGI(TAG, "Init GPIO%u", gpio_output[i]);
      esp_err_t error = gpio_set_direction(gpio_output[i], GPIO_MODE_OUTPUT);
      configASSERT(error == ESP_OK)
      gpio_set_level(gpio_output[i], gpio_output_level[i]);
   }

   for (int i = 0; i < sizeof(gpio_input) / sizeof(gpio_num_t); i++) {
      ESP_LOGI(TAG, "Init GPIO%u", gpio_input[i]);
      esp_err_t error = gpio_set_direction(gpio_input[i], GPIO_MODE_INPUT);
      configASSERT(error == ESP_OK)
   }
   ESP_LOGI(TAG, "Initialized GPIO done");

   _dac = new MCP4822(GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_15);
   _adc = new ADC128S102(GPIO_NUM_23, GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5);

   // Initialize timer used for stats in debugging
   WRITE_PERI_REG(TIMG_T1CONFIG_REG(DEMIURGE_TIMER_GROUP),
                  (1 << TIMG_T1_DIVIDER_S) | TIMG_T1_EN | TIMG_T1_AUTORELOAD | TIMG_T1_INCREASE);

}

void Demiurge::startRuntime(void (*user_code)(void *demiurge), int ticks_per_second) {
   if (_started)
      return;
   overrun_ptr = &overrun;
   TaskHandle_t idle_task = xTaskGetIdleTaskHandleForCPU(0);
   if (idle_task == NULL || esp_task_wdt_delete(idle_task) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to remove Core 0 IDLE task from WDT");
   }
   _started = true;
   _ticks_per_second = ticks_per_second;
   TaskHandle_t idleTask = xTaskGetIdleTaskHandle();
   esp_task_wdt_delete(idleTask);
   ESP_LOGI(TAG, "Executing in Core %d", xTaskGetAffinity(nullptr));
   xTaskCreatePinnedToCore(user_code, "App", 8192, this, 7, &_taskHandle, 0);
   xTaskCreatePinnedToCore(startInfiniteTask, "Audio", 8192, this, 7, &_taskHandle, 1);
   ESP_LOGE(TAG, "INTERNAL ERROR!!! Should never reach here.");
}

void Demiurge::initializeSinks() {
   ESP_LOGI(TAG, "Clearing all sinks!");
   for (auto &_sink : _sinks) {
      _sink = nullptr;
   }
}

Demiurge::~Demiurge() {
   delete _dac;
   delete _adc;
}

void Demiurge::registerSink(signal_t *processor) {
   ESP_LOGI(TAG, "Registering Sink: %llx", (uint64_t) processor);
   configASSERT(processor != nullptr)
   for (int i = 0; i < DEMIURGE_MAX_SINKS; i++) {
      if (_sinks[i] == nullptr) {
         _sinks[i] = processor;
         ESP_LOGI(TAG, "Registering Sink: %d", i);
         break;
      }
   }
}

void Demiurge::unregisterSink(signal_t *processor) {
   ESP_LOGI(TAG, "Unregistering Sink: %llx", (uint64_t) processor);
   configASSERT(processor != nullptr)
   for (int i = 0; i < DEMIURGE_MAX_SINKS; i++) {
      if (_sinks[i] == processor) {
         _sinks[i] = nullptr;
         ESP_LOGI(TAG, "Unregistering Sink: %d, %llx", i, (uint64_t) processor);
         break;
      }
   }
}

static uint32_t tick_update = 0;

void IRAM_ATTR Demiurge::tick() {
   gpio_set_level(GPIO_NUM_27, 1); // TP1 - Test Point to check timing
   if (tick_update > 76543) {
      WRITE_PERI_REG(TIMG_T1UPDATE_REG(DEMIURGE_TIMER_GROUP), tick_update);
      uint32_t lo = READ_PERI_REG(TIMG_T1LO_REG(DEMIURGE_TIMER_GROUP));
      uint32_t hi = READ_PERI_REG(TIMG_T1HI_REG(DEMIURGE_TIMER_GROUP));
      uint64_t now = lo + ((uint64_t) hi << 32);
      tick_interval = now - tick_start;
      tick_start = now;
   }
   // We are setting the outputs at the start of a cycle, to ensure that the interval is identical from cycle to cycle.
   _dac->setOutput((uint16_t) ((10.0f - _outputs[0]) * 204.8f), (uint16_t) ((10.0f - _outputs[1]) * 204.8f));

   readGpio();
   _adc->read_inputs(_inputs);
   timerCounter = timerCounter + 10; // pass along number of microseconds.

   for (auto sink : _sinks) {
      if (sink != nullptr) {
         sink->read_fn(sink, timerCounter);  // ignore return value
      }
   }
   if (tick_update > 76543) {
      WRITE_PERI_REG(TIMG_T1UPDATE_REG(DEMIURGE_TIMER_GROUP), 1);
      uint32_t lo = READ_PERI_REG(TIMG_T1LO_REG(DEMIURGE_TIMER_GROUP));
      uint32_t hi = READ_PERI_REG(TIMG_T1HI_REG(DEMIURGE_TIMER_GROUP));
      uint64_t now = lo + ((uint64_t) hi << 32);
      tick_duration = now - tick_start;
      tick_update = 0;
   }
   tick_update++;
   gpio_set_level(GPIO_NUM_27, 0); // TP1 - Test Point to check timing
}

void IRAM_ATTR Demiurge::readGpio() {
   _gpios = gpio_input_get() | (((uint64_t) gpio_input_get_high()) << 32);
}

float IRAM_ATTR Demiurge::input(int number) {
   configASSERT(number > 0 && number <= 8)
   return _inputs[number - 1];
}

float IRAM_ATTR Demiurge::output(int number) {
   configASSERT(number > 0 && number <= DEMIURGE_MAX_SINKS)
   return _outputs[number - 1];
}

bool IRAM_ATTR Demiurge::gpio(int pin) {
   configASSERT(pin > 0 && pin <= 39)
   return (_gpios >> pin & 1) != 0;
}

void IRAM_ATTR Demiurge::set_output(int number, float value) {
   configASSERT(number > 0 && number <= 2)
   _outputs[number - 1] = value;
}

void Demiurge::print_overview(const char *tag, Signal *signal) {
   ESP_LOGI("TICK", "interval=%d, duration=%d, start=%lld, overruns=%d", tick_interval, tick_duration, tick_start,
            *overrun_ptr);
   ESP_LOGI("INPUT", "Input: %2.1f, %2.1f, %2.1f, %2.1f, %2.1f, %2.1f, %2.1f, %2.1f",
            input(1),
            input(2),
            input(3),
            input(4),
            input(5),
            input(6),
            input(7),
            input(8)
   );
   ESP_LOGI("OUTPUT", "Output: %2.1f, %2.1f",
            output(1),
            output(2)
   );
   ESP_LOGI(tag, "Extras: %2.1f, %2.1f, %2.1f, %2.1f, %2.1f, %2.1f, %2.1f, %2.1f",
            signal->extra1(),
            signal->extra2(),
            signal->extra3(),
            signal->extra4(),
            signal->extra5(),
            signal->extra6(),
            signal->extra7(),
            signal->extra8()
   );

}

#undef TAG
