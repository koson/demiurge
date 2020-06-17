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
#include <soc/timer_group_reg.h>
#include <driver/ledc.h>
#include <mcp4822/MCP4822.h>
#include <soc/timer_periph.h>
#include <esp32/dport_access.h>
#include <adc128s102/ADC128S102.h>
#include <soc/dport_reg.h>
#include <esp32/cache_err_int.h>
#include <esp32/rom/cache.h>
#include "demiurge.h"

#define TAG "SOUND"

static gpio_num_t gpio_output[] = {GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_33, GPIO_NUM_27};
static uint32_t gpio_output_level[] = {1, 1, 1, 1, 0, 0};
static gpio_num_t gpio_input[] = {GPIO_NUM_32, GPIO_NUM_36, GPIO_NUM_37, GPIO_NUM_38, GPIO_NUM_39};
static uint64_t nanos_per_tick;

static uint32_t _ticks_per_second;
static uint64_t _gpios;
static bool _started;
static uint64_t timer_counter = 0;
static volatile uint64_t tick_start = 0;
static volatile uint64_t tick_duration = 0;
static volatile uint64_t tick_interval = 0;
static bool app_cpu_started = false;

static volatile signal_t *_sinks[DEMIURGE_MAX_SINKS];
float _inputs[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float _outputs[2] = {0.0f, 0.0f};
static mcp4822_t _dac;
static adc128s102_t _adc;

// Overrun increments means that the tick() took longer than the sample time.
// If you see this happening, either decrease the sample rate or optimize the tick()
// evaluation to take less time.
static uint32_t overrun = -3;  // 3 overruns happen during startup, and that is ok. Let's compensate for that.

extern int _init_start;

static void initialize_time() {
   // Initialize timer used for timing
   TIMERG0.hw_timer[1].config.divider = 80;         // 80MHz divided by 80, gives us a microsecond clock
   TIMERG0.hw_timer[1].config.increase = 1;
   TIMERG0.hw_timer[1].load_high = 0;
   TIMERG0.hw_timer[1].load_low = 0;
   TIMERG0.hw_timer[1].config.enable = 1;
}

uint64_t IRAM_ATTR demiurge_current_time() {
   TIMERG0.hw_timer[1].update = 1;
   uint64_t lo = TIMERG0.hw_timer[1].cnt_low;
   uint64_t hi = TIMERG0.hw_timer[1].cnt_high;
   return lo + (hi << 32) / 40;
}

static void IRAM_ATTR initialize_tick_timer(int ticks_per_second) {
   TIMERG0.clk.en = 1;
   nanos_per_tick = 1000000000 / ticks_per_second;
   TIMERG0.hw_timer[0].config.divider = 2;         // 80MHz divided by 2, gives us a 1/40MHz tick
   TIMERG0.hw_timer[0].config.increase = 1;
   TIMERG0.hw_timer[0].config.alarm_en = 1;
   TIMERG0.hw_timer[0].config.autoreload = 1;

   TIMERG0.hw_timer[0].load_high = 0;
   TIMERG0.hw_timer[0].load_low = 0;
   TIMERG0.hw_timer[0].alarm_high = 0;
   TIMERG0.hw_timer[0].alarm_low = (nanos_per_tick * 40) / 1000 - 1; // 40 ticks per microsecond
   TIMERG0.hw_timer[0].reload = 1;
   TIMERG0.hw_timer[0].config.enable = 1;
   ESP_EARLY_LOGI(TAG, "Microseconds/tick: %d", TIMERG0.hw_timer[0].alarm_low);
}


static void IRAM_ATTR wait_timer_alarm() {
   int counter = 0;
   // According to 18.2.3 in Technical Reference; "The timer alarm enable bit is automatically cleared once an alarm occurs"
   // So we need to wait for it to clear, and then set it again.
   while (TIMERG0.hw_timer[0].config.alarm_en) {
      counter++;
   }
   if (counter < 2)
      overrun++;
   TIMERG0.hw_timer[0].config.alarm_en = 1;
}

void demiurge_registerSink(signal_t *processor) {
   ESP_EARLY_LOGI(TAG, "Registering Sink: %p", (void *) processor);
   configASSERT(processor != NULL)
   for (int i = 0; i < DEMIURGE_MAX_SINKS; i++) {
      if (_sinks[i] == NULL) {
         _sinks[i] = processor;
         ESP_EARLY_LOGI(TAG, "Registering Sink: %d", i);
         break;
      }
   }
}

void demiurge_unregisterSink(signal_t *processor) {
   ESP_EARLY_LOGI(TAG, "Unregistering Sink: %p", (void *) processor);
   configASSERT(processor != NULL)
   for (int i = 0; i < DEMIURGE_MAX_SINKS; i++) {
      if (_sinks[i] == processor) {
         _sinks[i] = NULL;
         ESP_EARLY_LOGI(TAG, "Unregistering Sink: %d, %p", i, (void *) processor);
         break;
      }
   }
}

#ifdef DEMIURGE_TICK_TIMING
static uint32_t tick_update = 0;
#endif

bool IRAM_ATTR demiurge_gpio(int pin) {
   configASSERT(pin > 0 && pin <= 39)
   return (_gpios >> pin & 1) != 0;
}

void IRAM_ATTR demiurge_set_output(int number, float value) {
   configASSERT(number > 0 && number <= 2)
   _outputs[number - 1] = value;
}

void IRAM_ATTR demiurge_print_overview(const char *tag, signal_t *signal) {
#ifdef DEMIURGE_TICK_TIMING
   ESP_LOGI("TICK", "interval=%lld, duration=%lld, start=%lld, overrun=%d",
            tick_interval, tick_duration, tick_start, overrun);
#endif  //DEMIURGE_TICK_TIMING
   ESP_LOGI(tag, "Input: %2.1f, %2.1f, %2.1f, %2.1f, %2.1f, %2.1f, %2.1f, %2.1f",
            demiurge_input(1),
            demiurge_input(2),
            demiurge_input(3),
            demiurge_input(4),
            demiurge_input(5),
            demiurge_input(6),
            demiurge_input(7),
            demiurge_input(8)
   );
   ESP_LOGI(tag, "Output: %2.1f, %2.1f",
            demiurge_output(1),
            demiurge_output(2)
   );
#ifdef DEMIURGE_DEV
   ESP_LOGI(tag, "Extras: [%lld] : %2.1f - %2.1f, %2.1f, %2.1f, %2.1f, %2.1f, %2.1f, %2.1f, %2.1f",
            signal->last_calc,
            signal->cached,
            signal->extra1,
            signal->extra2,
            signal->extra3,
            signal->extra4,
            signal->extra5,
            signal->extra6,
            signal->extra7,
            signal->extra8
   );
#endif // DEMIURGE_DEV
}

float IRAM_ATTR demiurge_input(int number) {
   configASSERT(number > 0 && number <= 8)
   return _inputs[number - 1];
}

float IRAM_ATTR demiurge_output(int number) {
   configASSERT(number > 0 && number <= DEMIURGE_MAX_SINKS)
   return _outputs[number - 1];
}

static void IRAM_ATTR readGpio() {
   _gpios = gpio_input_get() | (((uint64_t) gpio_input_get_high()) << 32);
}

void IRAM_ATTR demiurge_tick() {
   gpio_set_level(GPIO_NUM_27, 1); // TP1 - Test Point to check timing
   timer_counter = demiurge_current_time();
#ifdef DEMIURGE_TICK_TIMING
   tick_interval = timer_counter - tick_start;
   tick_start = timer_counter;
#endif
   // We are setting the outputs at the start of a cycle, to ensure that the interval is identical from cycle to cycle.
   mcp4822_set(&_dac, (uint16_t) ((10.0f - _outputs[0]) * 204.8f), (uint16_t) ((10.0f - _outputs[1]) * 204.8f));

   readGpio();
   adc128s102_read(&_adc, _inputs);
   for (int i = 0; i < DEMIURGE_MAX_SINKS; i++) {
      signal_t *sink = _sinks[i];
      if (sink != NULL) {
         sink->read_fn(sink, timer_counter);  // ignore return value
      }
   }
#ifdef DEMIURGE_TICK_TIMING
   if (tick_update > 200000) {
      tick_duration = demiurge_current_time() - tick_start;
      tick_update = 0;
   }
   tick_update++;
#endif
   gpio_set_level(GPIO_NUM_27, 0); // TP1 - Test Point to check timing
}

static void demiurge_initialize() {
   ESP_EARLY_LOGI(TAG, "Starting Demiurge...\n");
   for (int i = 0; i < DEMIURGE_MAX_SINKS; i++)
      _sinks[i] = NULL;
   _started = false;
   _gpios = 0;
   for (int i = 0; i < DEMIURGE_MAX_SINKS; i++) {
      _sinks[i] = NULL;
   }
   initialize_time();
   initialize_tick_timer(_ticks_per_second);
   for (int i = 0; i < sizeof(gpio_output) / sizeof(gpio_num_t); i++) {
      ESP_EARLY_LOGI(TAG, "Init GPIO%u", gpio_output[i]);
      esp_err_t error = gpio_set_direction(gpio_output[i], GPIO_MODE_OUTPUT);
      configASSERT(error == ESP_OK)
      gpio_set_level(gpio_output[i], gpio_output_level[i]);
   }
   for (int i = 0; i < sizeof(gpio_input) / sizeof(gpio_num_t); i++) {
      ESP_EARLY_LOGI(TAG, "Init GPIO%u", gpio_input[i]);
      esp_err_t error = gpio_set_direction(gpio_input[i], GPIO_MODE_INPUT);
      configASSERT(error == ESP_OK)
   }
   ESP_EARLY_LOGI(TAG, "Initialized GPIO done");

   _dac.mosi_pin = GPIO_NUM_13;
   _dac.sclk_pin = GPIO_NUM_14;
   _dac.cs_pin = GPIO_NUM_15;
   _dac.ldac_pin = GPIO_NUM_4;
   _dac.spiHw = &SPI2;

   _adc.spiHw = &SPI3;
   _adc.mosi_pin = GPIO_NUM_23;
   _adc.miso_pin = GPIO_NUM_19;
   _adc.sclk_pin = GPIO_NUM_18;
   _adc.cs_pin = GPIO_NUM_5;
   mcp4822_initialize(&_dac);
   adc128s102_initialize(&_adc);
}

void demiurge_core1_task() {
   if (_started)
      return;
   _started = true;
   while (true) {
      wait_timer_alarm();
      demiurge_tick();
   }
}

void start_cpu1(void) {
   esp_cache_err_int_init();
   ESP_EARLY_LOGI(TAG, "Starting Demiurge on APP CPU: %d", xPortGetCoreID());
   app_cpu_started = true;
   demiurge_core1_task();
   abort(); /* Only get to here if Demiurge platform is somehow very broken */
}

static void wdt_reset_cpu1_info_enable(void) {
   DPORT_REG_SET_BIT(DPORT_APP_CPU_RECORD_CTRL_REG, DPORT_APP_CPU_PDEBUG_ENABLE | DPORT_APP_CPU_RECORD_ENABLE);
   DPORT_REG_CLR_BIT(DPORT_APP_CPU_RECORD_CTRL_REG, DPORT_APP_CPU_RECORD_ENABLE);
}

void IRAM_ATTR call_start_cpu1(void) {
   asm volatile (\
                  "wsr    %0, vecbase\n" \
                  ::"r"(&_init_start));

   ets_set_appcpu_boot_addr(0);
   cpu_configure_region_protection();
   cpu_init_memctl();

   wdt_reset_cpu1_info_enable();
   ESP_EARLY_LOGI(TAG, "APP CPU up: %d", xPortGetCoreID());
   start_cpu1();
}

void demiurge_start(uint64_t sample_rate) {
   _ticks_per_second = sample_rate;
   demiurge_initialize();
   //Flush and enable icache for APP CPU
   Cache_Flush(1);
   Cache_Read_Enable(1);
   esp_cpu_unstall(1);
   if (!DPORT_GET_PERI_REG_MASK(DPORT_APPCPU_CTRL_B_REG, DPORT_APPCPU_CLKGATE_EN)) {
      DPORT_SET_PERI_REG_MASK(DPORT_APPCPU_CTRL_B_REG, DPORT_APPCPU_CLKGATE_EN);
      DPORT_CLEAR_PERI_REG_MASK(DPORT_APPCPU_CTRL_C_REG, DPORT_APPCPU_RUNSTALL);
      DPORT_SET_PERI_REG_MASK(DPORT_APPCPU_CTRL_A_REG, DPORT_APPCPU_RESETTING);
      DPORT_CLEAR_PERI_REG_MASK(DPORT_APPCPU_CTRL_A_REG, DPORT_APPCPU_RESETTING);
   }
   ets_set_appcpu_boot_addr((uint32_t) call_start_cpu1);
   while (!app_cpu_started) {
      ets_delay_us(100);
   }
   ESP_LOGI(TAG, "Demiurge Started.");
}

#undef TAG
