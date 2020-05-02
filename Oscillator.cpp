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

#include <freertos/FreeRTOS.h>
#include <math.h>
#include <esp_task.h>
#include <esp_log.h>
#include "Oscillator.h"
#include "OctavePerVolt.h"

static bool sine_wave_initialized = false;
static float sine_wave[SINEWAVE_SAMPLES];

Oscillator::Oscillator(int mode) {
   ESP_LOGI("Oscillator", "Constructor: %llx", (uint64_t) this);
   if (!sine_wave_initialized) {
      for (int i = 0; i < SINEWAVE_SAMPLES; i++) {
         double radians = ((double) i / SINEWAVE_SAMPLES) * M_TWOPI;
         sine_wave[i] = sin(radians);
      }
      sine_wave_initialized = true;
   }
   _signal.data = &_data;
   _signal.read_fn = oscillator_read;
   _data.mode = mode;
   _triggerControl = nullptr;
   _amplitudeControl = nullptr;
   _frequencyControl = nullptr;
   _data.lastTrig = 0;
   auto *osc = (oscillator_t *) _signal.data;
   osc->scale = 1.0;
   osc->t0 = 0;
   osc->period_in_us = 1000000 / 440;
}

Oscillator::~Oscillator() = default;

void Oscillator::configure(Signal *freqControl, Signal *amplitudeControl, Signal *trigControl) {
   configureFrequency(freqControl);
   configureAmplitude(amplitudeControl);
   configureTrig(trigControl);
}

void Oscillator::configureFrequency(Signal *freqControl) {
   configASSERT(freqControl != nullptr)
   ESP_LOGD("Oscillator", "Configure frequency control: %llx for %llx", (uint64_t) freqControl, (uint64_t) this);

   _frequencyControl = freqControl;
   _data.frequency = &freqControl->_signal;
   ESP_LOGD("Oscillator", "frequency controller: %llx", (uint64_t) &freqControl->_signal);
}

void Oscillator::configureAmplitude(Signal *amplitudeControl) {
   configASSERT(amplitudeControl != nullptr)
   ESP_LOGD("Oscillator", "Configure amplitude control: %llx for %llx", (uint64_t) amplitudeControl, (uint64_t) this);
   _amplitudeControl = amplitudeControl;
   _data.amplitude = &amplitudeControl->_signal;
   ESP_LOGD("Oscillator", "amplitude controller: %llx", (uint64_t) &amplitudeControl->_signal);
}

void Oscillator::configureTrig(Signal *trigControl) {
   configASSERT(trigControl != nullptr)
   _triggerControl = trigControl;
   _data.trigger = &trigControl->_signal;
}

static uint32_t period(const oscillator_t *osc, signal_t *handle, uint64_t time_in_us) {
   signal_t *freqControl = osc->frequency;
   float freq = 440;
   if (freqControl != nullptr) {
      float voltage = freqControl->read_fn(freqControl, time_in_us);
      freq = octave_frequencyOf(voltage);
      handle->extra2 = voltage;
      handle->extra3 = freq;
   }
   uint32_t period_in_us = 1000000 / freq;
   return period_in_us;
}

static float scale(oscillator_t *osc, uint64_t time_in_us) {
   float scale = 1.0f;
   if (osc->amplitude != nullptr) {
      scale = (10.0f + osc->amplitude->read_fn(osc->amplitude, time_in_us))/4.0;
   }
   return scale / 2.0;
}

float IRAM_ATTR oscillator_read(signal_t *handle, uint64_t time_in_us) {
   auto *osc = (oscillator_t *) handle->data;
   // time in microseconds
   if (time_in_us > handle->last_calc) {
      handle->last_calc = time_in_us;

      uint64_t x = time_in_us - osc->t0;
      if (x >= osc->period_in_us) {
         x = 0;
         osc->t0 = time_in_us;
         osc->period_in_us = period(osc, handle, time_in_us);
         osc->slope = 1.0 / osc->period_in_us;
      }
      float out = 0.0;
      switch (osc->mode) {
         case DEMIURGE_SINE: {
            float percentage_of_cycle = ((float) x) / osc->period_in_us;
            uint16_t index = SINEWAVE_SAMPLES * percentage_of_cycle;
            float gain = scale(osc, time_in_us);
            out = sine_wave[index] * gain;
            handle->extra4 = gain;
            break;
         }
         case DEMIURGE_SQUARE: {
            if (x > osc->period_in_us / 2)
               out = scale(osc, time_in_us);
            else
               out = -scale(osc, time_in_us);
            break;
         }
         case DEMIURGE_TRIANGLE: {
            // TODO
            break;
         }
         case DEMIURGE_SAW: {
            out = (float) ((osc->slope * x)) * osc->scale;
            break;
         }
      }
      handle->extra1 = out;
      handle->cached = out;
      return out;
   }
   return handle->cached;
}

