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

#include "demi_asserts.h"
#include "oscillator.h"
#include "octave_per_volt.h"

static bool sine_wave_initialized = false;
static float *sine_wave;

void oscillator_init(oscillator_t *handle, int mode) {
   if (!sine_wave_initialized) {
      sine_wave = (float *) calloc(SINEWAVE_SAMPLES, sizeof(float));
      for (int i = 0; i < SINEWAVE_SAMPLES; i++) {
         double radians = ((double) i / SINEWAVE_SAMPLES) * M_TWOPI;
         sine_wave[i] = sin(radians);
      }
      sine_wave_initialized = true;
   }
   handle->me.read_fn = oscillator_read;
   handle->me.data = handle;
   handle->mode = mode;
   handle->frequency = NULL;
   handle->amplitude = NULL;
   handle->trigger = NULL;
   handle->lastTrig = 0;
   handle->scale = 1.0;
   handle->t0 = 0;
   handle->period_in_us = 1000000 / 440;
}
void oscillator_configure(oscillator_t *handle, signal_t *freqControl, signal_t *amplitudeControl, signal_t *trigControl) {
   oscillator_configure_frequency(handle, freqControl);
   oscillator_configure_amplitude(handle, amplitudeControl);
   oscillator_configure_trig(handle, trigControl);
}

void oscillator_configure_frequency(oscillator_t *handle, signal_t *control) {
   configASSERT(control != NULL)
   handle->frequency = control;
}

void oscillator_configure_amplitude(oscillator_t *handle, signal_t *control) {
   configASSERT(control != NULL)
   handle->amplitude = control;
}

void oscillator_configure_trig(oscillator_t *handle, signal_t *control) {
   configASSERT(control != NULL)
   handle->trigger = control;
}

static uint32_t IRAM_ATTR period(const oscillator_t *osc, signal_t *handle, uint64_t time_in_us) {
   signal_t *freqControl = osc->frequency;
   float freq = 440;
   if (freqControl != NULL) {
      float voltage = freqControl->read_fn(freqControl, time_in_us);
      freq = octave_frequencyOf(voltage);
      handle->extra2 = voltage;
      handle->extra3 = freq;
   }
   uint32_t period_in_us = 1000000 / freq;
   return period_in_us;
}

static float IRAM_ATTR scale(oscillator_t *osc, uint64_t time_in_us) {
   float scale = 1.0f;
   if (osc->amplitude != NULL) {
      scale = (10.0f + osc->amplitude->read_fn(osc->amplitude, time_in_us))/4.0;
   }
   return scale / 2.0;
}

float IRAM_ATTR oscillator_read(signal_t *handle, uint64_t time_in_us) {
   // time in microseconds
   if (time_in_us > handle->last_calc) {
      handle->last_calc = time_in_us;
      oscillator_t *osc = (oscillator_t *) handle->data;

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
      out = handle->post_fn(out);
      handle->extra1 = out;
      handle->cached = out;
      return out;
   }
   return handle->cached;
}

