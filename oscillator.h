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

#ifndef _DEMIURGE_OSCILLATOR_H_
#define _DEMIURGE_OSCILLATOR_H_

#define DEMIURGE_SINE 1
#define DEMIURGE_SQUARE 2
#define DEMIURGE_TRIANGLE 3
#define DEMIURGE_SAW 4

#include "signal.h"

typedef struct {
   uint64_t lastTrig;
   signal_t me;
   signal_t *frequency;
   signal_t *amplitude;
   signal_t *trigger;
   int mode;
   int idx;
   uint64_t t0;
   uint32_t period_in_us;
   float slope;
   float scale;
} oscillator_t;

static const int SINEWAVE_SAMPLES = 1800;

void oscillator_init( oscillator_t *handle, int mode);
void oscillator_configure(oscillator_t *handle, signal_t *freqControl, signal_t *amplitudeControl, signal_t *trigControl);
void oscillator_configure_frequency(oscillator_t *handle, signal_t *control);
void oscillator_configure_amplitude(oscillator_t *handle, signal_t *control);
void oscillator_configure_trig(oscillator_t *handle, signal_t *control);

float oscillator_read(signal_t *handle, uint64_t time_in_us);
#endif
