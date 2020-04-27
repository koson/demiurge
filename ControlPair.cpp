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
#include <esp_system.h>
#include <esp_log.h>
#include "ControlPair.h"

ControlPair::ControlPair(int position) {
<<<<<<< HEAD:ControlPair.cpp
   ESP_LOGD("ControlPair", "Constructor: %llx at position %d", (uint64_t) this, position);
=======
   ESP_LOGD("ControlPair", "Constructor: %llx at position %d", (uint64_t) this, position );
>>>>>>> b595f2ac077a1f9e560f73cacd57bba6de9898cc:src/ControlPair.cpp
   configASSERT(position > 0 && position <= 4)
   _potentiometer = new Potentiometer(position);
   _cvIn = new CvInPort(position);
   _signal.read_fn = controlpair_read;
   _signal.data = &_data;
   _data.potentiometer = &_potentiometer->_signal;
   _data.cv = &_cvIn->_signal;
   _data.potentiometer_scale = 1.0;
   _data.cv_scale = 1.0;
}

ControlPair::~ControlPair() {
   delete _potentiometer;
   delete _cvIn;
}

void ControlPair::set_potentiometer_scale(float scale) {
   _data.potentiometer_scale = scale;
}

void ControlPair::set_cv_scale(float scale) {
   _data.cv_scale = scale;
}

float IRAM_ATTR controlpair_read(signal_t *handle, uint64_t time) {
   auto *control = (control_pair_t *) handle->data;
   if (time > handle->last_calc) {
      handle->last_calc = time;

      signal_t *pot = control->potentiometer;
      float pot_in;
      float pot_scale = control->potentiometer_scale;
      if (pot_scale == 1.0) {
         pot_in = pot->read_fn(pot, time);
      } else {
         pot_in = pot->read_fn(pot, time) * pot_scale;
      }

      signal_t *cv = control->cv;
      float cv_in;
      float cv_scale = control->cv_scale;
      if (cv_scale == 1.0) {
         cv_in = cv->read_fn(cv, time);
      } else {
         cv_in = cv->read_fn(cv, time) * cv_scale;
      }

      float result = (pot_in + cv_in) / 2;
      handle->cached = result;
      return result;
   }
   return handle->cached;
}
