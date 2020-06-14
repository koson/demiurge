/*
  Copyright 2020, Awesome Audio Apparatus.

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

#include <esp_system.h>
#include <esp_log.h>
#include <math.h>
#include "lg10.h"
#include "signal.h"

void lg10_init(lg10_t *handle) {
   handle->me.read_fn = lg10_read;
   handle->me.data = handle;
}

void lg10_configure_input(lg10_t *handle, signal_t *input) {
   handle->input = input;
}

float IRAM_ATTR lg10_read(signal_t *handle, uint64_t time) {
   if (time > handle->last_calc) {
      handle->last_calc = time;
      lg10_t *lg10 = (lg10_t *) handle->data;
      float input = lg10->input->read_fn(lg10->input, time);
      float new_output = log10f(input);
      handle->cached = new_output;
      return new_output;
   }
   return handle->cached;
}