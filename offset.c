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
#include "offset.h"
#include "signal.h"


void offset_init(offset_t *handle) {
   handle->me.read_fn = offset_read;
   handle->me.data = handle;
   handle->offset = 0;
   handle->offset_control = NULL;
   handle->input = NULL;
}

void offset_configure_input(offset_t *handle, signal_t *input) {
   handle->input = input;
}

void offset_configure_control(offset_t *handle, signal_t *control) {
   handle->offset_control = control;
}

void offset_configure(offset_t *handle, signal_t *input, signal_t *offset_control) {
   handle->input = input;
   handle->offset_control = offset_control;
}

float IRAM_ATTR offset_read(signal_t *handle, uint64_t time){
   if (time > handle->last_calc) {
      handle->last_calc = time;
      offset_t *offset = (offset_t *) handle->data;
      float input = offset->input->read_fn(offset->input, time);
      float  new_output;

      if (offset->offset_control != NULL) {
         float new_offset = handle->read_fn(handle, time);
         new_output = input + new_offset;
      } else {
         new_output = input + offset->offset;
      }
      handle->cached = new_output;
      return new_output;
   }
   return handle->cached;
}
