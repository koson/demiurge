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

#include <esp_system.h>
#include <esp_log.h>
#include "passthru.h"

void passthru_init(passthru_t *handle){
   handle->me.read_fn = passthru_read;
   handle->me.data = handle;
}

void passthru_configure_input(passthru_t  *handle, signal_t *input) {
   handle->input = input;
}

float IRAM_ATTR passthru_read(signal_t *handle, uint64_t time) {
   if (time > handle->last_calc) {
      handle->last_calc = time;
      passthru_t *passthru = (passthru_t *) handle->data;
      float input = passthru->input->read_fn(passthru->input, time);
      handle->cached = input;
      return input;
   }
   return handle->cached;
}
