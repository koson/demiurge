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
#include <esp_log.h>
#include "potentiometer.h"
#include "demiurge.h"


void potentiometer_init(potentiometer_t *handle, int position) {
   configASSERT(position > 0 && position <= 4 )
   handle->me.read_fn = potentiometer_read;
   handle->me.data = handle;
   handle->position = position + DEMIURGE_POTENTIOMETER_OFFSET;
}

float IRAM_ATTR potentiometer_read(signal_t *handle, uint64_t time) {
   if (time > handle->last_calc) {
      handle->last_calc = time;
      potentiometer_t *port = (potentiometer_t *) handle->data;
      float result = demiurge_input(port->position);
      handle->cached = result;
      return result;
   }
   return handle->cached;
}