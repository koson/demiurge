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
#include "esp_log.h"
#include "pan.h"


void pan_init(pan_t *handle){
   handle->me.read_fn = NULL;
   handle->me.data = handle;
   handle->left.hostInput = &handle->me;
   handle->right.hostInput = &handle->me;

   handle->left.me.read_fn = panchannel_read;
   handle->left.me.data = &handle->left;
   handle->left.factor = 0.5;
   handle->right.me.read_fn = panchannel_read;
   handle->right.me.data = &handle->right;
   handle->right.factor = -0.5;
}

void pan_configure_control(pan_t *handle, signal_t *control) {
   configASSERT(control != NULL)
   handle->left.control = control;
   handle->right.control = control;
}

float IRAM_ATTR panchannel_read(signal_t *handle, uint64_t time) {
   panchannel_t *panchannel = (panchannel_t *) handle->data;
   if (time > handle->last_calc) {
      handle->last_calc = time;
      signal_t *panControl = panchannel->control;
      float control = panControl->read_fn(panControl, time);
      signal_t *hostInput = panchannel->hostInput;
      float input = hostInput->read_fn(hostInput, time);
      input = input * control;
      float result = input * panchannel->factor;
      handle->cached = result;
      return result;
   }
   return handle->cached;
}
