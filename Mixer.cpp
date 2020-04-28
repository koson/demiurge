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

#include <malloc.h>
#include <esp_system.h>
#include <esp_task.h>
#include <esp_log.h>
#include "Mixer.h"


Mixer::Mixer() {
   ESP_LOGI("Mixer", "Constructor: %llx", (uint64_t) this);
   _signal.data = &_data;
   _signal.read_fn = mixer_read;
   for (auto &input : _data.inputs) {
      input = nullptr;
   }
   _signal.extra1 = -1.0;
}

Mixer::~Mixer() {
   for (auto &input : _data.inputs) {
      if (input != nullptr) {

      }
   }

};

void Mixer::configure(int number, Signal *source, Signal *control) {
   int index = number - 1;
   configASSERT(number > 0 && number <= DEMIURGE_MAX_MIXER_IN)
   configASSERT(_data.inputs[index] == nullptr )

   auto *v = new Volume();
   _data.volumes[index] = v;
   _data.inputs[index] = &v->_signal;
   v->configure(source, control);
}

float IRAM_ATTR mixer_read(signal_t *handle, uint64_t time) {
   if (time > handle->last_calc) {
      handle->last_calc = time;
      auto *mixer = (mixer_t *) handle->data;
      float output = 0;
      int counter = 0;
      for (auto inp : mixer->inputs) {
         if (inp != nullptr) {
            output = output + inp->read_fn(inp, time);
            counter++;
         }
      }
      handle->extra1 = counter;
      if (mixer->inputs[0] != nullptr)
         handle->extra2 = mixer->inputs[0]->cached;
      if (mixer->inputs[1] != nullptr)
         handle->extra3 = mixer->inputs[1]->cached;
      if (mixer->inputs[2] != nullptr)
         handle->extra4 = mixer->inputs[2]->cached;
      output = output / counter;
      handle->cached = output;
      return output;
   }
   return handle->cached;
}
