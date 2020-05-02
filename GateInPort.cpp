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
#include <driver/gpio.h>
#include "Clipping.h"
#include "GateInPort.h"
#include "Demiurge.h"

GateInPort::GateInPort(int position) {
   ESP_LOGI("GateInPort", "Constructor: %llx at position %d", (uint64_t) this, position );
   configASSERT(position >= 0 && position <= 4 )
   _signal.data = &_data;
   _signal.read_fn = gateinport_read;
   _data.position = position;
}

GateInPort::~GateInPort() = default;

float IRAM_ATTR gateinport_read(signal_t *handle, uint64_t time){
   if( time > handle->last_calc ) {
      handle->last_calc = time;
      auto *port = (gate_in_port_t *) handle->data;
      handle->extra1 = port->position+1;
      // if position == 0, then use digital input, otherwise use analog inputs.
      float result;
      if( port->position ) {
         float input = Demiurge::runtime().input(port->position);
         result = clipGate(input);
      }
      else {
         bool state = Demiurge::runtime().gpio(32);
         result = state ? DEMIURGE_GATE_HIGH : DEMIURGE_GATE_LOW;
      }
      handle->extra2 = result;
      handle->cached = result;
   }
   return handle->cached;
}
