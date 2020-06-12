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

#include <driver/gpio.h>
#include "Signal.h"

Signal::Signal() noexcept {
   _signal.read_fn = nullptr;
}

Signal::~Signal() = default;

float Signal::extra1() {
   return _signal.extra1;
};

float Signal::extra2() {
   return _signal.extra2;
};

float Signal::extra3() {
   return _signal.extra3;
};

float Signal::extra4() {
   return _signal.extra4;
};
float Signal::extra5() {
   return _signal.extra5;
};
float Signal::extra6() {
   return _signal.extra6;
};
float Signal::extra7() {
   return _signal.extra7;
};
float Signal::extra8() {
   return _signal.extra8;
};

