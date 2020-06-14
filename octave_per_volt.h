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

#ifndef DEMIURGE_OCTAVEPERVOLT_H
#define DEMIURGE_OCTAVEPERVOLT_H

#include <math.h>

#define TWO_POWER_OF_2_75 6.7271713

float octave_frequencyOf(float voltage);

float octave_voltageOf(float frequency);

#endif