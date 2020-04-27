/*
  Copyright 2019-2020, Awesome Audio Apparatus.

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

#include "Arduino.h"
#include "Demiurge.h"

AudioInPort in1(1);
AudioInPort in2(3);

ControlPair pair1(2);
ControlPair pair2(4);

AudioOutPort out1(1);
AudioOutPort out2(2);

// Mixing 2 channels
Mixer mixer(2);

/*
 * A two port Mixer, with CV control
 */
void setup() {
   disableCore0WDT();

   mixer.configure(1, &in1, &pair1);
   mixer.configure(2, &in2, &pair2);

   out1.configure(&mixer);
   out2.configure(&mixer);

   Demiurge::runtime().begin(48000);
}

void loop() {

}
