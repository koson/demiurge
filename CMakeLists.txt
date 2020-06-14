#
# Copyright 2019, Awesome Audio Apparatus.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

idf_component_register(
    SRCS
        adsr.c
        audio_inport.c
        audio_outport.c
        averager.c
        calculator.c
        clipping.c
        control_pair.c
        cv_inport.c
        cv_outport.c
        demiurge.c
        fixed_signal.c
        gate_inport.c
        gate_outport.c
        inverter.c
        led.c
        lg.c
        lg2.c
        lg10.c
        mixer.c
        scale.c
        octave_per_volt.c
        offset.c
        oscillator.c
        pan.c
        passthru.c
        potentiometer.c
        pushbutton.c
        threshold.cpp
        volume.c
        adc128s102/ADC128S102.c
        mcp4822/MCP4822.c
    INCLUDE_DIRS "."
)

