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
    SRCS Adder.cpp
        Adsr.cpp
        AudioInPort.cpp
        AudioOutPort.cpp
        Averager.cpp
        Calculator.cpp
        Clipping.cpp
        ControlPair.cpp
        CvInPort.cpp
        CvOutPort.cpp
        Demiurge.cpp
        FixedSignal.cpp
        GateInPort.cpp
        GateOutPort.cpp
        Inverter.cpp
        LED.cpp
        Log.cpp
        Log2.cpp
        Log10.cpp
        Mixer.cpp
        Multiplier.cpp
        OctavePerVolt.cpp
        Oscillator.cpp
        Pan.cpp
        Passthru.cpp
        Potentiometer.cpp
        PushButton.cpp
        Signal.cpp
        Threshold.cpp
        Volume.cpp
        adc128s102/ADC128S102.cpp
        mclk/Mclk.cpp
        mcp4822/MCP4822.cpp
        spi/aaa_spi.c
        tlv320/Tlv320aic3104.cpp
        tlv320/i2s_print.c
    INCLUDE_DIRS "."
)

