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
             AudioOutPort.cpp  
             Clipping.cpp     
             CvOutPort.cpp    
             GateInPort.cpp   
             LED.cpp    
             Log.cpp         
             OctavePerVolt.cpp  
             Passthru.cpp       
             Signal.cpp
             Adsr.cpp         
             Averager.cpp      
             ControlPair.cpp  
             Demiurge.cpp     
             GateOutPort.cpp  
             Log10.cpp  
             Mixer.cpp       
             Oscillator.cpp     
             Potentiometer.cpp  
             Threshold.cpp
             AudioInPort.cpp  
             Calculator.cpp    
             CvInPort.cpp     
             FixedSignal.cpp  
             Inverter.cpp     
             Log2.cpp   
             Multiplier.cpp  
             Pan.cpp            
             PushButton.cpp     
             Volume.cpp
             adc128s102/ADC128S102.cpp  
             mcp4822/MCP4822.cpp  
             spi/aaa_spi.c
        INCLUDE_DIRS "."
)

