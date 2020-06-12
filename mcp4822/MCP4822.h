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

#ifndef _DEMIURGE_MCP4822_h_
#define _DEMIURGE_MCP4822_h_

#include "driver/spi_master.h"

#define MCP4822_CHANNEL_A 0x0000
#define MCP4822_CHANNEL_B 0x8000
#define MCP4822_ACTIVE 0x1000
#define MCP4822_GAIN 0x2000

#ifndef CONFIG_DEMIURGE_DAC_SYNC
#define CONFIG_DEMIURGE_DAC_SYNC 158
#endif

class MCP4822
{
public:
   MCP4822(gpio_num_t mosi_pin, gpio_num_t sclk_pin, gpio_num_t cs_pin);
   virtual ~MCP4822();

   void setOutput(int16_t output1, int16_t output2);

   lldesc_t *out;
private:
   void initialize_spi(gpio_num_t mosi_pin, gpio_num_t sclk_pin);

   spi_dev_t *spiHw;
};


#endif