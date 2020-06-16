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

#define DEMIURGE_ESP_ERR(base, x)      ((esp_err_t) ((base) + (x)))
#define DEMIURGE_ESP_ERR_USER_BASE    (0x40000000)
#define DEMIURGE_ESP_ERR_SPI_BASE     (DEMIURGE_ESP_ERR_USER_BASE + 0x1000 * 1)

#define DEMIURGE_ESP_ERR_SPI(x)        DEMIURGE_ESP_ERR(DEMIURGE_ESP_ERR_SPI_BASE, (x))
#define DEMIURGE_ESP_ERR_SPI_HOST_ALREADY_IN_USE      DEMIURGE_ESP_ERR_SPI(4)


typedef struct {
   gpio_num_t mosi_pin;
   gpio_num_t sclk_pin;
   gpio_num_t cs_pin;
   gpio_num_t ldac_pin;
   spi_dev_t *spiHw;
} mcp4822_t;

void mcp4822_initialize(mcp4822_t *dac);

void mcp4822_set(mcp4822_t *dac, int16_t output1, int16_t output2);

#endif
