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

#include <driver/ledc.h>
#include <driver/periph_ctrl.h>
#include "driver/spi_master.h"
#include <soc/mcpwm_struct.h>
#include <soc/mcpwm_reg.h>
#include <esp_system.h>
#include <esp_log.h>
#include <driver/spi_common_internal.h>

#include "MCP4822.h"

#define TAG "MCP4822"

static void initialize_spi_clocks(spi_dev_t *spiHw) {
   // Set SPI Clock
   //  Register 7.7: SPI_CLOCK_REG (0x18)
   //
   //		SPI_CLK_EQU_SYSCLK
   //			In master mode, when this bit is set to 1, spi_clk is equal
   //			to system clock; when set to 0, spi_clk is divided from system
   //			clock.
   //
   //		SPI_CLKDIV_PRE
   //			In master mode, the value of this register field is the
   //			pre-divider value for spi_clk, minus one.
   //
   //		SPI_CLKCNT_N
   //			In master mode, this is the divider for spi_clk minus one.
   //			The spi_clk frequency is
   //				system_clock/(SPI_CLKDIV_PRE+1)/(SPI_CLKCNT_N+1).
   //
   //		SPI_CLKCNT_H
   //			For a 50% duty cycle, set this to floor((SPI_CLKCNT_N+1)/2-1)
   //
   //		SPI_CLKCNT_L
   //			In master mode, this must be equal to SPI_CLKCNT_N.
   {
      const double preDivider = 1.0;
      const double apbClockSpeedInHz = APB_CLK_FREQ;
      const double apbDivider = (apbClockSpeedInHz / preDivider / 10000000);

      const int32_t clkdiv_pre = ((int32_t) preDivider) - 1;
      const int32_t clkcnt_n = ((int32_t) apbDivider) - 1;
      const int32_t clkcnt_h = (clkcnt_n + 1) / 2 - 1;
      const int32_t clkcnt_l = clkcnt_n;

      spiHw->clock.clk_equ_sysclk = 0;
      spiHw->clock.clkcnt_n = clkcnt_n;
      spiHw->clock.clkdiv_pre = clkdiv_pre;
      spiHw->clock.clkcnt_h = clkcnt_h;
      spiHw->clock.clkcnt_l = clkcnt_l;
   }
}

void mcp4822_initialize(mcp4822_t *dac) {

   const bool spi_periph_claimed = spicommon_periph_claim(HSPI_HOST, "demiurge");
   if (!spi_periph_claimed) {
      ESP_ERROR_CHECK(DEMIURGE_ESP_ERR_SPI_HOST_ALREADY_IN_USE);
   }
   //Use GPIO
   PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[dac->mosi_pin], PIN_FUNC_GPIO);
   PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[dac->sclk_pin], PIN_FUNC_GPIO);
   PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[dac->cs_pin], PIN_FUNC_GPIO);
   PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[dac->ldac_pin], PIN_FUNC_GPIO);

   gpio_set_direction(dac->ldac_pin, GPIO_MODE_OUTPUT);
   gpio_set_direction(dac->cs_pin, GPIO_MODE_OUTPUT);
   gpio_set_direction(dac->mosi_pin, GPIO_MODE_OUTPUT);
   gpio_set_direction(dac->sclk_pin, GPIO_MODE_OUTPUT);

   gpio_matrix_out(dac->mosi_pin, HSPID_OUT_IDX, false, false);
   gpio_matrix_in(dac->mosi_pin, HSPID_IN_IDX, false);

   gpio_matrix_out(dac->sclk_pin, HSPICLK_OUT_IDX, false, false);
   gpio_matrix_in(dac->sclk_pin, HSPICLK_IN_IDX, false);

   gpio_matrix_out(dac->cs_pin, HSPICS0_OUT_IDX, false, false);
   gpio_matrix_in(dac->cs_pin, HSPICS0_IN_IDX, false);

//   gpio_matrix_out(ldac_pin, HSPICS0_OUT_IDX, true, false);
//   gpio_matrix_in(ldac_pin, HSPICS0_IN_IDX, true);
   gpio_set_level(dac->ldac_pin, 0);

   dac->spiHw->user.usr_command = 0;
   dac->spiHw->user.wr_byte_order = 1;
   dac->spiHw->mosi_dlen.usr_mosi_dbitlen = 31;
   dac->spiHw->user.usr_mosi = 1;
   initialize_spi_clocks(dac->spiHw);
   ESP_LOGI(TAG, "DAC Initialized");
}

void mcp4822_set(mcp4822_t *dac, int16_t out1, int16_t out2) {
   if (out1 > 4095)
      out1 = 4095;
   if (out1 < 0)
      out1 = 0;
   if (out2 > 4095)
      out2 = 4095;
   if (out2 < 0)
      out2 = 0;
   dac->spiHw->data_buf[0] =
         ((MCP4822_CHANNEL_A | MCP4822_ACTIVE | MCP4822_GAIN | out1))
         | ((MCP4822_CHANNEL_B | MCP4822_ACTIVE | MCP4822_GAIN | out2) << 16);
   dac->spiHw->cmd.usr = 1;
}
#undef TAG
