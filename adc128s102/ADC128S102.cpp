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

#include <string.h>
#include <driver/periph_ctrl.h>
#include <soc/mcpwm_struct.h>
#include <soc/mcpwm_reg.h>
#include <driver/mcpwm.h>
#include <soc/spi_struct.h>
#include <esp_log.h>
#include <driver/spi_common_internal.h>

#include "ADC128S102.h"

#define TAG "ADC128S102"

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

ADC128S102::ADC128S102(gpio_num_t mosi_pin, gpio_num_t miso_pin, gpio_num_t sclk_pin, gpio_num_t cs_pin) {
   spiHw = &SPI3;
   const bool spi_periph_claimed = spicommon_periph_claim(VSPI_HOST, "demiurge");
   if (!spi_periph_claimed) {
      ESP_ERROR_CHECK(DEMIURGE_ESP_ERR_SPI_HOST_ALREADY_IN_USE);
   }
   //Use GPIO
   PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[mosi_pin], PIN_FUNC_GPIO);
   PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[sclk_pin], PIN_FUNC_GPIO);
   PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[miso_pin], PIN_FUNC_GPIO);

   gpio_set_direction(mosi_pin, GPIO_MODE_INPUT_OUTPUT);
   gpio_set_direction(sclk_pin, GPIO_MODE_INPUT_OUTPUT);
   if (miso_pin < GPIO_NUM_MAX) {
      gpio_set_direction(miso_pin, GPIO_MODE_INPUT);
      gpio_set_pull_mode(miso_pin, GPIO_PULLUP_ONLY);
   }

   gpio_matrix_out(mosi_pin, VSPID_OUT_IDX, false, false);
   gpio_matrix_in(mosi_pin, VSPID_IN_IDX, false);

   gpio_matrix_out(sclk_pin, VSPICLK_OUT_IDX, false, false);
   gpio_matrix_in(sclk_pin, VSPICLK_IN_IDX, false);

   PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[cs_pin], PIN_FUNC_GPIO);
   gpio_set_direction(cs_pin, GPIO_MODE_INPUT_OUTPUT);
   gpio_matrix_out(cs_pin, VSPICS0_OUT_IDX, false, false);
   gpio_matrix_in(cs_pin, VSPICS0_IN_IDX, false);

   spiHw->user.usr_command = 0;
   spiHw->pin.cs_keep_active = 0;
   spiHw->pin.master_cs_pol = 0;
   spiHw->ctrl2.cs_delay_mode = 0;
   spiHw->ctrl2.cs_delay_num = 0;
   spiHw->user.cs_hold = 1;
   spiHw->user.cs_setup = 1;
   spiHw->mosi_dlen.usr_mosi_dbitlen = 127;
   spiHw->miso_dlen.usr_miso_dbitlen = 127;
   spiHw->user.wr_byte_order = 0;
   spiHw->user.rd_byte_order = 1;

   spiHw->user.usr_mosi = 1;
   spiHw->user.usr_miso = 1;
   spiHw->user.doutdin = 1;
   spiHw->user1.usr_addr_bitlen = 0;
   spiHw->user1.usr_dummy_cyclelen = 0;
   spiHw->user2.usr_command_bitlen = 0;
   initialize_spi_clocks(spiHw);
}

ADC128S102::~ADC128S102() {
   // Stop SPI
   spicommon_periph_free(VSPI_HOST);
}

static uint16_t pattern[8] = {
      1 << 3,
      2 << 3,
      3 << 3,
      4 << 3,
      5 << 3,
      6 << 3,
      7 << 3,
      0
};

void ADC128S102::copy_buffer(void *dest) {
   // get previous cycle's data.
   memcpy((uint8_t *) dest, (void *) &spiHw->data_buf, 16);
   memcpy((void *) &spiHw->data_buf, (void *) pattern, 16);
   spiHw->cmd.usr = 1; // start SPI transfer
}

#undef TAG