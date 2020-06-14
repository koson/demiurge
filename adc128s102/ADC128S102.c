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

void adc128s102_initialize(adc128s102_t *adc) {
   const bool spi_periph_claimed = spicommon_periph_claim(VSPI_HOST, "demiurge");
   if (!spi_periph_claimed) {
      ESP_ERROR_CHECK(DEMIURGE_ESP_ERR_SPI_HOST_ALREADY_IN_USE);
   }
   //Use GPIO
   PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[adc->mosi_pin], PIN_FUNC_GPIO);
   PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[adc->sclk_pin], PIN_FUNC_GPIO);
   PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[adc->miso_pin], PIN_FUNC_GPIO);

   gpio_set_direction(adc->mosi_pin, GPIO_MODE_INPUT_OUTPUT);
   gpio_set_direction(adc->sclk_pin, GPIO_MODE_INPUT_OUTPUT);
   gpio_set_direction(adc->miso_pin, GPIO_MODE_INPUT);
   gpio_set_pull_mode(adc->miso_pin, GPIO_PULLUP_ONLY);

   gpio_matrix_out(adc->mosi_pin, VSPID_OUT_IDX, false, false);
   gpio_matrix_in(adc->mosi_pin, VSPID_IN_IDX, false);

   gpio_matrix_out(adc->sclk_pin, VSPICLK_OUT_IDX, false, false);
   gpio_matrix_in(adc->sclk_pin, VSPICLK_IN_IDX, false);

   PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[adc->cs_pin], PIN_FUNC_GPIO);
   gpio_set_direction(adc->cs_pin, GPIO_MODE_INPUT_OUTPUT);
   gpio_matrix_out(adc->cs_pin, VSPICS0_OUT_IDX, false, false);
   gpio_matrix_in(adc->cs_pin, VSPICS0_IN_IDX, false);

   adc->spiHw->user.usr_command = 0;
   adc->spiHw->mosi_dlen.usr_mosi_dbitlen = 127;
   adc->spiHw->miso_dlen.usr_miso_dbitlen = 127;
   adc->spiHw->user.wr_byte_order = 1;
   adc->spiHw->user.rd_byte_order = 1;

   adc->spiHw->user.usr_mosi = 1;
   adc->spiHw->user.usr_miso = 1;
   adc->spiHw->user.doutdin = 1;
   adc->spiHw->user1.usr_addr_bitlen = 0;
   adc->spiHw->user1.usr_dummy_cyclelen = 0;
   adc->spiHw->user2.usr_command_bitlen = 0;
   initialize_spi_clocks(adc->spiHw);
}

//                        01234567012345670123456701234567
static uint32_t ch1_2 = 0b00001000000000000001000000000000;
static uint32_t ch3_4 = 0b00011000000000000010000000000000;
static uint32_t ch5_6 = 0b00101000000000000011000000000000;
static uint32_t ch7_0 = 0b00111000000000000000000000000000;

void adc128s102_read(adc128s102_t *adc, float *ch) {
   // get previous cycle's data.
   volatile uint32_t *buf = adc->spiHw->data_buf;
   ch[3] = -(((buf[0] << 8) + buf[1]) / 204.8f - 10.0f);
   ch[2] = -(((buf[2] << 8) + buf[3]) / 204.8f - 10.0f);
   ch[1] = -(((buf[4] << 8) + buf[5]) / 204.8f - 10.0f);
   ch[0] = -(((buf[6] << 8) + buf[7]) / 204.8f - 10.0f);
   ch[7] = -(((buf[8] << 8) + buf[9]) / 204.8f - 10.0f);
   ch[6] = -(((buf[10] << 8) + buf[11]) / 204.8f - 10.0f);
   ch[5] = -(((buf[12] << 8) + buf[13]) / 204.8f - 10.0f);
   ch[4] = -(((buf[14] << 8) + buf[15]) / 204.8f - 10.0f);

   // Set up next read cycle.
   buf[0] = ch1_2;
   buf[1] = ch3_4;
   buf[2] = ch5_6;
   buf[3] = ch7_0;

   // start SPI transfer
   adc->spiHw->cmd.usr = 1;
}

#undef TAG