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
#include <driver/ledc.h>
#include <driver/mcpwm.h>
#include <driver/periph_ctrl.h>
#include "driver/spi_master.h"
#include <soc/mcpwm_struct.h>
#include <soc/mcpwm_reg.h>
#include <esp_system.h>
#include <esp_log.h>

#include "MCP4822.h"
#include "../spi/aaa_spi.h"

#define TAG "MCP4822"

static void initialize(gpio_num_t pin_out) {

   // We use MCPWM0 TIMER 0 Operator 0 for CS generation to DAC.

   PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[pin_out], PIN_FUNC_GPIO);
   gpio_set_direction(pin_out, GPIO_MODE_INPUT_OUTPUT);
   gpio_matrix_out(pin_out, PWM0_OUT0A_IDX, false, false);  // PWM0 operator 0 output A
   periph_module_enable(PERIPH_PWM0_MODULE);

   WRITE_PERI_REG(MCPWM_TIMER_SYNCI_CFG_REG(0), (1 << MCPWM_TIMER0_SYNCISEL_S));
   WRITE_PERI_REG(MCPWM_OPERATOR_TIMERSEL_REG(0), (0 << MCPWM_OPERATOR0_TIMERSEL_S));

   WRITE_PERI_REG(MCPWM_GEN0_TSTMP_A_REG(0), 128);  // 32 cycles LOW after UTEZ.

   // UTEZ= set PWM0A low, UTEA=set PWM0A high
   WRITE_PERI_REG(MCPWM_GEN0_A_REG(0), (1 << MCPWM_GEN0_A_UTEZ_S) | (2 << MCPWM_GEN0_A_UTEA_S));

   // Prescale=4, so timer is 20MHz, 128 clocks per cycle
   WRITE_PERI_REG(MCPWM_TIMER0_CFG0_REG(0), 3 << MCPWM_TIMER0_PRESCALE_S | 255 << MCPWM_TIMER0_PERIOD_S);

   // Continuously running, decrease mode.
   WRITE_PERI_REG(MCPWM_TIMER0_CFG1_REG(0), (1 << MCPWM_TIMER0_MOD_S) | (2 << MCPWM_TIMER0_START_S));
}

void MCP4822::initialize_spi(gpio_num_t mosi_pin, gpio_num_t sclk_pin)
{
   out = static_cast<lldesc_t *>(heap_caps_malloc(sizeof(lldesc_t), MALLOC_CAP_DMA));
   memset((void *) out, 0, sizeof(lldesc_t));
   out->size = 8;
   out->length = 8;
   out->offset = 0;
   out->sosf = 0;
   out->eof = 0;
   out->owner = 1;
   out->qe.stqe_next = out;
   out->buf = static_cast<uint8_t *>(heap_caps_malloc(16, MALLOC_CAP_DMA));
   out->buf[0] = 0x00;
   out->buf[1] = 0x00;
   out->buf[2] = 0x00;
   out->buf[3] = 0x00;
   out->buf[4] = 0xFF;
   out->buf[5] = 0x55;
   out->buf[6] = 0x55;
   out->buf[7] = 0xFF;

}

MCP4822::MCP4822(gpio_num_t mosi_pin, gpio_num_t sclk_pin, gpio_num_t cs_pin) {

//   initialize_spi(mosi_pin, sclk_pin);
   spiHw = aaa_spi_get_hw_for_host(HSPI_HOST);
   PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[cs_pin], PIN_FUNC_GPIO);
   gpio_set_direction(cs_pin, GPIO_MODE_INPUT_OUTPUT);
   gpio_matrix_out(cs_pin, HSPICS0_OUT_IDX, false, false);
   gpio_matrix_in(cs_pin, HSPICS0_IN_IDX, false);

   esp_err_t er = aaa_spi_prepare_circular(HSPI_HOST, 1, out, nullptr, 10000000, mosi_pin, GPIO_NUM_MAX, sclk_pin, 0);
   ESP_ERROR_CHECK(er);
   spiHw->pin.cs0_dis = 0;
   spiHw->pin.cs1_dis = 0;
   spiHw->pin.cs2_dis = 0;
   spiHw->pin.cs_keep_active = 0;
   spiHw->pin.master_cs_pol = 0;
   spiHw->ctrl2.cs_delay_mode = 0;
   spiHw->ctrl2.cs_delay_num = 0;
   spiHw->user.wr_byte_order = 1;
   spiHw->user.cs_hold = 1;
   spiHw->user.cs_setup = 1;
   spiHw->mosi_dlen.usr_mosi_dbitlen = 31;
   spiHw->user.usr_mosi = 1;

   // Values to be written during time critical stage
//   auto s0 = 1 << SPI_USR_S;
//   auto s1 = (1 << MCPWM_TIMER0_MOD_S) | (2 << MCPWM_TIMER0_START_S);
//   auto s3 = (CONFIG_DEMIURGE_DAC_SYNC << MCPWM_TIMER0_PHASE_S) | (0 << MCPWM_TIMER0_SYNCO_SEL_S) | (1 << MCPWM_TIMER0_SYNC_SW_S);

//   portDISABLE_INTERRUPTS();  // No interference in timing.
//   for (int i = 0; i < 2; i++) {  // Make sure SPI Flash fetches doesn't interfere

//      initialize(cs_pin);
//      spiHw->dma_out_link.start = 0;   // Stop SPI DMA transfer (1)
//      spiHw->cmd.usr = 0;   // SPI: Stop SPI DMA transfer

      // --- sync to known prescaled cycle.
//      auto reg = READ_PERI_REG(MCPWM_TIMER0_STATUS_REG(0));
//      while (reg == READ_PERI_REG(MCPWM_TIMER0_STATUS_REG(0)));

//      spiHw->dma_out_link.start = 1;
//      spiHw->cmd.usr = 1;
//      WRITE_PERI_REG(MCPWM_TIMER0_CFG1_REG(0), s1); // start timer 0
//      WRITE_PERI_REG(SPI_CMD_REG(3), s0); // start SPI transfer

//      WRITE_PERI_REG(MCPWM_TIMER0_SYNC_REG(0), s3);
//   }
//   portENABLE_INTERRUPTS();
}

void MCP4822::setOutput(int16_t out1, int16_t out2) {
   if (out1 > 4095)
      out1 = 4095;
   if (out1 < 0)
      out1 = 0;
   if (out2 > 4095)
      out2 = 4095;
   if (out2 < 0)
      out2 = 0;
   spiHw->data_buf[0] =
         (( MCP4822_CHANNEL_A | MCP4822_ACTIVE | MCP4822_GAIN | out1 ) << 16)
         | (( MCP4822_CHANNEL_B | MCP4822_ACTIVE | MCP4822_GAIN | out2 ) );
   spiHw->cmd.usr = 1;
}

MCP4822::~MCP4822() {
   // Stop hardware
// TODO: Is this actually correct? Take away for now.
//   WRITE_PERI_REG(SPI_CMD_REG(3), READ_PERI_REG(SPI_CMD_REG(3)) & ~SPI_USR_M); // stop SPI transfer
   WRITE_PERI_REG(MCPWM_TIMER0_CFG1_REG(0),
                  READ_PERI_REG(MCPWM_TIMER0_CFG1_REG(0)) & ~MCPWM_TIMER0_MOD_M); // stop timer 0

   aaa_spi_release_circular_buffer(HSPI_HOST, 1);
   free(out);
}

#undef TAG