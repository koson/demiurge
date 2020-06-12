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

#include "ADC128S102.h"
#include "../spi/aaa_spi.h"

#define TAG "ADC128S102"

static void initialize(gpio_num_t pin_out) {
   // We use MCPWM0 TIMER 1 Operator 1 for CS generation to DAC.

   PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[pin_out], PIN_FUNC_GPIO);
   gpio_set_direction(pin_out, GPIO_MODE_INPUT_OUTPUT);
   gpio_matrix_out(pin_out, PWM0_OUT1A_IDX, false, false);  // PWM0 operator 1 output A
   periph_module_enable(PERIPH_PWM0_MODULE);

   WRITE_PERI_REG(MCPWM_TIMER_SYNCI_CFG_REG(0), (1 << MCPWM_TIMER1_SYNCISEL_S));
   WRITE_PERI_REG(MCPWM_OPERATOR_TIMERSEL_REG(0), (1 << MCPWM_OPERATOR1_TIMERSEL_S));

   WRITE_PERI_REG(MCPWM_GEN1_TSTMP_A_REG(0), 512);  // 128 cycles LOW after UTEZ.

   // UTEZ= set PWM0A low, UTEA=set PWM0A high
   WRITE_PERI_REG(MCPWM_GEN1_A_REG(0), (1 << MCPWM_GEN1_A_UTEZ_S) | (2 << MCPWM_GEN1_A_UTEA_S));

   // Prescale=4, so timer is 20MHz, 640 clocks per cycle
   WRITE_PERI_REG(MCPWM_TIMER1_CFG0_REG(0), 3 << MCPWM_TIMER1_PRESCALE_S | 639 << MCPWM_TIMER1_PERIOD_S);

   // Continuously running, decrease mode.
   WRITE_PERI_REG(MCPWM_TIMER1_CFG1_REG(0), (1 << MCPWM_TIMER1_MOD_S) | (2 << MCPWM_TIMER1_START_S));
}

ADC128S102::ADC128S102(gpio_num_t mosi_pin, gpio_num_t miso_pin, gpio_num_t sclk_pin, gpio_num_t cs_pin) {
//   out = static_cast<lldesc_t *>(heap_caps_malloc(sizeof(lldesc_t), MALLOC_CAP_DMA));
//
//   memset((void *) out, 0, sizeof(lldesc_t));
//   out->size = 20;
//   out->length = 20;
//   out->offset = 0;
//   out->sosf = 0;
//   out->eof = 0;
//   out->qe.stqe_next = out;
//   out->buf = static_cast<uint8_t *>(heap_caps_malloc(20, MALLOC_CAP_DMA));
//   out->buf[0] = 1 << 3;
//   out->buf[1] = 0;
//   out->buf[2] = 2 << 3;
//   out->buf[3] = 0;
//   out->buf[4] = 3 << 3;
//   out->buf[5] = 0;
//   out->buf[6] = 4 << 3;
//   out->buf[7] = 0;
//   out->buf[8] = 5 << 3;
//   out->buf[9] = 0;
//   out->buf[10] = 6 << 3;
//   out->buf[11] = 0;
//   out->buf[12] = 7 << 3;
//   out->buf[13] = 0;
//   out->buf[14] = 0 << 3;
//   out->buf[15] = 0;
//   out->buf[16] = 255;
//   out->buf[17] = 255;
//   out->buf[18] = 255;
//   out->buf[19] = 255;
//   out->owner = 1;
//
//   in = static_cast<lldesc_t *>(heap_caps_malloc(sizeof(lldesc_t), MALLOC_CAP_DMA));
//   memset((void *) in, 0, sizeof(lldesc_t));
//   in->size = 20;
//   in->length = 20;
//   in->offset = 0;
//   in->sosf = 0;
//   in->eof = 0;
//   in->qe.stqe_next = in;
//   in->buf = static_cast<uint8_t *>(heap_caps_malloc(32, MALLOC_CAP_DMA));
//   memset((void *) in->buf, 0, 32);
//   in->owner = 1;
//
//   // Values to be written during time critical stage
//   auto s1 = (1 << MCPWM_TIMER1_MOD_S) | (2 << MCPWM_TIMER1_START_S);
//   auto s3 = (CONFIG_DEMIURGE_ADC_SYNC << MCPWM_TIMER1_PHASE_S) | (0 << MCPWM_TIMER1_SYNCO_SEL) | (1 << MCPWM_TIMER1_SYNC_SW_S);


   esp_err_t error = aaa_spi_prepare_circular(VSPI_HOST, 2, nullptr, nullptr, 10000000, mosi_pin, miso_pin, sclk_pin, 0);
   ESP_ERROR_CHECK(error);

   spiHw = aaa_spi_get_hw_for_host(VSPI_HOST);

//   portDISABLE_INTERRUPTS();  // No interference in timing.
//   for (int i = 0; i < 2; i++) {  // Make sure SPI Flash fetches doesn't interfere
//      initialize(cs_pin);

//      //Reset DMA
//      spiHw->dma_out_link.start = 0;
//      spiHw->dma_in_link.start = 0;
//      spiHw->cmd.usr = 0;   // SPI: Stop SPI DMA transfer
//
//      // --- sync to known prescaled cycle.
//      auto reg = READ_PERI_REG(MCPWM_TIMER1_STATUS_REG(0));
//      while (reg == READ_PERI_REG(MCPWM_TIMER1_STATUS_REG(0)));

//      spiHw->cmd.val = 0;
//      spiHw->dma_conf.val |= SPI_OUT_RST | SPI_IN_RST | SPI_AHBM_RST | SPI_AHBM_FIFO_RST;
//      spiHw->dma_conf.val &= ~(SPI_OUT_RST | SPI_IN_RST | SPI_AHBM_RST | SPI_AHBM_FIFO_RST);
//      spiHw->dma_conf.out_data_burst_en = 0;

//      spiHw->dma_out_link.start = 1;   // Start SPI DMA transfer (1)  M
//      spiHw->dma_in_link.start = 1;   // Start SPI DMA transfer (1)  M
//      WRITE_PERI_REG(MCPWM_TIMER1_CFG1_REG(0), s1); // start timer 1
//      spiHw->cmd.usr = 1; // start SPI transfer

//      WRITE_PERI_REG(MCPWM_TIMER1_SYNC_REG(0), s3);
//   }
//   portENABLE_INTERRUPTS();
   PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[cs_pin], PIN_FUNC_GPIO);
   gpio_set_direction(cs_pin, GPIO_MODE_INPUT_OUTPUT);
   gpio_matrix_out(cs_pin, VSPICS0_OUT_IDX, false, false);
   gpio_matrix_in(cs_pin, VSPICS0_IN_IDX, false);
   spiHw->pin.cs0_dis = 0;
   spiHw->pin.cs1_dis = 0;
   spiHw->pin.cs2_dis = 0;
   spiHw->pin.cs_keep_active = 0;
   spiHw->pin.master_cs_pol = 0;
   spiHw->ctrl2.cs_delay_mode = 0;
   spiHw->ctrl2.cs_delay_num = 0;
   spiHw->user.cs_hold = 1;
   spiHw->user.cs_setup = 1;
   spiHw->mosi_dlen.usr_mosi_dbitlen = 127;
   spiHw->miso_dlen.usr_miso_dbitlen = 127;

   spiHw->user.usr_mosi = 1;
   spiHw->user.usr_miso = 1;
   spiHw->user.doutdin = 1;
   spiHw->user1.usr_addr_bitlen = 0;
   spiHw->user1.usr_dummy_cyclelen = 0;
   spiHw->user2.usr_command_bitlen = 0;

}

ADC128S102::~ADC128S102() {

   // stop timer 1
//   WRITE_PERI_REG(MCPWM_TIMER1_CFG1_REG(0), READ_PERI_REG(MCPWM_TIMER1_CFG1_REG(0)) & ~MCPWM_TIMER1_MOD_M);

   // Stop SPI
   aaa_spi_release_circular_buffer(VSPI_HOST, 2);

//   free(out);
//   free(in);
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