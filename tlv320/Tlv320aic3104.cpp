
#include <hal/gpio_types.h>
#include <driver/i2s.h>
#include <esp32/rom/lldesc.h>
#include "hal/i2s_hal.h"
#include <esp_log.h>
#include <cstring>
#include "Tlv320aic3104.h"

#define TAG "TLV320"

#define DMA_SIZE 128

void Tlv320aic3104::setup_setup_ll_desc() {
   out = static_cast<lldesc_t *>(heap_caps_malloc(sizeof(lldesc_t), MALLOC_CAP_DMA));
   memset((void *) out, 0, sizeof(lldesc_t));
   out->size = DMA_SIZE;
   out->length = DMA_SIZE;
   out->offset = 0;
   out->sosf = 0;
   out->eof = 0;
   out->qe.stqe_next = out;
   out->buf = static_cast<uint8_t *>(heap_caps_malloc(DMA_SIZE, MALLOC_CAP_DMA));
   memset((void *) out->buf, 0, 4);
   out->owner = 1;

   in = static_cast<lldesc_t *>(heap_caps_malloc(sizeof(lldesc_t), MALLOC_CAP_DMA));
   memset((void *) in, 0, sizeof(lldesc_t));
   in->size = DMA_SIZE;
   in->length = DMA_SIZE;
   in->offset = 0;
   in->sosf = 0;
   in->eof = 0;
   in->qe.stqe_next = in;
   in->buf = static_cast<uint8_t *>(heap_caps_malloc(DMA_SIZE, MALLOC_CAP_DMA));
   memset((void *) in->buf, 0, DMA_SIZE);
   in->owner = 1;
}

void Tlv320aic3104::setup_i2s(gpio_num_t bclk_pin, gpio_num_t ws_pin, gpio_num_t din, gpio_num_t dout) {

   ESP_LOGI(TAG, "Default values in I2S bitfields");
   ESP_LOGI(TAG, "-------------------------------");

   // Needed to enable gpio direction?
//   gpio_set_direction(ws_pin, GPIO_MODE_OUTPUT);
//   gpio_set_direction(bclk_pin, GPIO_MODE_OUTPUT);
//   gpio_set_direction(dout, GPIO_MODE_OUTPUT);
//   gpio_set_direction(din, GPIO_MODE_INPUT);

   rtc_clk_apll_enable(true,39, 15, 7, 1);
   if (channel == 0) {
      i2s_context.dev = &I2S0;
      periph_module_enable(PERIPH_I2S0_MODULE);
      gpio_matrix_out(ws_pin, I2S0O_WS_OUT_IDX, false, false);
      gpio_matrix_out(dout, I2S0O_DATA_OUT23_IDX, false, false);
      gpio_matrix_in(din, I2S0I_DATA_IN15_IDX, false);
      gpio_matrix_out(bclk_pin, I2S0O_BCK_OUT_IDX, false, false);
      gpio_matrix_in(bclk_pin, I2S0I_BCK_IN_IDX, false); // route BCK_OUT back to BCK_IN
      gpio_matrix_in(ws_pin, I2S0I_WS_IN_IDX, false);    // route WS_OUT back to WS_IN
   } else {
      i2s_context.dev = &I2S1;
      periph_module_enable(PERIPH_I2S1_MODULE);
      gpio_matrix_out(bclk_pin, I2S1O_BCK_OUT_IDX, false, false);
      gpio_matrix_out(ws_pin, I2S1O_WS_OUT_IDX, false, false);
      gpio_matrix_out(dout, I2S1O_DATA_OUT23_IDX, false, false);
      gpio_matrix_in(din, I2S1I_DATA_IN15_IDX, false);
      gpio_matrix_in(bclk_pin, I2S1I_BCK_IN_IDX, false); // route BCK_OUT back to BCK_IN
      gpio_matrix_in(ws_pin, I2S1I_WS_IN_IDX, false);    // route WS_OUT back to WS_IN
   }
//   i2s_print(i2s_context.dev); // causes Linking error if included. Why??

   setup_setup_ll_desc();

   ESP_LOGI(TAG, "DOUT: %d", dout);
   ESP_LOGI(TAG, " DIN: %d", din);
   ESP_LOGI(TAG, " BCK: %d", bclk_pin);
   ESP_LOGI(TAG, "  WS: %d", ws_pin);

//   i2s_config_t config = {
//         .mode = I2S_MODE_MASTER,
//         .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
//         .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
//         .communication_format = I2S_COMM_FORMAT_I2S,
//         .intr_alloc_flags = 0,
//         .dma_buf_count = 1,
//         .dma_buf_len = 8,
//         .use_apll = false,
//         .tx_desc_auto_clear = false,
//         .fixed_mclk = 1
//   };
//   i2s_hal_init(&i2s_context, channel);
//   i2s_hal_config_param(&i2s_context, &config);
//   i2s_hal_enable_sig_loopback(&i2s_context);
//   i2s_hal_set_rx_mode(&i2s_context, I2S_CHANNEL_STEREO, I2S_BITS_PER_SAMPLE_16BIT);
//   i2s_hal_set_tx_mode(&i2s_context, I2S_CHANNEL_STEREO, I2S_BITS_PER_SAMPLE_16BIT);
//   i2s_hal_set_tx_bits_mod(&i2s_context, I2S_BITS_PER_SAMPLE_16BIT);
//   i2s_hal_set_rx_bits_mod(&i2s_context, I2S_BITS_PER_SAMPLE_16BIT);
//   i2s_hal_enable_master_mode(&i2s_context);
//
//   i2s_hal_set_in_link(&i2s_context, 4, (uint64_t) (void *) in);
//   i2s_hal_set_out_link_addr(&i2s_context, (uint64_t) (void *) out);

   int num = 6;         // max 8 bits
   int a = 0;           // max 6 bits
   int b = 0;           // max 6 bits
   int bck_div = 8;     // max 6 bits, 1536kHz * 8 = 12.288 MHz
//   i2s_hal_set_clk_div(&i2s_context, num, a, b, bck_div, bck_div);
//   i2s_hal_reset_fifo(&i2s_context);
//   i2s_hal_reset(&i2s_context);
//   i2s_hal_start_tx(&i2s_context);
//   i2s_hal_start_rx(&i2s_context);


   // Software should configure the I2S DMA as follows:
   //    1.Configure I2S-controller-related registers;
   //    2.Reset the DMA state machine and FIFO parameters;
   //    3.Configure DMA-related registers for operation;
   //    4.In I2S master mode, set I2S_TX_START bit or I2S_RX_START bit to initiate an I2S operation;
   //      In I2S slave mode, set I2S_TX_START bit or I2S_RX_START bit and wait for data transfer to be initiated by the host device.


   //    1.Configure I2S-controller-related registers;
   i2s_context.dev->conf.tx_slave_mod = 0;                  // Set this bit to enable slave transmitter mode. (R/W)
   i2s_context.dev->conf.rx_slave_mod = 0;                  // Set this bit to enable slave receiver mode. (R/W)
   i2s_context.dev->conf.tx_right_first = 0;                // Set this bit to transmit right-channel data first. (R/W)
   i2s_context.dev->conf.rx_right_first = 0;                // Set this bit to receive right-channel data first. (R/W)
   i2s_context.dev->conf.tx_msb_shift = 0;                  // Set this bit to enable transmitter in Philips standard mode. (R/W)
   i2s_context.dev->conf.rx_msb_shift = 0;                  // Set this bit to enable receiver in Philips standard mode. (R/W)
   i2s_context.dev->conf.tx_short_sync = 0;                 // Set this bit to enable transmitter in PCM standard mode. (R/W)
   i2s_context.dev->conf.rx_short_sync = 0;                 // Set this bit to enable receiver in PCM standard mode. (R/W)
   i2s_context.dev->conf.tx_mono = 0;                       // Set this bit to enable transmitter’s mono mode in PCM standard mode. (R/W)
   i2s_context.dev->conf.rx_mono = 0;                       // Set this bit to enable receiver’s mono mode in PCM standard mode. (R/W)
   i2s_context.dev->conf.tx_msb_right = 0;                  // Set this bit to place right-channel data at the MSB in the transmit FIFO. (R/W)
   i2s_context.dev->conf.rx_msb_right = 0;                  // Set this to place right-channel data at the MSB in the receive FIFO. (R/W)
   i2s_context.dev->conf.sig_loopback = 1;                  // Enable signal loopback mode, with transmitter module and receiver modulesharing the same WS and BCK signals. (R/W)

   i2s_context.dev->clkm_conf.clkm_div_num = num;           // I2S clock divider’s integral value. (R/W)
   i2s_context.dev->clkm_conf.clkm_div_b = b;               // Fractional clock divider’s numerator value. (R/W)
   i2s_context.dev->clkm_conf.clkm_div_a = a;               // Fractional clock divider’s denominator value. (R/W)
   i2s_context.dev->clkm_conf.clk_en = 0;                   // (undocumented)
   i2s_context.dev->clkm_conf.clka_en = 1;                  // Set this bit to enable clk_apll. (R/W)

   //    2.Reset the DMA state machine and FIFO parameters;
   i2s_context.dev->conf.tx_fifo_reset = 1;                 // Set this bit to reset the transmit FIFO. (R/W)
   i2s_context.dev->conf.rx_fifo_reset = 1;                 // Set this bit to reset the receive FIFO. (R/W)
   i2s_context.dev->conf.tx_reset = 1;                      // Set this bit to reset the transmitter. (R/W)
   i2s_context.dev->conf.rx_reset = 1;                      // Set this bit to reset the receiver. (R/W)

   i2s_context.dev->lc_conf.in_rst = 1;                     // Set this bit to reset in DMA FSM. (R/W)
   i2s_context.dev->lc_conf.out_rst = 1;                    // Set this bit to reset out DMA FSM. (R/W)
   i2s_context.dev->lc_conf.ahbm_fifo_rst = 1;              // Set this bit to reset AHB interface cmd FIFO of DMA. (R/W)
   i2s_context.dev->lc_conf.ahbm_rst = 1;                   // Set this bit to reset AHB interface of DMA. (R/W)

// The below hangs forever
//   while( i2s_context.dev->state.tx_fifo_reset_back );      // This bit is used to confirm if the Tx FIFO reset is done.
//                                                            //       1: reset is not ready;
//                                                            //       0: reset is ready. (RO)
//   while( i2s_context.dev->state.rx_fifo_reset_back );      // This bit is used to confirm if the Rx FIFO reset is done.
//                                                            //       1: reset is not ready;
//                                                            //       0: reset is ready. (RO)
   ESP_LOGD(TAG, "Fifo Rx Reset: %d", i2s_context.dev->state.rx_fifo_reset_back);
   ESP_LOGD(TAG, "Fifo Tx Reset: %d", i2s_context.dev->state.tx_fifo_reset_back);

   //    3.Configure DMA-related registers for operation;
   i2s_context.dev->fifo_conf.rx_data_num = 8;              // Threshold of data length in the receive FIFO. (R/W)
   i2s_context.dev->fifo_conf.tx_data_num = 8;              // Threshold of data length in the transmit FIFO. (R/W)
   i2s_context.dev->fifo_conf.dscr_en = 1;                  // Set this bit to enable I2S DMA mode. (R/W)
   i2s_context.dev->fifo_conf.tx_fifo_mod = 0;              // Transmit FIFO mode configuration bit. (R/W)
                                                            // Tx FIFO mode0
                                                            //       0  16-bit dual channel data
                                                            //       2  32-bit dual channel data
                                                            //       3  32-bit single channel data
                                                            // Tx FIFO mode1
                                                            //       1  16-bit single channel data.
   i2s_context.dev->fifo_conf.rx_fifo_mod = 0;              // Receive FIFO mode configuration bit. (R/W)
                                                            //       0  16-bit dual channel data
                                                            //       1  16-bit single channel data
                                                            //       2  32-bit dual channel data
                                                            //       3  32-bit single channel data
   i2s_context.dev->fifo_conf.tx_fifo_mod_force_en = 1;     // The bit should always be set to 1. (R/W)
   i2s_context.dev->fifo_conf.rx_fifo_mod_force_en = 1;     // The bit should always be set to 1. (R/W)

   i2s_context.dev->conf_chan.tx_chan_mod = 0;              // I2S transmitter channel mode configuration bits. Please refer to Section 12.4.4 for further details. (R/W)
                                                            //       0:    Dual channel mode
                                                            //       1:    Mono mode. When I2S_TX_MSB_RIGHT equals 0, the left-channel data are ”holding”
                                                            //             their values and the right-channel data change into the left-channel data.
                                                            //             When I2S_TX_MSB_RIGHT equals 1, the right-channel data are ”holding” their values
                                                            //             and the left-channel data change into the right-channel data.
                                                            //       2:    Mono mode. When I2S_TX_MSB_RIGHT equals 0, the right-channel data are ”holding”
                                                            //             their values and the left-channel data change into the right-channel data.
                                                            //             When I2S_TX_MSB_RIGHT equals 1, the left-channel data are ”holding”their values
                                                            //             and the right-channel data change into the left-channel data.
                                                            //       3:    Mono modeWhen I2S_TX_MSB_RIGHT equals 0, the left-channel data are constants
                                                            //             in the range of REG[31:0].When I2S_TX_MSB_RIGHT equals 1, the right-channel
                                                            //             data are constantsin the range of REG[31:0].
                                                            //       4:    Mono modeWhen I2S_TX_MSB_RIGHT equals 0, the right-channel data are constants
                                                            //             in the range of REG[31:0].When I2S_TX_MSB_RIGHT equals 1, the left-channel
                                                            //             data are constantsin the range of REG[31:0].

   i2s_context.dev->conf_chan.rx_chan_mod = 0;              // I2S receiver channel mode configuration bits. Please refer to Section 12.4.5 for further details. (R/W)
                                                            // See Table 61 in Section 12.4.5. Ignored for Dual channel operation, but set to 0 to be safe.

   i2s_context.dev->out_link.addr = (uint64_t) (void *) out;// The address of first outlink descriptor. (R/W)
   i2s_context.dev->out_link.start = 0;                     // Set this bit to start outlink descriptor. (R/W)

   i2s_context.dev->in_link.addr = (uint64_t) (void *)in;   // The address of first inlink descriptor. (R/W)
   i2s_context.dev->in_link.start = 0;                      // Set this bit to start inlink descriptor. (R/W)

   i2s_context.dev->lc_conf.check_owner = 0;                // Set this bit to check the owner bit by hardware. (R/W)

   i2s_context.dev->pd_conf.fifo_force_pu = 1;              // Force FIFO power-up. (R/W)
   i2s_context.dev->conf2.lcd_en = 0;                       // Set this bit to enable LCD mode. (R/W) default:1


   i2s_context.dev->sample_rate_conf.tx_bck_div_num = bck_div;    // Bit clock configuration bit in transmitter mode. (R/W)
   i2s_context.dev->sample_rate_conf.rx_bck_div_num = bck_div;    // Bit clock configuration bit in receiver mode. (R/W)
   i2s_context.dev->sample_rate_conf.tx_bits_mod = 16;     // Set the bits to configure the bit length of I2S transmitter channel. (R/W)
   i2s_context.dev->sample_rate_conf.rx_bits_mod = 16;     // Set the bits to configure the bit length of I2S receiver channel. (R/W)


   //  4.In I2S master mode, set I2S_TX_START bit or I2S_RX_START bit to initiate an I2S operation;
   i2s_context.dev->conf.tx_start = 1;                      // Set this bit to start transmitting data. (R/W)
   i2s_context.dev->conf.rx_start = 1;                      // Set this bit to start receiving data. (R/W)

   ESP_LOGI(TAG, "I2S initialization is done!");
}

Tlv320aic3104::Tlv320aic3104(int channel, gpio_num_t bclk_pin, gpio_num_t ws_pin, gpio_num_t din_pin,
                             gpio_num_t dout_pin) {
   this->channel = channel;
   reset_external_chip();
   configure_external_chip();
   setup_i2s(bclk_pin, ws_pin, din_pin, dout_pin);
}

void Tlv320aic3104::reset_external_chip() const {
   gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
   gpio_set_level(GPIO_NUM_4, false);
   gpio_set_level(GPIO_NUM_4, true); // RESET for >10ns according to 10.3.1 in datasheet

}

void Tlv320aic3104::readInput(int16_t *inputLeft, int16_t *inputRight) {
//   *inputLeft = ((uint16_t *) in->buf)[0];
//   *inputRight = ((uint16_t *) in->buf)[1];
//   *inputLeft = ((uint32_t *) in->buf)[0];
//   *inputRight = ((uint32_t *) in->buf)[1];
   if (this->print_counter++ > 100000) {
      this->print_counter = 0;
      ESP_LOGI(TAG, "In Buffer: %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x",
            in->buf[0],
            in->buf[1],
            in->buf[2],
            in->buf[3],
            in->buf[4],
            in->buf[5],
            in->buf[6],
            in->buf[7],
            in->buf[8],
            in->buf[9],
            in->buf[10],
            in->buf[11],
            in->buf[12],
            in->buf[13],
            in->buf[14],
            in->buf[15]
            );
      ESP_LOGI(TAG, "Out Buffer: %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x",
            out->buf[0],
            out->buf[1],
            out->buf[2],
            out->buf[3],
            out->buf[4],
            out->buf[5],
            out->buf[6],
            out->buf[7],
            out->buf[8],
            out->buf[9],
            out->buf[10],
            out->buf[11],
            out->buf[12],
            out->buf[13],
            out->buf[14],
            out->buf[15]
            );
      ESP_LOGI(TAG, "DMA State: %x, %x", i2s_context.dev->lc_state0, i2s_context.dev->lc_state1);
   }
}

void Tlv320aic3104::setOutput(int16_t outputLeft, int16_t outputRight) {
//   if (i2s_context.dev->state.tx_idle == 0)
   {
      this->idle_counter++;
      if (idle_counter > 50000) {
         idle_counter = 0;
         ESP_LOGI(TAG, "Calc loop is too slow for current sample rate: %08x", i2s_context.dev->state.tx_idle);
      }
   }
   memset((void*) out->buf, 55, DMA_SIZE);
   if (this->print_counter > 100000) {
      ESP_LOGI(TAG, "push: %x", i2s_context.dev->out_fifo_push.val);
   }
}

void Tlv320aic3104::configure_external_chip() {
   // I2C configuration of the TLV320AIC3104
   ESP_LOGI(TAG, "Configuring the TLV320AIC3104 for I2S%d", channel);
}

#undef TAG
