
#include <soc/i2s_reg.h>
#include <hal/gpio_types.h>
#include <hal/i2s_types.h>
#include <esp_intr.h>
#include <driver/i2s.h>
#include <esp32/rom/lldesc.h>
#include "hal/i2s_hal.h"
#include <esp_log.h>
#include <cstring>
#include "Tlv320aic3104.h"

#define TAG "TLV320"

#define DMA_SIZE 8 // FIFO is 32 bits wide, and one FIFO per channel?

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
   out->buf[0] = 1;
   out->buf[1] = 2;
   out->buf[2] = 3;
   out->buf[3] = 4;
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
   memset((void *) in->buf, 0, 4);
   in->owner = 1;
}

void Tlv320aic3104::setup_i2s(gpio_num_t bclk_pin, gpio_num_t ws_pin, gpio_num_t din, gpio_num_t dout) {
   setup_setup_ll_desc();
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

   // CLK = PLL_D2_CLK / (NUM + B/A)
   //    or
   // CLK = APLL_CLK / (NUM + B/A)
   //   =>
   // 12.288 MHz = 40 Mhz / (NUM + B/A)
   //   =>
   // 40/12.288 = NUM + B/A
   // NUM = 3
   // A = 255
   // B = 65

   // bclk = samples/sec * 2 * 16bits  -->  48ksps = 1536,000 bits/sec
   // div = CLK / bclk  -->  8 = 12,288,000 / 1536,000

   int num = 3;         // max 8 bits
   int a = 255;         // max 6 bits
   int b = 65;          // max 6 bits
   int bck_div = 8;     // max 6 bits, 1536kHz * 8 = 12.288 MHz
//   i2s_hal_set_clk_div(&i2s_context, num, a, b, bck_div, bck_div);
//   i2s_hal_reset_fifo(&i2s_context);
//   i2s_hal_reset(&i2s_context);
//   i2s_hal_start_tx(&i2s_context);
//   i2s_hal_start_rx(&i2s_context);


   ESP_LOGI(TAG, "conf.tx_start: %x", i2s_context.dev->conf.tx_start);
   ESP_LOGI(TAG, "conf.rx_start: %x", i2s_context.dev->conf.rx_start);
   ESP_LOGI(TAG, "conf.tx_slave_mod: %x", i2s_context.dev->conf.tx_slave_mod);
   ESP_LOGI(TAG, "conf.rx_slave_mod: %x", i2s_context.dev->conf.rx_slave_mod);
   ESP_LOGI(TAG, "conf.tx_right_first: %x", i2s_context.dev->conf.tx_right_first);
   ESP_LOGI(TAG, "conf.rx_right_first: %x", i2s_context.dev->conf.rx_right_first);
   ESP_LOGI(TAG, "conf.tx_msb_shift: %x", i2s_context.dev->conf.tx_msb_shift);
   ESP_LOGI(TAG, "conf.rx_msb_shift: %x", i2s_context.dev->conf.rx_msb_shift);
   ESP_LOGI(TAG, "conf.tx_short_sync: %x", i2s_context.dev->conf.tx_short_sync);
   ESP_LOGI(TAG, "conf.rx_short_sync: %x", i2s_context.dev->conf.rx_short_sync);
   ESP_LOGI(TAG, "conf.tx_mono: %x", i2s_context.dev->conf.tx_mono);
   ESP_LOGI(TAG, "conf.rx_mono: %x", i2s_context.dev->conf.rx_mono);
   ESP_LOGI(TAG, "conf.tx_msb_right: %x", i2s_context.dev->conf.tx_msb_right);
   ESP_LOGI(TAG, "conf.rx_msb_right: %x", i2s_context.dev->conf.rx_msb_right);
   ESP_LOGI(TAG, "conf.sig_loopback: %x", i2s_context.dev->conf.sig_loopback);
   ESP_LOGI(TAG, "conf.tx_fifo_reset: %x", i2s_context.dev->conf.tx_fifo_reset);
   ESP_LOGI(TAG, "conf.rx_fifo_reset: %x", i2s_context.dev->conf.rx_fifo_reset);
   ESP_LOGI(TAG, "conf.tx_reset: %x", i2s_context.dev->conf.tx_reset);
   ESP_LOGI(TAG, "conf.rx_reset: %x", i2s_context.dev->conf.rx_reset);
   ESP_LOGI(TAG, "int_raw.rx_take_data: %x", i2s_context.dev->int_raw.rx_take_data);
   ESP_LOGI(TAG, "int_raw.tx_put_data: %x", i2s_context.dev->int_raw.tx_put_data);
   ESP_LOGI(TAG, "int_raw.rx_wfull: %x", i2s_context.dev->int_raw.rx_wfull);
   ESP_LOGI(TAG, "int_raw.rx_rempty: %x", i2s_context.dev->int_raw.rx_rempty);
   ESP_LOGI(TAG, "int_raw.tx_wfull: %x", i2s_context.dev->int_raw.tx_wfull);
   ESP_LOGI(TAG, "int_raw.tx_rempty: %x", i2s_context.dev->int_raw.tx_rempty);
   ESP_LOGI(TAG, "int_raw.rx_hung: %x", i2s_context.dev->int_raw.rx_hung);
   ESP_LOGI(TAG, "int_raw.tx_hung: %x", i2s_context.dev->int_raw.tx_hung);
   ESP_LOGI(TAG, "int_raw.in_done: %x", i2s_context.dev->int_raw.in_done);
   ESP_LOGI(TAG, "int_raw.in_suc_eof: %x", i2s_context.dev->int_raw.in_suc_eof);
   ESP_LOGI(TAG, "int_raw.in_err_eof: %x", i2s_context.dev->int_raw.in_err_eof);
   ESP_LOGI(TAG, "int_raw.out_done: %x", i2s_context.dev->int_raw.out_done);
   ESP_LOGI(TAG, "int_raw.out_eof: %x", i2s_context.dev->int_raw.out_eof);
   ESP_LOGI(TAG, "int_raw.in_dscr_err: %x", i2s_context.dev->int_raw.in_dscr_err);
   ESP_LOGI(TAG, "int_raw.out_dscr_err: %x", i2s_context.dev->int_raw.out_dscr_err);
   ESP_LOGI(TAG, "int_raw.in_dscr_empty: %x", i2s_context.dev->int_raw.in_dscr_empty);
   ESP_LOGI(TAG, "int_raw.out_total_eof: %x", i2s_context.dev->int_raw.out_total_eof);
   ESP_LOGI(TAG, "int_st.rx_take_data: %x", i2s_context.dev->int_st.rx_take_data);
   ESP_LOGI(TAG, "int_st.tx_put_data: %x", i2s_context.dev->int_st.tx_put_data);
   ESP_LOGI(TAG, "int_st.rx_wfull: %x", i2s_context.dev->int_st.rx_wfull);
   ESP_LOGI(TAG, "int_st.rx_rempty: %x", i2s_context.dev->int_st.rx_rempty);
   ESP_LOGI(TAG, "int_st.tx_wfull: %x", i2s_context.dev->int_st.tx_wfull);
   ESP_LOGI(TAG, "int_st.tx_rempty: %x", i2s_context.dev->int_st.tx_rempty);
   ESP_LOGI(TAG, "int_st.rx_hung: %x", i2s_context.dev->int_st.rx_hung);
   ESP_LOGI(TAG, "int_st.tx_hung: %x", i2s_context.dev->int_st.tx_hung);
   ESP_LOGI(TAG, "int_st.in_done: %x", i2s_context.dev->int_st.in_done);
   ESP_LOGI(TAG, "int_st.in_suc_eof: %x", i2s_context.dev->int_st.in_suc_eof);
   ESP_LOGI(TAG, "int_st.in_err_eof: %x", i2s_context.dev->int_st.in_err_eof);
   ESP_LOGI(TAG, "int_st.out_done: %x", i2s_context.dev->int_st.out_done);
   ESP_LOGI(TAG, "int_st.out_eof: %x", i2s_context.dev->int_st.out_eof );
   ESP_LOGI(TAG, "int_st.in_dscr_err: %x", i2s_context.dev->int_st.in_dscr_err);
   ESP_LOGI(TAG, "int_st.out_dscr_err: %x", i2s_context.dev->int_st.out_dscr_err);
   ESP_LOGI(TAG, "int_st.in_dscr_empty: %x", i2s_context.dev->int_st.in_dscr_empty);
   ESP_LOGI(TAG, "int_st.out_total_eof: %x", i2s_context.dev->int_st.out_total_eof);
   ESP_LOGI(TAG, "int_ena.rx_take_data: %x", i2s_context.dev->int_ena.rx_take_data);
   ESP_LOGI(TAG, "int_ena.tx_put_data: %x", i2s_context.dev->int_ena.tx_put_data);
   ESP_LOGI(TAG, "int_ena.rx_wfull: %x", i2s_context.dev->int_ena.rx_wfull);
   ESP_LOGI(TAG, "int_ena.rx_rempty: %x", i2s_context.dev->int_ena.rx_rempty);
   ESP_LOGI(TAG, "int_ena.tx_wfull: %x", i2s_context.dev->int_ena.tx_wfull);
   ESP_LOGI(TAG, "int_ena.tx_rempty: %x", i2s_context.dev->int_ena.tx_rempty);
   ESP_LOGI(TAG, "int_ena.rx_hung: %x", i2s_context.dev->int_ena.rx_hung);
   ESP_LOGI(TAG, "int_ena.tx_hung: %x", i2s_context.dev->int_ena.tx_hung);
   ESP_LOGI(TAG, "int_ena.in_done: %x", i2s_context.dev->int_ena.in_done);
   ESP_LOGI(TAG, "int_ena.in_suc_eof: %x", i2s_context.dev->int_ena.in_suc_eof);
   ESP_LOGI(TAG, "int_ena.in_err_eof: %x", i2s_context.dev->int_ena.in_err_eof);
   ESP_LOGI(TAG, "int_ena.out_done: %x", i2s_context.dev->int_ena.out_done);
   ESP_LOGI(TAG, "int_ena.out_eof: %x", i2s_context.dev->int_ena.out_eof);
   ESP_LOGI(TAG, "int_ena.in_dscr_err: %x", i2s_context.dev->int_ena.in_dscr_err);
   ESP_LOGI(TAG, "int_ena.out_dscr_err: %x", i2s_context.dev->int_ena.out_dscr_err);
   ESP_LOGI(TAG, "int_ena.in_dscr_empty: %x", i2s_context.dev->int_ena.in_dscr_empty);
   ESP_LOGI(TAG, "int_ena.out_total_eof: %x", i2s_context.dev->int_ena.out_total_eof);
   ESP_LOGI(TAG, "int_clr.take_data: %x", i2s_context.dev->int_clr.take_data);
   ESP_LOGI(TAG, "int_clr.put_data: %x", i2s_context.dev->int_clr.put_data);
   ESP_LOGI(TAG, "int_clr.rx_wfull: %x", i2s_context.dev->int_clr.rx_wfull);
   ESP_LOGI(TAG, "int_clr.rx_rempty: %x", i2s_context.dev->int_clr.rx_rempty);
   ESP_LOGI(TAG, "int_clr.tx_wfull: %x", i2s_context.dev->int_clr.tx_wfull);
   ESP_LOGI(TAG, "int_clr.tx_rempty: %x", i2s_context.dev->int_clr.tx_rempty);
   ESP_LOGI(TAG, "int_clr.rx_hung: %x", i2s_context.dev->int_clr.rx_hung);
   ESP_LOGI(TAG, "int_clr.tx_hung: %x", i2s_context.dev->int_clr.tx_hung);
   ESP_LOGI(TAG, "int_clr.in_done: %x", i2s_context.dev->int_clr.in_done);
   ESP_LOGI(TAG, "int_clr.in_suc_eof: %x", i2s_context.dev->int_clr.in_suc_eof);
   ESP_LOGI(TAG, "int_clr.in_err_eof: %x", i2s_context.dev->int_clr.in_err_eof);
   ESP_LOGI(TAG, "int_clr.out_done: %x", i2s_context.dev->int_clr.out_done);
   ESP_LOGI(TAG, "int_clr.out_eof: %x", i2s_context.dev->int_clr.out_eof);
   ESP_LOGI(TAG, "int_clr.in_dscr_err: %x", i2s_context.dev->int_clr.in_dscr_err);
   ESP_LOGI(TAG, "int_clr.out_dscr_err: %x", i2s_context.dev->int_clr.out_dscr_err);
   ESP_LOGI(TAG, "int_clr.in_dscr_empty: %x", i2s_context.dev->int_clr.in_dscr_empty);
   ESP_LOGI(TAG, "int_clr.out_total_eof: %x", i2s_context.dev->int_clr.out_total_eof);
   ESP_LOGI(TAG, "timing.tx_bck_in_delay: %x", i2s_context.dev->timing.tx_bck_in_delay);
   ESP_LOGI(TAG, "timing.tx_ws_in_delay: %x", i2s_context.dev->timing.tx_ws_in_delay);
   ESP_LOGI(TAG, "timing.rx_bck_in_delay: %x", i2s_context.dev->timing.rx_bck_in_delay);
   ESP_LOGI(TAG, "timing.rx_ws_in_delay: %x", i2s_context.dev->timing.rx_ws_in_delay);
   ESP_LOGI(TAG, "timing.rx_sd_in_delay: %x", i2s_context.dev->timing.rx_sd_in_delay);
   ESP_LOGI(TAG, "timing.tx_bck_out_delay: %x", i2s_context.dev->timing.tx_bck_out_delay);
   ESP_LOGI(TAG, "timing.tx_ws_out_delay: %x", i2s_context.dev->timing.tx_ws_out_delay);
   ESP_LOGI(TAG, "timing.tx_sd_out_delay: %x", i2s_context.dev->timing.tx_sd_out_delay);
   ESP_LOGI(TAG, "timing.rx_ws_out_delay: %x", i2s_context.dev->timing.rx_ws_out_delay);
   ESP_LOGI(TAG, "timing.rx_bck_out_delay: %x", i2s_context.dev->timing.rx_bck_out_delay);
   ESP_LOGI(TAG, "timing.tx_dsync_sw: %x", i2s_context.dev->timing.tx_dsync_sw);
   ESP_LOGI(TAG, "timing.rx_dsync_sw: %x", i2s_context.dev->timing.rx_dsync_sw);
   ESP_LOGI(TAG, "timing.data_enable_delay: %x", i2s_context.dev->timing.data_enable_delay);
   ESP_LOGI(TAG, "timing.tx_bck_in_inv: %x", i2s_context.dev->timing.tx_bck_in_inv);
   ESP_LOGI(TAG, "fifo_conf.rx_data_num: %x", i2s_context.dev->fifo_conf.rx_data_num);
   ESP_LOGI(TAG, "fifo_conf.tx_data_num: %x", i2s_context.dev->fifo_conf.tx_data_num);
   ESP_LOGI(TAG, "fifo_conf.dscr_en: %x", i2s_context.dev->fifo_conf.dscr_en);
   ESP_LOGI(TAG, "fifo_conf.tx_fifo_mod: %x", i2s_context.dev->fifo_conf.tx_fifo_mod);
   ESP_LOGI(TAG, "fifo_conf.rx_fifo_mod: %x", i2s_context.dev->fifo_conf.rx_fifo_mod);
   ESP_LOGI(TAG, "fifo_conf.tx_fifo_mod_force_en: %x", i2s_context.dev->fifo_conf.tx_fifo_mod_force_en);
   ESP_LOGI(TAG, "fifo_conf.rx_fifo_mod_force_en: %x", i2s_context.dev->fifo_conf.rx_fifo_mod_force_en);
   ESP_LOGI(TAG, "rx_eof_num: %x", i2s_context.dev->rx_eof_num);
   ESP_LOGI(TAG, "conf_single_data: %x", i2s_context.dev->conf_single_data);
   ESP_LOGI(TAG, "conf_chan.tx_chan_mod: %x", i2s_context.dev->conf_chan.tx_chan_mod);
   ESP_LOGI(TAG, "conf_chan.rx_chan_mod: %x", i2s_context.dev->conf_chan.rx_chan_mod);
   ESP_LOGI(TAG, "outlink.addr: %x", i2s_context.dev->out_link.addr);
   ESP_LOGI(TAG, "outlink.stop: %x", i2s_context.dev->out_link.stop);
   ESP_LOGI(TAG, "outlink.start: %x", i2s_context.dev->out_link.start);
   ESP_LOGI(TAG, "outlink.restart: %x", i2s_context.dev->out_link.restart);
   ESP_LOGI(TAG, "outlink.park: %x", i2s_context.dev->out_link.park);
   ESP_LOGI(TAG, "inlink.addr: %x", i2s_context.dev->in_link.addr);
   ESP_LOGI(TAG, "inlink.stop: %x", i2s_context.dev->in_link.stop);
   ESP_LOGI(TAG, "inlink.start: %x", i2s_context.dev->in_link.start);
   ESP_LOGI(TAG, "inlink.restart: %x", i2s_context.dev->in_link.restart);
   ESP_LOGI(TAG, "inlink.park: %x", i2s_context.dev->in_link.park);
   ESP_LOGI(TAG, "out_eof_des_addr: %x", i2s_context.dev->out_eof_des_addr);
   ESP_LOGI(TAG, "in_eof_des_addr: %x", i2s_context.dev->in_eof_des_addr);
   ESP_LOGI(TAG, "out_eof_bfr_des_addr: %x", i2s_context.dev->out_eof_bfr_des_addr);
   ESP_LOGI(TAG, "ahb.test.mode: %x", i2s_context.dev->ahb_test.mode);
   ESP_LOGI(TAG, "ahb.test.addr: %x", i2s_context.dev->ahb_test.addr);
   ESP_LOGI(TAG, "in_link_dscr: %x", i2s_context.dev->in_link_dscr);
   ESP_LOGI(TAG, "in_link_dscr_bf0: %x", i2s_context.dev->in_link_dscr_bf0);
   ESP_LOGI(TAG, "in_link_dscr_bf1: %x", i2s_context.dev->in_link_dscr_bf1);
   ESP_LOGI(TAG, "out_link_dscr: %x", i2s_context.dev->out_link_dscr);
   ESP_LOGI(TAG, "out_link_dscr_bf0: %x", i2s_context.dev->out_link_dscr_bf0);
   ESP_LOGI(TAG, "out_link_dscr_bf1: %x", i2s_context.dev->out_link_dscr_bf1);
   ESP_LOGI(TAG, "lc_conf.in_rst: %x", i2s_context.dev->lc_conf.in_rst);
   ESP_LOGI(TAG, "lc_conf.out_rst: %x", i2s_context.dev->lc_conf.out_rst);
   ESP_LOGI(TAG, "lc_conf.ahbm_fifo_rst: %x", i2s_context.dev->lc_conf.ahbm_fifo_rst);
   ESP_LOGI(TAG, "lc_conf.ahbm_rst: %x", i2s_context.dev->lc_conf.ahbm_rst);
   ESP_LOGI(TAG, "lc_conf.out_loop_test: %x", i2s_context.dev->lc_conf.out_loop_test);
   ESP_LOGI(TAG, "lc_conf.in_loop_test: %x", i2s_context.dev->lc_conf.in_loop_test);
   ESP_LOGI(TAG, "lc_conf.out_auto_wrback: %x", i2s_context.dev->lc_conf.out_auto_wrback);
   ESP_LOGI(TAG, "lc_conf.out_no_restart_clr: %x", i2s_context.dev->lc_conf.out_no_restart_clr);
   ESP_LOGI(TAG, "lc_conf.out_eof_mode: %x", i2s_context.dev->lc_conf.out_eof_mode);
   ESP_LOGI(TAG, "lc_conf.outdscr_burst_en: %x", i2s_context.dev->lc_conf.outdscr_burst_en);
   ESP_LOGI(TAG, "lc_conf.indscr_burst_en: %x", i2s_context.dev->lc_conf.indscr_burst_en);
   ESP_LOGI(TAG, "lc_conf.out_data_burst_en: %x", i2s_context.dev->lc_conf.out_data_burst_en);
   ESP_LOGI(TAG, "lc_conf.check_owner: %x", i2s_context.dev->lc_conf.check_owner);
   ESP_LOGI(TAG, "lc_conf.mem_trans_en: %x", i2s_context.dev->lc_conf.mem_trans_en);
   ESP_LOGI(TAG, "lc_state0: %x", i2s_context.dev->lc_state0);
   ESP_LOGI(TAG, "lc_state1: %x", i2s_context.dev->lc_state1);
   ESP_LOGI(TAG, "out_fifo_push.wdata: %x", i2s_context.dev->out_fifo_push.wdata);
   ESP_LOGI(TAG, "out_fifo_push.push: %x", i2s_context.dev->out_fifo_push.push);
   ESP_LOGI(TAG, "in_fifo_pop.rdata: %x", i2s_context.dev->in_fifo_pop.rdata);
   ESP_LOGI(TAG, "in_fifo_pop.pop: %x", i2s_context.dev->in_fifo_pop.pop);
   ESP_LOGI(TAG, "lc_hung.fifo_timeout: %x", i2s_context.dev->lc_hung_conf.fifo_timeout);
   ESP_LOGI(TAG, "lc_hung.fifo_timeout_shift: %x", i2s_context.dev->lc_hung_conf.fifo_timeout_shift);
   ESP_LOGI(TAG, "lc_hung.fifo_timeout_ena: %x", i2s_context.dev->lc_hung_conf.fifo_timeout_ena);
   ESP_LOGI(TAG, "cvsd_conf0.y_max: %x", i2s_context.dev->cvsd_conf0.y_max);
   ESP_LOGI(TAG, "cvsd_conf0.y_min: %x", i2s_context.dev->cvsd_conf0.y_min);
   ESP_LOGI(TAG, "cvsd_conf1.sigma_max: %x", i2s_context.dev->cvsd_conf1.sigma_max);
   ESP_LOGI(TAG, "cvsd_conf1.sigma_min: %x", i2s_context.dev->cvsd_conf1.sigma_min);
   ESP_LOGI(TAG, "cvsd_conf2.cvsd_k: %x", i2s_context.dev->cvsd_conf2.cvsd_k);
   ESP_LOGI(TAG, "cvsd_conf2.cvsd_j: %x", i2s_context.dev->cvsd_conf2.cvsd_j);
   ESP_LOGI(TAG, "cvsd_conf2.cvsd_beta: %x", i2s_context.dev->cvsd_conf2.cvsd_beta);
   ESP_LOGI(TAG, "cvsd_conf2.cvsd_h: %x", i2s_context.dev->cvsd_conf2.cvsd_h);
   ESP_LOGI(TAG, "plc_conf0.good_pack_max: %x", i2s_context.dev->plc_conf0.good_pack_max);
   ESP_LOGI(TAG, "plc_conf0.n_err_seg: %x", i2s_context.dev->plc_conf0.n_err_seg);
   ESP_LOGI(TAG, "plc_conf0.shift_rate: %x", i2s_context.dev->plc_conf0.shift_rate);
   ESP_LOGI(TAG, "plc_conf0.max_slide_sample: %x", i2s_context.dev->plc_conf0.max_slide_sample);
   ESP_LOGI(TAG, "plc_conf0.pack_len_8k: %x", i2s_context.dev->plc_conf0.pack_len_8k);
   ESP_LOGI(TAG, "plc_conf0.n_min_err: %x", i2s_context.dev->plc_conf0.n_min_err);
   ESP_LOGI(TAG, "plc_conf1.bad_cef_atten_para: %x", i2s_context.dev->plc_conf1.bad_cef_atten_para);
   ESP_LOGI(TAG, "plc_conf1.bad_cef_atten_para_shift: %x", i2s_context.dev->plc_conf1.bad_cef_atten_para_shift);
   ESP_LOGI(TAG, "plc_conf1.bad_ola_win2_para_shift: %x", i2s_context.dev->plc_conf1.bad_ola_win2_para_shift);
   ESP_LOGI(TAG, "plc_conf1.bad_ola_win2_para: %x", i2s_context.dev->plc_conf1.bad_ola_win2_para);
   ESP_LOGI(TAG, "plc_conf1.slide_win_len: %x", i2s_context.dev->plc_conf1.slide_win_len);
   ESP_LOGI(TAG, "plc_conf2.cvsd_seg_mod: %x", i2s_context.dev->plc_conf2.cvsd_seg_mod);
   ESP_LOGI(TAG, "plc_conf2.min_period: %x", i2s_context.dev->plc_conf2.min_period);
   ESP_LOGI(TAG, "esco_conf0.en: %x", i2s_context.dev->esco_conf0.en);
   ESP_LOGI(TAG, "esco_conf0.chan_mod: %x", i2s_context.dev->esco_conf0.chan_mod);
   ESP_LOGI(TAG, "esco_conf0.cvsd_dec_pack_err: %x", i2s_context.dev->esco_conf0.cvsd_dec_pack_err);
   ESP_LOGI(TAG, "esco_conf0.cvsd_pack_len_8k: %x", i2s_context.dev->esco_conf0.cvsd_pack_len_8k);
   ESP_LOGI(TAG, "esco_conf0.cvsd_inf_en: %x", i2s_context.dev->esco_conf0.cvsd_inf_en);
   ESP_LOGI(TAG, "esco_conf0.cvsd_dec_start: %x", i2s_context.dev->esco_conf0.cvsd_dec_start);
   ESP_LOGI(TAG, "esco_conf0.cvsd_dec_reset: %x", i2s_context.dev->esco_conf0.cvsd_dec_reset);
   ESP_LOGI(TAG, "esco_conf0.plc_en: %x", i2s_context.dev->esco_conf0.plc_en);
   ESP_LOGI(TAG, "esco_conf0.plc2dma_en: %x", i2s_context.dev->esco_conf0.plc2dma_en);
   ESP_LOGI(TAG, "sco_conf0.with_en: %x", i2s_context.dev->sco_conf0.with_en);
   ESP_LOGI(TAG, "sco_conf0.no_en: %x", i2s_context.dev->sco_conf0.no_en);
   ESP_LOGI(TAG, "sco_conf0.cvsd_enc_start: %x", i2s_context.dev->sco_conf0.cvsd_enc_start);
   ESP_LOGI(TAG, "sco_conf0.cvsd_enc_reset: %x", i2s_context.dev->sco_conf0.cvsd_enc_reset);
   ESP_LOGI(TAG, "conf1.tx_pcm_conf: %x", i2s_context.dev->conf1.tx_pcm_conf);
   ESP_LOGI(TAG, "conf1.tx_pcm_bypass: %x", i2s_context.dev->conf1.tx_pcm_bypass);
   ESP_LOGI(TAG, "conf1.rx_pcm_conf: %x", i2s_context.dev->conf1.rx_pcm_conf);
   ESP_LOGI(TAG, "conf1.rx_pcm_bypass: %x", i2s_context.dev->conf1.rx_pcm_bypass);
   ESP_LOGI(TAG, "conf1.tx_stop_en: %x", i2s_context.dev->conf1.tx_stop_en);
   ESP_LOGI(TAG, "conf1.tx_zeros_rm_en: %x", i2s_context.dev->conf1.tx_zeros_rm_en);
   ESP_LOGI(TAG, "pd_conf.fifo_force_pd: %x", i2s_context.dev->pd_conf.fifo_force_pd);
   ESP_LOGI(TAG, "pd_conf.fifo_force_pu: %x", i2s_context.dev->pd_conf.fifo_force_pu);
   ESP_LOGI(TAG, "pd_conf.plc_mem_force_pd: %x", i2s_context.dev->pd_conf.plc_mem_force_pd);
   ESP_LOGI(TAG, "pd_conf.plc_mem_force_pu: %x", i2s_context.dev->pd_conf.plc_mem_force_pu);
   ESP_LOGI(TAG, "conf2.camera_en: %x", i2s_context.dev->conf2.camera_en);
   ESP_LOGI(TAG, "conf2.lcd_tx_wrx2_en: %x", i2s_context.dev->conf2.lcd_tx_wrx2_en);
   ESP_LOGI(TAG, "conf2.lcd_tx_sdx2_en: %x", i2s_context.dev->conf2.lcd_tx_sdx2_en);
   ESP_LOGI(TAG, "conf2.data_enable_test_en: %x", i2s_context.dev->conf2.data_enable_test_en);
   ESP_LOGI(TAG, "conf2.data_enable: %x", i2s_context.dev->conf2.data_enable);
   ESP_LOGI(TAG, "conf2.lcd_en: %x", i2s_context.dev->conf2.lcd_en);
   ESP_LOGI(TAG, "conf2.ext_adc_start_en: %x", i2s_context.dev->conf2.ext_adc_start_en);
   ESP_LOGI(TAG, "conf2.inter_valid_en: %x", i2s_context.dev->conf2.inter_valid_en);
   ESP_LOGI(TAG, "clkm_conf.clkm_div_num: %x", i2s_context.dev->clkm_conf.clkm_div_num);
   ESP_LOGI(TAG, "clkm_conf.clkm_div_b: %x", i2s_context.dev->clkm_conf.clkm_div_b);
   ESP_LOGI(TAG, "clkm_conf.clkm_div_a: %x", i2s_context.dev->clkm_conf.clkm_div_a);
   ESP_LOGI(TAG, "clkm_conf.clk_en: %x", i2s_context.dev->clkm_conf.clk_en);
   ESP_LOGI(TAG, "clkm_conf.clka_en: %x", i2s_context.dev->clkm_conf.clka_en);
   ESP_LOGI(TAG, "sample_rate.tx_bck_div_num: %x", i2s_context.dev->sample_rate_conf.tx_bck_div_num);
   ESP_LOGI(TAG, "sample_rate.rx_bck_div_num: %x", i2s_context.dev->sample_rate_conf.rx_bck_div_num);
   ESP_LOGI(TAG, "sample_rate.tx_bits_mod: %x", i2s_context.dev->sample_rate_conf.tx_bits_mod);
   ESP_LOGI(TAG, "sample_rate.rx_bits_mod: %x", i2s_context.dev->sample_rate_conf.rx_bits_mod);
   ESP_LOGI(TAG, "pdm_conf.tx_pdm_en: %x", i2s_context.dev->pdm_conf.tx_pdm_en);
   ESP_LOGI(TAG, "pdm_conf.rx_pdm_en: %x", i2s_context.dev->pdm_conf.rx_pdm_en);
   ESP_LOGI(TAG, "pdm_conf.pcm2pdm_conv_en: %x", i2s_context.dev->pdm_conf.pcm2pdm_conv_en);
   ESP_LOGI(TAG, "pdm_conf.pdm2pcm_conv_en: %x", i2s_context.dev->pdm_conf.pdm2pcm_conv_en);
   ESP_LOGI(TAG, "pdm_conf.tx_sinc_osr2: %x", i2s_context.dev->pdm_conf.tx_sinc_osr2);
   ESP_LOGI(TAG, "pdm_conf.tx_prescale: %x", i2s_context.dev->pdm_conf.tx_prescale);
   ESP_LOGI(TAG, "pdm_conf.tx_hp_in_shift: %x", i2s_context.dev->pdm_conf.tx_hp_in_shift);
   ESP_LOGI(TAG, "pdm_conf.tx_lp_in_shift: %x", i2s_context.dev->pdm_conf.tx_lp_in_shift);
   ESP_LOGI(TAG, "pdm_conf.tx_sinc_in_shift: %x", i2s_context.dev->pdm_conf.tx_sinc_in_shift);
   ESP_LOGI(TAG, "pdm_conf.tx_sigmadelta_in_shift: %x", i2s_context.dev->pdm_conf.tx_sigmadelta_in_shift);
   ESP_LOGI(TAG, "pdm_conf.rx_sinc_dsr_16_en: %x", i2s_context.dev->pdm_conf.rx_sinc_dsr_16_en);
   ESP_LOGI(TAG, "pdm_conf.txhp_bypass: %x", i2s_context.dev->pdm_conf.txhp_bypass);
   ESP_LOGI(TAG, "pdm_freq_conf.tx_pdm_fs: %x", i2s_context.dev->pdm_freq_conf.tx_pdm_fs);
   ESP_LOGI(TAG, "pdm_freq_conf.tx_pdm_fp: %x", i2s_context.dev->pdm_freq_conf.tx_pdm_fp);
   ESP_LOGI(TAG, "state.tx_idle: %x", i2s_context.dev->state.tx_idle);
   ESP_LOGI(TAG, "state.tx_fifo_reset_back: %x", i2s_context.dev->state.tx_fifo_reset_back);
   ESP_LOGI(TAG, "state.rx_fifo_reset_back: %x", i2s_context.dev->state.rx_fifo_reset_back);
   ESP_LOGI(TAG, "date: %x", i2s_context.dev->date);



/*
   i2s_context.dev->conf.tx_start = 1;                      // Set this bit to start transmitting data. (R/W)
   i2s_context.dev->conf.rx_start = 1;                      // Set this bit to start receiving data. (R/W)
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
   i2s_context.dev->conf.tx_fifo_reset = 1;                 // Set this bit to reset the transmit FIFO. (R/W)
   i2s_context.dev->conf.rx_fifo_reset = 1;                 // Set this bit to reset the receive FIFO. (R/W)
   i2s_context.dev->conf.tx_reset = 1;                      // Set this bit to reset the transmitter. (R/W)
   i2s_context.dev->conf.rx_reset = 1;                      // Set this bit to reset the receiver. (R/W)

   i2s_context.dev->int_raw.rx_take_data = 0;               // The raw interrupt status bit for the I2S_RX_TAKE_DATA_INT interrupt. (RO)
   i2s_context.dev->int_raw.tx_put_data = 0;                // The raw interrupt status bit for the I2S_TX_PUT_DATA_INT interrupt.(RO)
   i2s_context.dev->int_raw.rx_wfull = 0;                   // The raw interrupt status bit for the I2S_RX_WFULL_INT interrupt. (RO)
   i2s_context.dev->int_raw.rx_rempty = 0;                  // The raw interrupt status bit for the I2S_RX_REMPTY_IN Tinterrupt.(RO)
   i2s_context.dev->int_raw.tx_wfull = 0;                   // The raw interrupt status bit for the I2S_TX_WFULL_INT interrupt. (RO)
   i2s_context.dev->int_raw.tx_rempty = 0;                  // The raw interrupt status bit for the I2S_TX_REMPTY_INT interrupt. (RO)
   i2s_context.dev->int_raw.rx_hung = 0;                    // The raw interrupt status bit for the I2S_RX_HUNG_INT interrupt. (RO)
   i2s_context.dev->int_raw.tx_hung = 0;                    // The raw interrupt status bit for the I2S_TX_HUNG_INT interrupt. (RO)
   i2s_context.dev->int_raw.in_done = 0;                    // The raw interrupt status bit for the I2S_IN_DONE_INT interrupt. (RO)
   i2s_context.dev->int_raw.in_suc_eof = 0;                 // The raw interrupt status bit for the I2S_IN_SUC_EOF_IN Tinterrupt.(RO)
   i2s_context.dev->int_raw.in_err_eof = 0;                 // (undocumented)
   i2s_context.dev->int_raw.out_done = 0;                   // The raw interrupt status bit for the I2S_OUT_DONE_INT interrupt. (RO)
   i2s_context.dev->int_raw.out_eof = 0;                    // The raw interrupt status bit for the I2S_OUT_EOF_INT interrupt. (RO)
   i2s_context.dev->int_raw.in_dscr_err = 0;                // The raw interrupt status bit for the I2S_IN_DSCR_ERR_IN Tinterrupt.(RO)
   i2s_context.dev->int_raw.out_dscr_err = 0;               // The raw interrupt status bit for the I2S_OUT_DSCR_ERR_INT interrupt. (RO)
   i2s_context.dev->int_raw.in_dscr_empty = 0;              // The raw interrupt status bit for the I2S_IN_DSCR_EMPTY_INT interrupt. (RO)
   i2s_context.dev->int_raw.out_total_eof = 0;              // The raw interrupt status bit for the I2S_OUT_TOTAL_EOF_INT interrupt. (RO)

   i2s_context.dev->int_st.rx_take_data= 0;                 // The masked interrupt status bit for the I2S_RX_TAKE_DATA_INT interrupt. (RO)
   i2s_context.dev->int_st.tx_put_data= 0;                  // The masked interrupt status bit for the I2S_TX_PUT_DATA_INT interrupt. (RO)
   i2s_context.dev->int_st.rx_wfull= 0;                     // The masked interrupt status bit for the I2S_RX_WFULL_INT interrupt. (RO)
   i2s_context.dev->int_st.rx_rempty= 0;                    // The masked interrupt status bit for the I2S_RX_REMPTY_INT interrupt.(RO)
   i2s_context.dev->int_st.tx_wfull= 0;                     // The masked interrupt status bit for the I2S_TX_WFULL_INT interrupt. (RO)
   i2s_context.dev->int_st.tx_rempty= 0;                    // The masked interrupt status bit for the I2S_TX_REMPTY_INT interrupt.(RO)
   i2s_context.dev->int_st.rx_hung= 0;                      // The masked interrupt status bit for the I2S_RX_HUNG_INT interrupt. (RO)
   i2s_context.dev->int_st.tx_hung= 0;                      // The masked interrupt status bit for the I2S_TX_HUNG_INT interrupt. (RO)
   i2s_context.dev->int_st.in_done= 0;                      // The masked interrupt status bit for the I2S_IN_DONE_INT interrupt. (RO)
   i2s_context.dev->int_st.in_suc_eof= 0;                   // The masked interrupt status bit for the I2S_IN_SUC_EOF_INT interrupt.(RO)
   i2s_context.dev->int_st.in_err_eof= 0;                   // (undocumented)
   i2s_context.dev->int_st.out_done= 0;                     // The masked interrupt status bit for the I2S_OUT_DONE_INT interrupt. (RO)
   i2s_context.dev->int_st.out_eof= 0;                      // The masked interrupt status bit for the I2S_OUT_EOF_INT interrupt. (RO)
   i2s_context.dev->int_st.in_dscr_err= 0;                  // The masked interrupt status bit for the I2S_IN_DSCR_ERR_INT interrupt. (RO)
   i2s_context.dev->int_st.out_dscr_err= 0;                 // The masked interrupt status bit for the I2S_OUT_DSCR_ERR_INT interrupt. (RO)
   i2s_context.dev->int_st.in_dscr_empty= 0;                // The masked interrupt status bit for the I2S_IN_DSCR_EMPTY_INT interrupt. (RO)
   i2s_context.dev->int_st.out_total_eof= 0;                // The masked interrupt status bit for the I2S_OUT_TOTAL_EOF_INT interrupt. (RO)

   i2s_context.dev->int_ena.rx_take_data = 0;               // The interrupt enable bit for the I2S_RX_TAKE_DATA_INT interrupt.(R/W)
   i2s_context.dev->int_ena.tx_put_data = 0;                // The interrupt enable bit for the I2S_TX_PUT_DATA_INT interrupt.(R/W)
   i2s_context.dev->int_ena.rx_wfull = 0;                   // The interrupt enable bit for the I2S_RX_WFULL_INT interrupt. (R/W)
   i2s_context.dev->int_ena.rx_rempty = 0;                  // The interrupt enable bit for the I2S_RX_REMPTY_INT interrupt. (R/W)
   i2s_context.dev->int_ena.tx_wfull = 0;                   // The interrupt enable bit for the I2S_TX_WFULL_INT interrupt. (R/W)
   i2s_context.dev->int_ena.tx_rempty = 0;                  // The interrupt enable bit for the I2S_TX_REMPTY_INT interrupt. (R/W)
   i2s_context.dev->int_ena.rx_hung = 0;                    // The interrupt enable bit for the I2S_RX_HUNG_INT interrupt. (R/W)
   i2s_context.dev->int_ena.tx_hung = 0;                    // The interrupt enable bit for the I2S_TX_HUNG_INT interrupt. (R/W)
   i2s_context.dev->int_ena.in_done = 0;                    // The interrupt enable bit for the I2S_IN_DONE_INT interrupt. (R/W)
   i2s_context.dev->int_ena.in_suc_eof = 0;                 // The interrupt enable bit for the I2S_IN_SUC_EOF_INT interrupt. (R/W)
   i2s_context.dev->int_ena.in_err_eof = 0;                 // (undocumented)
    i2s_context.dev->int_ena.out_done = 0;                   // The interrupt enable bit for the I2S_OUT_DONE_INT interrupt. (R/W)
   i2s_context.dev->int_ena.out_eof = 0;                    // The interrupt enable bit for the I2S_OUT_EOF_INT interrupt. (R/W)
   i2s_context.dev->int_ena.in_dscr_err = 0;                // The interrupt enable bit for the I2S_IN_DSCR_ERR_INT interrupt.(R/W)
   i2s_context.dev->int_ena.out_dscr_err = 0;               // The interrupt enable bit for the I2S_OUT_DSCR_ERR_INT interrupt.(R/W)
   i2s_context.dev->int_ena.in_dscr_empty = 0;              // The interrupt enable bit for the I2S_IN_DSCR_EMPTY_INT interrupt. (R/W)
   i2s_context.dev->int_ena.out_total_eof = 0;              // The interrupt enable bit for the I2S_OUT_TOTAL_EOF_INT interrupt. (R/W)

   i2s_context.dev->int_clr.take_data = 0;                  // Set this bit to clear the I2S_RX_TAKE_DATA_INT interrupt.(WO)
   i2s_context.dev->int_clr.put_data = 0;                   // Set this bit to clear the I2S_TX_PUT_DATA_INT interrupt.(WO)
   i2s_context.dev->int_clr.rx_wfull = 0;                   // Set this bit to clear the I2S_RX_WFULL_INT interrupt. (WO)
   i2s_context.dev->int_clr.rx_rempty = 0;                  // Set this bit to clear the I2S_RX_REMPTY_INT interrupt. (WO)
   i2s_context.dev->int_clr.tx_wfull = 0;                   // Set this bit to clear the I2S_TX_WFULL_INT interrupt. (WO)
   i2s_context.dev->int_clr.tx_rempty = 0;                  // Set this bit to clear the I2S_TX_REMPTY_INT interrupt. (WO)
   i2s_context.dev->int_clr.rx_hung = 0;                    // Set this bit to clear the I2S_RX_HUNG_INT interrupt. (WO)
   i2s_context.dev->int_clr.tx_hung = 0;                    // Set this bit to clear the I2S_TX_HUNG_INT interrupt. (WO)
   i2s_context.dev->int_clr.in_done = 0;                    // Set this bit to clear the I2S_IN_DONE_INT interrupt. (WO)
   i2s_context.dev->int_clr.in_suc_eof = 0;                 // Set this bit to clear the I2S_IN_SUC_EOF_INT interrupt. (WO)
   i2s_context.dev->int_clr.in_err_eof = 0;                 // (undocumented)
   i2s_context.dev->int_clr.out_done = 0;                   // Set this bit to clear the I2S_OUT_DONE_INT interrupt. (WO)
   i2s_context.dev->int_clr.out_eof = 0;                    // Set this bit to clear the I2S_OUT_EOF_INT interrupt. (WO)
   i2s_context.dev->int_clr.in_dscr_err = 0;                // Set this bit to clear the I2S_IN_DSCR_ERR_INT interrupt.(WO)
   i2s_context.dev->int_clr.out_dscr_err = 0;               // Set this bit to clear the I2S_OUT_DSCR_ERR_INT interrupt.(WO)
   i2s_context.dev->int_clr.in_dscr_empty = 0;              // Set this bit to clear the I2S_IN_DSCR_EMPTY_INT interrupt. (WO)
   i2s_context.dev->int_clr.out_total_eof = 0;              // Set this bit to clear the I2S_OUT_TOTAL_EOF_INT interrupt. (WO)

   i2s_context.dev->timing.tx_bck_in_delay = 0;             // Number of delay cycles for BCK signal into the transmitter. (R/W)
   i2s_context.dev->timing.tx_ws_in_delay = 0;              // Number of delay cycles for WS signal into the transmitter. (R/W)
   i2s_context.dev->timing.rx_bck_in_delay = 0;             // Number of delay cycles for BCK signal into the receiver. (R/W)
   i2s_context.dev->timing.rx_ws_in_delay = 0;              // Number of delay cycles for WS signal into the receiver. (R/W)
   i2s_context.dev->timing.rx_sd_in_delay = 0;              // Number of delay cycles for SD signal into the receiver. (R/W)
   i2s_context.dev->timing.tx_bck_out_delay = 0;            // Number of delay cycles for BCK signal out of the transmitter. (R/W)
   i2s_context.dev->timing.tx_ws_out_delay = 0;             // Number of delay cycles for WS signal out of the transmitter. (R/W)
   i2s_context.dev->timing.tx_sd_out_delay = 0;             // Number of delay cycles for SD signal out of the transmitter. (R/W)
   i2s_context.dev->timing.rx_ws_out_delay = 0;             // Number of delay cycles for WS signal out of the receiver. (R/W)
   i2s_context.dev->timing.rx_bck_out_delay = 0;            // Number of delay cycles for BCK signal out of the receiver. (R/W)
   i2s_context.dev->timing.tx_dsync_sw = 0;                 // Set this bit to synchronize signals into the transmitter in double sync method.(R/W)
   i2s_context.dev->timing.rx_dsync_sw = 0;                 // Set this bit to synchronize signals into the receiver in double sync method.(R/W)
   i2s_context.dev->timing.data_enable_delay = 0;           // Number of delay cycles for data valid flag. (R/W)
   i2s_context.dev->timing.tx_bck_in_inv = 0;               // Set this bit to invert the BCK signal into the slave transmitter. (R/W)

   i2s_context.dev->fifo_conf.rx_data_num = 0;              // Threshold of data length in the receive FIFO. (R/W)
   i2s_context.dev->fifo_conf.tx_data_num = 0;              // Threshold of data length in the transmit FIFO. (R/W)
   i2s_context.dev->fifo_conf.dscr_en = 0;                  // Set this bit to enable I2S DMA mode. (R/W)
   i2s_context.dev->fifo_conf.tx_fifo_mod = 0;              // Transmit FIFO mode configuration bit. (R/W)
   i2s_context.dev->fifo_conf.rx_fifo_mod = 0;              // Receive FIFO mode configuration bit. (R/W)
   i2s_context.dev->fifo_conf.tx_fifo_mod_force_en = 0;     // The bit should always be set to 1. (R/W)
   i2s_context.dev->fifo_conf.rx_fifo_mod_force_en = 0;     // The bit should always be set to 1. (R/W)

   i2s_context.dev->rx_eof_num = 0;                         // The length of the data to be received. It will trigger I2S_IN_SUC_EOF_INT. (R/W)
   i2s_context.dev->conf_single_data =0;                    // The right channel or the left channel outputs constant values storedin this register according to TX_CHAN_MOD and I2S_TX_MSB_RIGHT. (R/W)

   i2s_context.dev->conf_chan.tx_chan_mod = 0;              // I2S transmitter channel mode configuration bits. Please refer to Section 12.4.4 for further details. (R/W)
   i2s_context.dev->conf_chan.rx_chan_mod = 0;              // I2S receiver channel mode configuration bits. Please refer to Section 12.4.5 for further details. (R/W)

   i2s_context.dev->out_link.addr = 0;                      // The address of first outlink descriptor. (R/W)
   i2s_context.dev->out_link.stop = 0;                      // Set this bit to stop outlink descriptor. (R/W)
   i2s_context.dev->out_link.start = 0;                     // Set this bit to start outlink descriptor. (R/W)
   i2s_context.dev->out_link.restart = 0;                   // Set this bit to restart outlink descriptor. (R/W)
   i2s_context.dev->out_link.park = 0;                      // (undocumented)

   i2s_context.dev->in_link.addr = 0;                       // The address of first inlink descriptor. (R/W)
   i2s_context.dev->in_link.stop = 0;                       // Set this bit to stop inlink descriptor. (R/W)
   i2s_context.dev->in_link.start = 0;                      // Set this bit to start inlink descriptor. (R/W)
   i2s_context.dev->in_link.restart = 0;                    // Set this bit to restart inlink descriptor. (R/W)
   i2s_context.dev->in_link.park = 0;                       // (undocumented)

   i2s_context.dev->out_eof_des_addr = 0;                   // The address of outlink descriptor that produces EOF. (RO)
   i2s_context.dev->in_eof_des_addr = 0;                    // The address of inlink descriptor that produces EOF. (RO)
   i2s_context.dev->out_eof_bfr_des_addr = 0;               // The address of the buffer corresponding to the outlink de-scriptor that produces EOF. (RO)

   i2s_context.dev->ahb_test.mode =0;                       // (undocumented)
   i2s_context.dev->ahb_test.addr =0;                       // (undocumented)

   i2s_context.dev->in_link_dscr = 0;                       // The address of current inlink descriptor. (RO)
   i2s_context.dev->in_link_dscr_bf0 = 0;                   // The address of next inlink descriptor. (RO)
   i2s_context.dev->in_link_dscr_bf1 = 0;                   // The address of next inlink data buffer. (RO)
   i2s_context.dev->out_link_dscr = 0;                      // The address of current outlink descriptor. (RO)
   i2s_context.dev->out_link_dscr_bf0 = 0;                  // The address of next outlink descriptor. (RO)
   i2s_context.dev->out_link_dscr_bf1 = 0;                  // The address of next outlink data buffer. (RO)

   i2s_context.dev->lc_conf.in_rst = 0;                     // Set this bit to reset in DMA FSM. (R/W)
   i2s_context.dev->lc_conf.out_rst = 0;                    // Set this bit to reset out DMA FSM. (R/W)
   i2s_context.dev->lc_conf.ahbm_fifo_rst = 0;              // Set this bit to reset AHB interface cmdFIFO of DMA. (R/W)
   i2s_context.dev->lc_conf.ahbm_rst = 0;                   // Set this bit to reset AHB interface of DMA. (R/W)
   i2s_context.dev->lc_conf.out_loop_test = 0;              // Set this bit to loop test inlink. (R/W)
   i2s_context.dev->lc_conf.in_loop_test = 0;               // Set this bit to loop test outlink. (R/W)
   i2s_context.dev->lc_conf.out_auto_wrback = 0;            // Set this bit to enable automatic outlink-writeback when all the data in txbuffer has been transmitted. (R/W)
   i2s_context.dev->lc_conf.out_no_restart_clr = 0;         // (undocumented)
   i2s_context.dev->lc_conf.out_eof_mode = 0;               // DMAI2S_OUT_EOF_INTgeneration mode. (R/W)
                                                            //       1: When DMA has popped all data from the FIFO;
                                                            //       0: When AHB has pushed all data to the FIFO.
   i2s_context.dev->lc_conf.outdscr_burst_en = 0;           // DMA outlink descriptor transfer mode configuration bit. (R/W)
                                                            //       1: Transfer outlink descriptor in burst mode;
                                                            //       0: Transfer outlink descriptor in byte mode.
   i2s_context.dev->lc_conf.indscr_burst_en = 0;            // DMA inlink descriptor transfer mode configuration bit. (R/W)
                                                            //       1: Transfer inlink descriptor in burst mode;
                                                            //       0: Transfer inlink descriptor in byte mode.
   i2s_context.dev->lc_conf.out_data_burst_en = 0;          // Transmitter data transfer mode configuration bit. (R/W)
                                                            //       1: Transmit data in burst mode;
                                                            //       0: Transmit data in byte mode;
   i2s_context.dev->lc_conf.check_owner = 0;                // Set this bit to check the owner bit by hardware. (R/W)
   i2s_context.dev->lc_conf.mem_trans_en = 0;               // (undocumented)

   i2s_context.dev->lc_state0 = 0;                          // Receiver DMA channel status register. (RO)
   i2s_context.dev->lc_state1 = 0;                          // Transmitter DMA channel status register. (RO)

   i2s_context.dev->out_fifo_push.wdata = 0;                // (undocumented)
   i2s_context.dev->out_fifo_push.push = 0;                 // (undocumented)

   i2s_context.dev->in_fifo_pop.rdata = 0;                  // (undocumented)
   i2s_context.dev->in_fifo_pop.pop = 0;                    // (undocumented)

   i2s_context.dev->lc_hung_conf.fifo_timeout = 0;          // When the value of FIFO hung counter is equal to this bit value, sendingdata-timeout interrupt or receiving data-timeout interrupt will be triggered. (R/W)
   i2s_context.dev->lc_hung_conf.fifo_timeout_shift = 0;    // The bits are used to set the tick counter threshold. The tick counter is reset when the counter value >= 88000/2^i2s_lc_fifo_timeout_shift. (R/W)
   i2s_context.dev->lc_hung_conf.fifo_timeout_ena = 0;      // The enable bit for FIFO timeout. (R/W)

   i2s_context.dev->cvsd_conf0.y_max = 0;                   // (undocumented)
   i2s_context.dev->cvsd_conf0.y_min = 0;                   // (undocumented)

   i2s_context.dev->cvsd_conf1.sigma_max = 0;               // (undocumented)
   i2s_context.dev->cvsd_conf1.sigma_min = 0;               // (undocumented)

   i2s_context.dev->cvsd_conf2.cvsd_k = 0;                  // (undocumented)
   i2s_context.dev->cvsd_conf2.cvsd_j = 0;                  // (undocumented)
   i2s_context.dev->cvsd_conf2.cvsd_beta = 0;               // (undocumented)
   i2s_context.dev->cvsd_conf2.cvsd_h = 0;                  // (undocumented)

   i2s_context.dev->plc_conf0.good_pack_max = 0;            // (undocumented)
   i2s_context.dev->plc_conf0.n_err_seg = 0;                // (undocumented)
   i2s_context.dev->plc_conf0.shift_rate = 0;               // (undocumented)
   i2s_context.dev->plc_conf0.max_slide_sample = 0;         // (undocumented)
   i2s_context.dev->plc_conf0.pack_len_8k = 0;              // (undocumented)
   i2s_context.dev->plc_conf0.n_min_err = 0;                // (undocumented)

   i2s_context.dev->plc_conf1.bad_cef_atten_para = 0;       // (undocumented)
   i2s_context.dev->plc_conf1.bad_cef_atten_para_shift = 0; // (undocumented)
   i2s_context.dev->plc_conf1.bad_ola_win2_para_shift = 0;  // (undocumented)
   i2s_context.dev->plc_conf1.bad_ola_win2_para = 0;        // (undocumented)
   i2s_context.dev->plc_conf1.slide_win_len = 0;            // (undocumented)

   i2s_context.dev->plc_conf2.cvsd_seg_mod = 0;             // (undocumented)
   i2s_context.dev->plc_conf2.min_period = 0;               // (undocumented)

   i2s_context.dev->esco_conf0.en = 0;                      // (undocumented)
   i2s_context.dev->esco_conf0.chan_mod = 0;                // (undocumented)
   i2s_context.dev->esco_conf0.cvsd_dec_pack_err = 0;       // (undocumented)
   i2s_context.dev->esco_conf0.cvsd_pack_len_8k = 0;        // (undocumented)
   i2s_context.dev->esco_conf0.cvsd_inf_en = 0;             // (undocumented)
   i2s_context.dev->esco_conf0.cvsd_dec_start = 0;          // (undocumented)
   i2s_context.dev->esco_conf0.cvsd_dec_reset = 0;          // (undocumented)
   i2s_context.dev->esco_conf0.plc_en = 0;                  // (undocumented)
   i2s_context.dev->esco_conf0.plc2dma_en = 0;              // (undocumented)

   i2s_context.dev->sco_conf0.with_en = 0;                  // (undocumented)
   i2s_context.dev->sco_conf0.no_en = 0;                    // (undocumented)
   i2s_context.dev->sco_conf0.cvsd_enc_start = 0;           // (undocumented)
   i2s_context.dev->sco_conf0.cvsd_enc_reset = 0;           // (undocumented)

   i2s_context.dev->conf1.tx_pcm_conf = 0;                  // Compress/Decompress module configuration bit. (R/W)
                                                            //       0: Decompress transmitted data;
                                                            //       1: Compress transmitted data.
   i2s_context.dev->conf1.tx_pcm_bypass = 0;                // Set this bit to bypass the Compress/Decompress module for the transmitted data. (R/W)
   i2s_context.dev->conf1.rx_pcm_conf = 0;                  // Compress/Decompress module configuration bit. (R/W)
                                                            //       0: Decompress received data;
                                                            //       1: Compress received data.
   i2s_context.dev->conf1.rx_pcm_bypass = 0;                // Set this bit to bypass the Compress/Decompress module for the received data. (R/W)
   i2s_context.dev->conf1.tx_stop_en = 0;                   // Set this bit and the transmitter will stop transmitting BCK signal and WS signal when tx FIFO is empty. (R/W)
   i2s_context.dev->conf1.tx_zeros_rm_en = 0;               // (undocumented)

   i2s_context.dev->pd_conf.fifo_force_pd = 0;              // Force FIFO power-down. (R/W)
   i2s_context.dev->pd_conf.fifo_force_pu = 0;              // Force FIFO power-up. (R/W)
   i2s_context.dev->pd_conf.plc_mem_force_pd = 0;           // (undocumented)
   i2s_context.dev->pd_conf.plc_mem_force_pu = 0;           // (undocumented)

   i2s_context.dev->conf2.camera_en = 0;                    // Set this bit to enable camera mode. (R/W)
   i2s_context.dev->conf2.lcd_tx_wrx2_en = 0;               // One datum will be written twice in LCD mode. (R/W)
   i2s_context.dev->conf2.lcd_tx_sdx2_en = 0;               // Set this bit to duplicate data pairs (Data Frame, Form 2) in LCD mode. (R/W)
   i2s_context.dev->conf2.data_enable_test_en = 0;          // (undocumented)
   i2s_context.dev->conf2.data_enable = 0;                  // (undocumented)
   i2s_context.dev->conf2.lcd_en = 0;                       // Set this bit to enable LCD mode. (R/W)
   i2s_context.dev->conf2.ext_adc_start_en = 0;             // Set this bit to enable the start of external ADC . (R/W)
   i2s_context.dev->conf2.inter_valid_en = 0;               // Set this bit to enable camera’s internal validation. (R/W)

   i2s_context.dev->clkm_conf.clkm_div_num = 0;             // I2S clock divider’s integral value. (R/W)
   i2s_context.dev->clkm_conf.clkm_div_b = 0;               // Fractional clock divider’s numerator value. (R/W)
   i2s_context.dev->clkm_conf.clkm_div_a = 0;               // Fractional clock divider’s denominator value. (R/W)
   i2s_context.dev->clkm_conf.clk_en = 0;                   // (undocumented)
   i2s_context.dev->clkm_conf.clka_en = 0;                  // Set this bit to enable clk_apll. (R/W)

   i2s_context.dev->sample_rate_conf.tx_bck_div_num = 0;    // Bit clock configuration bit in transmitter mode. (R/W)
   i2s_context.dev->sample_rate_conf.rx_bck_div_num = 0;    // Bit clock configuration bit in receiver mode. (R/W)
   i2s_context.dev->sample_rate_conf.tx_bits_mod = 0;       // Set the bits to configure the bit length of I2S transmitter channel. (R/W)
   i2s_context.dev->sample_rate_conf.rx_bits_mod = 0;       // Set the bits to configure the bit length of I2S receiver channel. (R/W)

   i2s_context.dev->pdm_conf.tx_pdm_en = 0;                 // Set this bit to enable transmitter’s PDM mode. (R/W)
   i2s_context.dev->pdm_conf.rx_pdm_en = 0;                 // Set this bit to enable receiver’s PDM mode. (R/W)
   i2s_context.dev->pdm_conf.pcm2pdm_conv_en = 0;           // Set this bit to enable PCM-to-PDM converter. (R/W)
   i2s_context.dev->pdm_conf.pdm2pcm_conv_en = 0;           // Set this bit to enable PDM-to-PCM converter. (R/W)
   i2s_context.dev->pdm_conf.tx_sinc_osr2 = 0;              // Upsampling rate = 64×i2s_tx_pdm_sinc_osr2 (R/W)
   i2s_context.dev->pdm_conf.tx_prescale = 0;               // (undocumented)
   i2s_context.dev->pdm_conf.tx_hp_in_shift = 0;            // Adjust the size of the input signal into filter module. (R/W)
                                                            //       0: divided by 2;
                                                            //       1: multiplied by 1;
                                                            //       2: multiplied by 2;
                                                            //       3: multiplied by 4.
   i2s_context.dev->pdm_conf.tx_lp_in_shift = 0;            // Adjust the size of the input signal into filter module. (R/W)
                                                            //       0: divided by 2;
                                                            //       1: multiplied by 1;
                                                            //       2: multiplied by 2;
                                                            //       3: multiplied by 4.
   i2s_context.dev->pdm_conf.tx_sinc_in_shift = 0;          // Adjust the size of the input signal into filter module. (R/W)
                                                            //       0: divided by 2;
                                                            //       1: multiplied by 1;
                                                            //       2: multiplied by 2;
                                                            //       3: multiplied by 4.
   i2s_context.dev->pdm_conf.tx_sigmadelta_in_shift = 0;    // Adjust the size of the input signal into filter module. (R/W)
                                                            //       0: divided by 2;
                                                            //       1: multiplied by 1;
                                                            //       2: multiplied by 2;
                                                            //       3: multiplied by 4.
   i2s_context.dev->pdm_conf.rx_sinc_dsr_16_en = 0;         // PDM downsampling rate for filter group 1 in receiver mode. (R/W)
                                                            //       1: downsampling rate = 128;
                                                            //       0: downsampling rate = 64.
   i2s_context.dev->pdm_conf.txhp_bypass = 0;               // Set this bit to bypass the transmitter’s PDM HP filter. (R/W)

   i2s_context.dev->pdm_freq_conf.tx_pdm_fs = 0;            // PCM-to-PDM converter’s PCM frequency parameter. (R/W)
   i2s_context.dev->pdm_freq_conf.tx_pdm_fp = 0;            // PCM-to-PDM converter’s PDM frequency parameter. (R/W)

   i2s_context.dev->state.tx_idle = 0;                      // This bit is used to confirm if the Rx FIFO reset is done.
                                                            //       1: reset is not ready;
                                                            //       0: reset is ready. (RO)
   i2s_context.dev->state.tx_fifo_reset_back = 0;           // This bit is used to confirm if the Tx FIFO reset is done.
                                                            //       1: reset is not ready;
                                                            //       0: reset is ready. (RO)
   i2s_context.dev->state.rx_fifo_reset_back = 0;           // This bit is used to confirm if the Rx FIFO reset is done.
                                                            //       1: reset is not ready;
                                                            //       0: reset is ready. (RO)

   i2s_context.dev->date = 0;                               // (undocumented)
*/

   ESP_LOGI(TAG, "\n"
            "          reserved_0 = %08x, \n" \
            "          reserved_4 = %08x, \n" \
            "                conf = %08x, \n" \
            "             int_raw = %08x, \n" \
            "              int_st = %08x, \n" \
            "             int_ena = %08x, \n" \
            "             int_clr = %08x, \n" \
            "              timing = %08x, \n" \
            "           fifo_conf = %08x, \n" \
            "          rx_eof_num = %08x, \n" \
            "    conf_single_data = %08x, \n" \
            "           conf_chan = %08x, \n" \
            "            out_link = %08x, \n" \
            "             in_link = %08x, \n" \
            "    out_eof_des_addr = %08x, \n" \
            "     in_eof_des_addr = %08x, \n" \
            "out_eof_bfr_des_addr = %08x, \n" \
            "            ahb_test = %08x, \n" \
            "        in_link_dscr = %08x, \n" \
            "    in_link_dscr_bf0 = %08x, \n" \
            "    in_link_dscr_bf1 = %08x, \n" \
            "       out_link_dscr = %08x, \n" \
            "   out_link_dscr_bf0 = %08x, \n" \
            "   out_link_dscr_bf1 = %08x, \n" \
            "             lc_conf = %08x, \n" \
            "       out_fifo_push = %08x, \n" \
            "         in_fifo_pop = %08x, \n" \
            "           lc_state0 = %08x, \n" \
            "           lc_state1 = %08x, \n" \
            "        lc_hung_conf = %08x, \n" \
            "         reserved_78 = %08x, \n" \
            "         reserved_7c = %08x, \n" \
            "          cvsd_conf0 = %08x, \n" \
            "          cvsd_conf1 = %08x, \n" \
            "          cvsd_conf2 = %08x, \n" \
            "           plc_conf0 = %08x, \n" \
            "           plc_conf1 = %08x, \n" \
            "           plc_conf2 = %08x, \n" \
            "          esco_conf0 = %08x, \n" \
            "           sco_conf0 = %08x, \n" \
            "               conf1 = %08x, \n" \
            "             pd_conf = %08x, \n" \
            "               conf2 = %08x, \n" \
            "           clkm_conf = %08x, \n" \
            "    sample_rate_conf = %08x, \n" \
            "            pdm_conf = %08x, \n" \
            "       pdm_freq_conf = %08x, \n" \
            "               state = %08x, \n" \
            "         reserved_c0 = %08x, \n" \
            "         reserved_c4 = %08x, \n" \
            "         reserved_c8 = %08x, \n" \
            "         reserved_cc = %08x, \n" \
            "         reserved_d0 = %08x, \n" \
            "         reserved_d4 = %08x, \n" \
            "         reserved_d8 = %08x, \n" \
            "         reserved_dc = %08x, \n" \
            "         reserved_e0 = %08x, \n" \
            "         reserved_e4 = %08x, \n" \
            "         reserved_e8 = %08x, \n" \
            "         reserved_ec = %08x, \n" \
            "         reserved_f0 = %08x, \n" \
            "         reserved_f4 = %08x, \n" \
            "         reserved_f8 = %08x, \n" \
            "                date = %08x, \n",
            i2s_context.dev->reserved_0,
            i2s_context.dev->reserved_4,
            i2s_context.dev->conf.val,
            i2s_context.dev->int_raw.val,
            i2s_context.dev->int_st.val,
            i2s_context.dev->int_ena.val,
            i2s_context.dev->int_clr.val,
            i2s_context.dev->timing.val,
            i2s_context.dev->fifo_conf.val,
            i2s_context.dev->rx_eof_num,
            i2s_context.dev->conf_single_data,
            i2s_context.dev->conf_chan.val,
            i2s_context.dev->out_link.val,
            i2s_context.dev->in_link.val,
            i2s_context.dev->out_eof_des_addr,
            i2s_context.dev->in_eof_des_addr,
            i2s_context.dev->out_eof_bfr_des_addr,
            i2s_context.dev->ahb_test.val,
            i2s_context.dev->in_link_dscr,
            i2s_context.dev->in_link_dscr_bf0,
            i2s_context.dev->in_link_dscr_bf1,
            i2s_context.dev->out_link_dscr,
            i2s_context.dev->out_link_dscr_bf0,
            i2s_context.dev->out_link_dscr_bf1,
            i2s_context.dev->lc_conf.val,
            i2s_context.dev->out_fifo_push.val,
            i2s_context.dev->in_fifo_pop.val,
            i2s_context.dev->lc_state0,
            i2s_context.dev->lc_state1,
            i2s_context.dev->lc_hung_conf.val,
            i2s_context.dev->reserved_78,
            i2s_context.dev->reserved_7c,
            i2s_context.dev->cvsd_conf0.val,
            i2s_context.dev->cvsd_conf1.val,
            i2s_context.dev->cvsd_conf2.val,
            i2s_context.dev->plc_conf0.val,
            i2s_context.dev->plc_conf1.val,
            i2s_context.dev->plc_conf2.val,
            i2s_context.dev->esco_conf0.val,
            i2s_context.dev->sco_conf0.val,
            i2s_context.dev->conf1.val,
            i2s_context.dev->pd_conf.val,
            i2s_context.dev->conf2.val,
            i2s_context.dev->clkm_conf.val,
            i2s_context.dev->sample_rate_conf.val,
            i2s_context.dev->pdm_conf.val,
            i2s_context.dev->pdm_freq_conf.val,
            i2s_context.dev->state.val,
            i2s_context.dev->reserved_c0,
            i2s_context.dev->reserved_c4,
            i2s_context.dev->reserved_c8,
            i2s_context.dev->reserved_cc,
            i2s_context.dev->reserved_d0,
            i2s_context.dev->reserved_d4,
            i2s_context.dev->reserved_d8,
            i2s_context.dev->reserved_dc,
            i2s_context.dev->reserved_e0,
            i2s_context.dev->reserved_e4,
            i2s_context.dev->reserved_e8,
            i2s_context.dev->reserved_ec,
            i2s_context.dev->reserved_f0,
            i2s_context.dev->reserved_f4,
            i2s_context.dev->reserved_f8,
            i2s_context.dev->date);
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
   *inputLeft = ((uint32_t *) in->buf)[0];
   *inputRight = ((uint32_t *) in->buf)[1];
}

void Tlv320aic3104::setOutput(int16_t outputLeft, int16_t outputRight) {
   int tx_idle = READ_PERI_REG(I2S_STATE_REG(channel)) & I2S_TX_IDLE;
   if (tx_idle != 0) {
      this->idle_counter++;
      if (idle_counter > 50000) {
         idle_counter = 0;
         ESP_LOGI(TAG, "Calc loop is too slow for current sample rate: %08x", tx_idle);
      }
   }
   ((uint32_t *) out->buf)[0] = outputLeft;
   ((uint32_t *) out->buf)[1] = outputRight;
}

void Tlv320aic3104::configure_external_chip() {
   // I2C configuration of the TLV320AIC3104
   ESP_LOGI(TAG, "Configuring the TLV320AIC3104 for I2S%d", channel);
}

#undef TAG
