
#include <esp_log.h>
#include <hal/i2s_hal.h>
#include "i2s_print.h"


#define TAG "i2s"
void i2s_print(i2s_dev_t *dev)
{
   ESP_LOGI(TAG, "conf.tx_start: %x", dev->conf.tx_start);
   ESP_LOGI(TAG, "conf.rx_start: %x", dev->conf.rx_start);
   ESP_LOGI(TAG, "conf.tx_slave_mod: %x", dev->conf.tx_slave_mod);
   ESP_LOGI(TAG, "conf.rx_slave_mod: %x", dev->conf.rx_slave_mod);
   ESP_LOGI(TAG, "conf.tx_right_first: %x", dev->conf.tx_right_first);
   ESP_LOGI(TAG, "conf.rx_right_first: %x", dev->conf.rx_right_first);
   ESP_LOGI(TAG, "conf.tx_msb_shift: %x", dev->conf.tx_msb_shift);
   ESP_LOGI(TAG, "conf.rx_msb_shift: %x", dev->conf.rx_msb_shift);
   ESP_LOGI(TAG, "conf.tx_short_sync: %x", dev->conf.tx_short_sync);
   ESP_LOGI(TAG, "conf.rx_short_sync: %x", dev->conf.rx_short_sync);
   ESP_LOGI(TAG, "conf.tx_mono: %x", dev->conf.tx_mono);
   ESP_LOGI(TAG, "conf.rx_mono: %x", dev->conf.rx_mono);
   ESP_LOGI(TAG, "conf.tx_msb_right: %x", dev->conf.tx_msb_right);
   ESP_LOGI(TAG, "conf.rx_msb_right: %x", dev->conf.rx_msb_right);
   ESP_LOGI(TAG, "conf.sig_loopback: %x", dev->conf.sig_loopback);
   ESP_LOGI(TAG, "conf.tx_fifo_reset: %x", dev->conf.tx_fifo_reset);
   ESP_LOGI(TAG, "conf.rx_fifo_reset: %x", dev->conf.rx_fifo_reset);
   ESP_LOGI(TAG, "conf.tx_reset: %x", dev->conf.tx_reset);
   ESP_LOGI(TAG, "conf.rx_reset: %x", dev->conf.rx_reset);
   ESP_LOGI(TAG, "int_raw.rx_take_data: %x", dev->int_raw.rx_take_data);
   ESP_LOGI(TAG, "int_raw.tx_put_data: %x", dev->int_raw.tx_put_data);
   ESP_LOGI(TAG, "int_raw.rx_wfull: %x", dev->int_raw.rx_wfull);
   ESP_LOGI(TAG, "int_raw.rx_rempty: %x", dev->int_raw.rx_rempty);
   ESP_LOGI(TAG, "int_raw.tx_wfull: %x", dev->int_raw.tx_wfull);
   ESP_LOGI(TAG, "int_raw.tx_rempty: %x", dev->int_raw.tx_rempty);
   ESP_LOGI(TAG, "int_raw.rx_hung: %x", dev->int_raw.rx_hung);
   ESP_LOGI(TAG, "int_raw.tx_hung: %x", dev->int_raw.tx_hung);
   ESP_LOGI(TAG, "int_raw.in_done: %x", dev->int_raw.in_done);
   ESP_LOGI(TAG, "int_raw.in_suc_eof: %x", dev->int_raw.in_suc_eof);
   ESP_LOGI(TAG, "int_raw.in_err_eof: %x", dev->int_raw.in_err_eof);
   ESP_LOGI(TAG, "int_raw.out_done: %x", dev->int_raw.out_done);
   ESP_LOGI(TAG, "int_raw.out_eof: %x", dev->int_raw.out_eof);
   ESP_LOGI(TAG, "int_raw.in_dscr_err: %x", dev->int_raw.in_dscr_err);
   ESP_LOGI(TAG, "int_raw.out_dscr_err: %x", dev->int_raw.out_dscr_err);
   ESP_LOGI(TAG, "int_raw.in_dscr_empty: %x", dev->int_raw.in_dscr_empty);
   ESP_LOGI(TAG, "int_raw.out_total_eof: %x", dev->int_raw.out_total_eof);
   ESP_LOGI(TAG, "int_st.rx_take_data: %x", dev->int_st.rx_take_data);
   ESP_LOGI(TAG, "int_st.tx_put_data: %x", dev->int_st.tx_put_data);
   ESP_LOGI(TAG, "int_st.rx_wfull: %x", dev->int_st.rx_wfull);
   ESP_LOGI(TAG, "int_st.rx_rempty: %x", dev->int_st.rx_rempty);
   ESP_LOGI(TAG, "int_st.tx_wfull: %x", dev->int_st.tx_wfull);
   ESP_LOGI(TAG, "int_st.tx_rempty: %x", dev->int_st.tx_rempty);
   ESP_LOGI(TAG, "int_st.rx_hung: %x", dev->int_st.rx_hung);
   ESP_LOGI(TAG, "int_st.tx_hung: %x", dev->int_st.tx_hung);
   ESP_LOGI(TAG, "int_st.in_done: %x", dev->int_st.in_done);
   ESP_LOGI(TAG, "int_st.in_suc_eof: %x", dev->int_st.in_suc_eof);
   ESP_LOGI(TAG, "int_st.in_err_eof: %x", dev->int_st.in_err_eof);
   ESP_LOGI(TAG, "int_st.out_done: %x", dev->int_st.out_done);
   ESP_LOGI(TAG, "int_st.out_eof: %x", dev->int_st.out_eof);
   ESP_LOGI(TAG, "int_st.in_dscr_err: %x", dev->int_st.in_dscr_err);
   ESP_LOGI(TAG, "int_st.out_dscr_err: %x", dev->int_st.out_dscr_err);
   ESP_LOGI(TAG, "int_st.in_dscr_empty: %x", dev->int_st.in_dscr_empty);
   ESP_LOGI(TAG, "int_st.out_total_eof: %x", dev->int_st.out_total_eof);
   ESP_LOGI(TAG, "int_ena.rx_take_data: %x", dev->int_ena.rx_take_data);
   ESP_LOGI(TAG, "int_ena.tx_put_data: %x", dev->int_ena.tx_put_data);
   ESP_LOGI(TAG, "int_ena.rx_wfull: %x", dev->int_ena.rx_wfull);
   ESP_LOGI(TAG, "int_ena.rx_rempty: %x", dev->int_ena.rx_rempty);
   ESP_LOGI(TAG, "int_ena.tx_wfull: %x", dev->int_ena.tx_wfull);
   ESP_LOGI(TAG, "int_ena.tx_rempty: %x", dev->int_ena.tx_rempty);
   ESP_LOGI(TAG, "int_ena.rx_hung: %x", dev->int_ena.rx_hung);
   ESP_LOGI(TAG, "int_ena.tx_hung: %x", dev->int_ena.tx_hung);
   ESP_LOGI(TAG, "int_ena.in_done: %x", dev->int_ena.in_done);
   ESP_LOGI(TAG, "int_ena.in_suc_eof: %x", dev->int_ena.in_suc_eof);
   ESP_LOGI(TAG, "int_ena.in_err_eof: %x", dev->int_ena.in_err_eof);
   ESP_LOGI(TAG, "int_ena.out_done: %x", dev->int_ena.out_done);
   ESP_LOGI(TAG, "int_ena.out_eof: %x", dev->int_ena.out_eof);
   ESP_LOGI(TAG, "int_ena.in_dscr_err: %x", dev->int_ena.in_dscr_err);
   ESP_LOGI(TAG, "int_ena.out_dscr_err: %x", dev->int_ena.out_dscr_err);
   ESP_LOGI(TAG, "int_ena.in_dscr_empty: %x", dev->int_ena.in_dscr_empty);
   ESP_LOGI(TAG, "int_ena.out_total_eof: %x", dev->int_ena.out_total_eof);
   ESP_LOGI(TAG, "int_clr.take_data: %x", dev->int_clr.take_data);
   ESP_LOGI(TAG, "int_clr.put_data: %x", dev->int_clr.put_data);
   ESP_LOGI(TAG, "int_clr.rx_wfull: %x", dev->int_clr.rx_wfull);
   ESP_LOGI(TAG, "int_clr.rx_rempty: %x", dev->int_clr.rx_rempty);
   ESP_LOGI(TAG, "int_clr.tx_wfull: %x", dev->int_clr.tx_wfull);
   ESP_LOGI(TAG, "int_clr.tx_rempty: %x", dev->int_clr.tx_rempty);
   ESP_LOGI(TAG, "int_clr.rx_hung: %x", dev->int_clr.rx_hung);
   ESP_LOGI(TAG, "int_clr.tx_hung: %x", dev->int_clr.tx_hung);
   ESP_LOGI(TAG, "int_clr.in_done: %x", dev->int_clr.in_done);
   ESP_LOGI(TAG, "int_clr.in_suc_eof: %x", dev->int_clr.in_suc_eof);
   ESP_LOGI(TAG, "int_clr.in_err_eof: %x", dev->int_clr.in_err_eof);
   ESP_LOGI(TAG, "int_clr.out_done: %x", dev->int_clr.out_done);
   ESP_LOGI(TAG, "int_clr.out_eof: %x", dev->int_clr.out_eof);
   ESP_LOGI(TAG, "int_clr.in_dscr_err: %x", dev->int_clr.in_dscr_err);
   ESP_LOGI(TAG, "int_clr.out_dscr_err: %x", dev->int_clr.out_dscr_err);
   ESP_LOGI(TAG, "int_clr.in_dscr_empty: %x", dev->int_clr.in_dscr_empty);
   ESP_LOGI(TAG, "int_clr.out_total_eof: %x", dev->int_clr.out_total_eof);
   ESP_LOGI(TAG, "timing.tx_bck_in_delay: %x", dev->timing.tx_bck_in_delay);
   ESP_LOGI(TAG, "timing.tx_ws_in_delay: %x", dev->timing.tx_ws_in_delay);
   ESP_LOGI(TAG, "timing.rx_bck_in_delay: %x", dev->timing.rx_bck_in_delay);
   ESP_LOGI(TAG, "timing.rx_ws_in_delay: %x", dev->timing.rx_ws_in_delay);
   ESP_LOGI(TAG, "timing.rx_sd_in_delay: %x", dev->timing.rx_sd_in_delay);
   ESP_LOGI(TAG, "timing.tx_bck_out_delay: %x", dev->timing.tx_bck_out_delay);
   ESP_LOGI(TAG, "timing.tx_ws_out_delay: %x", dev->timing.tx_ws_out_delay);
   ESP_LOGI(TAG, "timing.tx_sd_out_delay: %x", dev->timing.tx_sd_out_delay);
   ESP_LOGI(TAG, "timing.rx_ws_out_delay: %x", dev->timing.rx_ws_out_delay);
   ESP_LOGI(TAG, "timing.rx_bck_out_delay: %x", dev->timing.rx_bck_out_delay);
   ESP_LOGI(TAG, "timing.tx_dsync_sw: %x", dev->timing.tx_dsync_sw);
   ESP_LOGI(TAG, "timing.rx_dsync_sw: %x", dev->timing.rx_dsync_sw);
   ESP_LOGI(TAG, "timing.data_enable_delay: %x", dev->timing.data_enable_delay);
   ESP_LOGI(TAG, "timing.tx_bck_in_inv: %x", dev->timing.tx_bck_in_inv);
   ESP_LOGI(TAG, "fifo_conf.rx_data_num: %x", dev->fifo_conf.rx_data_num);
   ESP_LOGI(TAG, "fifo_conf.tx_data_num: %x", dev->fifo_conf.tx_data_num);
   ESP_LOGI(TAG, "fifo_conf.dscr_en: %x", dev->fifo_conf.dscr_en);
   ESP_LOGI(TAG, "fifo_conf.tx_fifo_mod: %x", dev->fifo_conf.tx_fifo_mod);
   ESP_LOGI(TAG, "fifo_conf.rx_fifo_mod: %x", dev->fifo_conf.rx_fifo_mod);
   ESP_LOGI(TAG, "fifo_conf.tx_fifo_mod_force_en: %x", dev->fifo_conf.tx_fifo_mod_force_en);
   ESP_LOGI(TAG, "fifo_conf.rx_fifo_mod_force_en: %x", dev->fifo_conf.rx_fifo_mod_force_en);
   ESP_LOGI(TAG, "rx_eof_num: %x", dev->rx_eof_num);
   ESP_LOGI(TAG, "conf_single_data: %x", dev->conf_single_data);
   ESP_LOGI(TAG, "conf_chan.tx_chan_mod: %x", dev->conf_chan.tx_chan_mod);
   ESP_LOGI(TAG, "conf_chan.rx_chan_mod: %x", dev->conf_chan.rx_chan_mod);
   ESP_LOGI(TAG, "outlink.addr: %x", dev->out_link.addr);
   ESP_LOGI(TAG, "outlink.stop: %x", dev->out_link.stop);
   ESP_LOGI(TAG, "outlink.start: %x", dev->out_link.start);
   ESP_LOGI(TAG, "outlink.restart: %x", dev->out_link.restart);
   ESP_LOGI(TAG, "outlink.park: %x", dev->out_link.park);
   ESP_LOGI(TAG, "inlink.addr: %x", dev->in_link.addr);
   ESP_LOGI(TAG, "inlink.stop: %x", dev->in_link.stop);
   ESP_LOGI(TAG, "inlink.start: %x", dev->in_link.start);
   ESP_LOGI(TAG, "inlink.restart: %x", dev->in_link.restart);
   ESP_LOGI(TAG, "inlink.park: %x", dev->in_link.park);
   ESP_LOGI(TAG, "out_eof_des_addr: %x", dev->out_eof_des_addr);
   ESP_LOGI(TAG, "in_eof_des_addr: %x", dev->in_eof_des_addr);
   ESP_LOGI(TAG, "out_eof_bfr_des_addr: %x", dev->out_eof_bfr_des_addr);
   ESP_LOGI(TAG, "ahb.test.mode: %x", dev->ahb_test.mode);
   ESP_LOGI(TAG, "ahb.test.addr: %x", dev->ahb_test.addr);
   ESP_LOGI(TAG, "in_link_dscr: %x", dev->in_link_dscr);
   ESP_LOGI(TAG, "in_link_dscr_bf0: %x", dev->in_link_dscr_bf0);
   ESP_LOGI(TAG, "in_link_dscr_bf1: %x", dev->in_link_dscr_bf1);
   ESP_LOGI(TAG, "out_link_dscr: %x", dev->out_link_dscr);
   ESP_LOGI(TAG, "out_link_dscr_bf0: %x", dev->out_link_dscr_bf0);
   ESP_LOGI(TAG, "out_link_dscr_bf1: %x", dev->out_link_dscr_bf1);
   ESP_LOGI(TAG, "lc_conf.in_rst: %x", dev->lc_conf.in_rst);
   ESP_LOGI(TAG, "lc_conf.out_rst: %x", dev->lc_conf.out_rst);
   ESP_LOGI(TAG, "lc_conf.ahbm_fifo_rst: %x", dev->lc_conf.ahbm_fifo_rst);
   ESP_LOGI(TAG, "lc_conf.ahbm_rst: %x", dev->lc_conf.ahbm_rst);
   ESP_LOGI(TAG, "lc_conf.out_loop_test: %x", dev->lc_conf.out_loop_test);
   ESP_LOGI(TAG, "lc_conf.in_loop_test: %x", dev->lc_conf.in_loop_test);
   ESP_LOGI(TAG, "lc_conf.out_auto_wrback: %x", dev->lc_conf.out_auto_wrback);
   ESP_LOGI(TAG, "lc_conf.out_no_restart_clr: %x", dev->lc_conf.out_no_restart_clr);
   ESP_LOGI(TAG, "lc_conf.out_eof_mode: %x", dev->lc_conf.out_eof_mode);
   ESP_LOGI(TAG, "lc_conf.outdscr_burst_en: %x", dev->lc_conf.outdscr_burst_en);
   ESP_LOGI(TAG, "lc_conf.indscr_burst_en: %x", dev->lc_conf.indscr_burst_en);
   ESP_LOGI(TAG, "lc_conf.out_data_burst_en: %x", dev->lc_conf.out_data_burst_en);
   ESP_LOGI(TAG, "lc_conf.check_owner: %x", dev->lc_conf.check_owner);
   ESP_LOGI(TAG, "lc_conf.mem_trans_en: %x", dev->lc_conf.mem_trans_en);
   ESP_LOGI(TAG, "lc_state0: %x", dev->lc_state0);
   ESP_LOGI(TAG, "lc_state1: %x", dev->lc_state1);
   ESP_LOGI(TAG, "out_fifo_push.wdata: %x", dev->out_fifo_push.wdata);
   ESP_LOGI(TAG, "out_fifo_push.push: %x", dev->out_fifo_push.push);
   ESP_LOGI(TAG, "in_fifo_pop.rdata: %x", dev->in_fifo_pop.rdata);
   ESP_LOGI(TAG, "in_fifo_pop.pop: %x", dev->in_fifo_pop.pop);
   ESP_LOGI(TAG, "lc_hung.fifo_timeout: %x", dev->lc_hung_conf.fifo_timeout);
   ESP_LOGI(TAG, "lc_hung.fifo_timeout_shift: %x", dev->lc_hung_conf.fifo_timeout_shift);
   ESP_LOGI(TAG, "lc_hung.fifo_timeout_ena: %x", dev->lc_hung_conf.fifo_timeout_ena);
   ESP_LOGI(TAG, "cvsd_conf0.y_max: %x", dev->cvsd_conf0.y_max);
   ESP_LOGI(TAG, "cvsd_conf0.y_min: %x", dev->cvsd_conf0.y_min);
   ESP_LOGI(TAG, "cvsd_conf1.sigma_max: %x", dev->cvsd_conf1.sigma_max);
   ESP_LOGI(TAG, "cvsd_conf1.sigma_min: %x", dev->cvsd_conf1.sigma_min);
   ESP_LOGI(TAG, "cvsd_conf2.cvsd_k: %x", dev->cvsd_conf2.cvsd_k);
   ESP_LOGI(TAG, "cvsd_conf2.cvsd_j: %x", dev->cvsd_conf2.cvsd_j);
   ESP_LOGI(TAG, "cvsd_conf2.cvsd_beta: %x", dev->cvsd_conf2.cvsd_beta);
   ESP_LOGI(TAG, "cvsd_conf2.cvsd_h: %x", dev->cvsd_conf2.cvsd_h);
   ESP_LOGI(TAG, "plc_conf0.good_pack_max: %x", dev->plc_conf0.good_pack_max);
   ESP_LOGI(TAG, "plc_conf0.n_err_seg: %x", dev->plc_conf0.n_err_seg);
   ESP_LOGI(TAG, "plc_conf0.shift_rate: %x", dev->plc_conf0.shift_rate);
   ESP_LOGI(TAG, "plc_conf0.max_slide_sample: %x", dev->plc_conf0.max_slide_sample);
   ESP_LOGI(TAG, "plc_conf0.pack_len_8k: %x", dev->plc_conf0.pack_len_8k);
   ESP_LOGI(TAG, "plc_conf0.n_min_err: %x", dev->plc_conf0.n_min_err);
   ESP_LOGI(TAG, "plc_conf1.bad_cef_atten_para: %x", dev->plc_conf1.bad_cef_atten_para);
   ESP_LOGI(TAG, "plc_conf1.bad_cef_atten_para_shift: %x", dev->plc_conf1.bad_cef_atten_para_shift);
   ESP_LOGI(TAG, "plc_conf1.bad_ola_win2_para_shift: %x", dev->plc_conf1.bad_ola_win2_para_shift);
   ESP_LOGI(TAG, "plc_conf1.bad_ola_win2_para: %x", dev->plc_conf1.bad_ola_win2_para);
   ESP_LOGI(TAG, "plc_conf1.slide_win_len: %x", dev->plc_conf1.slide_win_len);
   ESP_LOGI(TAG, "plc_conf2.cvsd_seg_mod: %x", dev->plc_conf2.cvsd_seg_mod);
   ESP_LOGI(TAG, "plc_conf2.min_period: %x", dev->plc_conf2.min_period);
   ESP_LOGI(TAG, "esco_conf0.en: %x", dev->esco_conf0.en);
   ESP_LOGI(TAG, "esco_conf0.chan_mod: %x", dev->esco_conf0.chan_mod);
   ESP_LOGI(TAG, "esco_conf0.cvsd_dec_pack_err: %x", dev->esco_conf0.cvsd_dec_pack_err);
   ESP_LOGI(TAG, "esco_conf0.cvsd_pack_len_8k: %x", dev->esco_conf0.cvsd_pack_len_8k);
   ESP_LOGI(TAG, "esco_conf0.cvsd_inf_en: %x", dev->esco_conf0.cvsd_inf_en);
   ESP_LOGI(TAG, "esco_conf0.cvsd_dec_start: %x", dev->esco_conf0.cvsd_dec_start);
   ESP_LOGI(TAG, "esco_conf0.cvsd_dec_reset: %x", dev->esco_conf0.cvsd_dec_reset);
   ESP_LOGI(TAG, "esco_conf0.plc_en: %x", dev->esco_conf0.plc_en);
   ESP_LOGI(TAG, "esco_conf0.plc2dma_en: %x", dev->esco_conf0.plc2dma_en);
   ESP_LOGI(TAG, "sco_conf0.with_en: %x", dev->sco_conf0.with_en);
   ESP_LOGI(TAG, "sco_conf0.no_en: %x", dev->sco_conf0.no_en);
   ESP_LOGI(TAG, "sco_conf0.cvsd_enc_start: %x", dev->sco_conf0.cvsd_enc_start);
   ESP_LOGI(TAG, "sco_conf0.cvsd_enc_reset: %x", dev->sco_conf0.cvsd_enc_reset);
   ESP_LOGI(TAG, "conf1.tx_pcm_conf: %x", dev->conf1.tx_pcm_conf);
   ESP_LOGI(TAG, "conf1.tx_pcm_bypass: %x", dev->conf1.tx_pcm_bypass);
   ESP_LOGI(TAG, "conf1.rx_pcm_conf: %x", dev->conf1.rx_pcm_conf);
   ESP_LOGI(TAG, "conf1.rx_pcm_bypass: %x", dev->conf1.rx_pcm_bypass);
   ESP_LOGI(TAG, "conf1.tx_stop_en: %x", dev->conf1.tx_stop_en);
   ESP_LOGI(TAG, "conf1.tx_zeros_rm_en: %x", dev->conf1.tx_zeros_rm_en);
   ESP_LOGI(TAG, "pd_conf.fifo_force_pd: %x", dev->pd_conf.fifo_force_pd);
   ESP_LOGI(TAG, "pd_conf.fifo_force_pu: %x", dev->pd_conf.fifo_force_pu);
   ESP_LOGI(TAG, "pd_conf.plc_mem_force_pd: %x", dev->pd_conf.plc_mem_force_pd);
   ESP_LOGI(TAG, "pd_conf.plc_mem_force_pu: %x", dev->pd_conf.plc_mem_force_pu);
   ESP_LOGI(TAG, "conf2.camera_en: %x", dev->conf2.camera_en);
   ESP_LOGI(TAG, "conf2.lcd_tx_wrx2_en: %x", dev->conf2.lcd_tx_wrx2_en);
   ESP_LOGI(TAG, "conf2.lcd_tx_sdx2_en: %x", dev->conf2.lcd_tx_sdx2_en);
   ESP_LOGI(TAG, "conf2.data_enable_test_en: %x", dev->conf2.data_enable_test_en);
   ESP_LOGI(TAG, "conf2.data_enable: %x", dev->conf2.data_enable);
   ESP_LOGI(TAG, "conf2.lcd_en: %x", dev->conf2.lcd_en);
   ESP_LOGI(TAG, "conf2.ext_adc_start_en: %x", dev->conf2.ext_adc_start_en);
   ESP_LOGI(TAG, "conf2.inter_valid_en: %x", dev->conf2.inter_valid_en);
   ESP_LOGI(TAG, "clkm_conf.clkm_div_num: %x", dev->clkm_conf.clkm_div_num);
   ESP_LOGI(TAG, "clkm_conf.clkm_div_b: %x", dev->clkm_conf.clkm_div_b);
   ESP_LOGI(TAG, "clkm_conf.clkm_div_a: %x", dev->clkm_conf.clkm_div_a);
   ESP_LOGI(TAG, "clkm_conf.clk_en: %x", dev->clkm_conf.clk_en);
   ESP_LOGI(TAG, "clkm_conf.clka_en: %x", dev->clkm_conf.clka_en);
   ESP_LOGI(TAG, "sample_rate.tx_bck_div_num: %x", dev->sample_rate_conf.tx_bck_div_num);
   ESP_LOGI(TAG, "sample_rate.rx_bck_div_num: %x", dev->sample_rate_conf.rx_bck_div_num);
   ESP_LOGI(TAG, "sample_rate.tx_bits_mod: %x", dev->sample_rate_conf.tx_bits_mod);
   ESP_LOGI(TAG, "sample_rate.rx_bits_mod: %x", dev->sample_rate_conf.rx_bits_mod);
   ESP_LOGI(TAG, "pdm_conf.tx_pdm_en: %x", dev->pdm_conf.tx_pdm_en);
   ESP_LOGI(TAG, "pdm_conf.rx_pdm_en: %x", dev->pdm_conf.rx_pdm_en);
   ESP_LOGI(TAG, "pdm_conf.pcm2pdm_conv_en: %x", dev->pdm_conf.pcm2pdm_conv_en);
   ESP_LOGI(TAG, "pdm_conf.pdm2pcm_conv_en: %x", dev->pdm_conf.pdm2pcm_conv_en);
   ESP_LOGI(TAG, "pdm_conf.tx_sinc_osr2: %x", dev->pdm_conf.tx_sinc_osr2);
   ESP_LOGI(TAG, "pdm_conf.tx_prescale: %x", dev->pdm_conf.tx_prescale);
   ESP_LOGI(TAG, "pdm_conf.tx_hp_in_shift: %x", dev->pdm_conf.tx_hp_in_shift);
   ESP_LOGI(TAG, "pdm_conf.tx_lp_in_shift: %x", dev->pdm_conf.tx_lp_in_shift);
   ESP_LOGI(TAG, "pdm_conf.tx_sinc_in_shift: %x", dev->pdm_conf.tx_sinc_in_shift);
   ESP_LOGI(TAG, "pdm_conf.tx_sigmadelta_in_shift: %x", dev->pdm_conf.tx_sigmadelta_in_shift);
   ESP_LOGI(TAG, "pdm_conf.rx_sinc_dsr_16_en: %x", dev->pdm_conf.rx_sinc_dsr_16_en);
   ESP_LOGI(TAG, "pdm_conf.txhp_bypass: %x", dev->pdm_conf.txhp_bypass);
   ESP_LOGI(TAG, "pdm_freq_conf.tx_pdm_fs: %x", dev->pdm_freq_conf.tx_pdm_fs);
   ESP_LOGI(TAG, "pdm_freq_conf.tx_pdm_fp: %x", dev->pdm_freq_conf.tx_pdm_fp);
   ESP_LOGI(TAG, "state.tx_idle: %x", dev->state.tx_idle);
   ESP_LOGI(TAG, "state.tx_fifo_reset_back: %x", dev->state.tx_fifo_reset_back);
   ESP_LOGI(TAG, "state.rx_fifo_reset_back: %x", dev->state.rx_fifo_reset_back);
   ESP_LOGI(TAG, "date: %x", dev->date);
}

void i2s_print_reg(i2s_dev_t *dev) {
   ESP_LOGI(TAG, "\nOperational Values in I2S\n" \
            "\n-------------------------\n" \
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
            dev->reserved_0,
            dev->reserved_4,
            dev->conf.val,
            dev->int_raw.val,
            dev->int_st.val,
            dev->int_ena.val,
            dev->int_clr.val,
            dev->timing.val,
            dev->fifo_conf.val,
            dev->rx_eof_num,
            dev->conf_single_data,
            dev->conf_chan.val,
            dev->out_link.val,
            dev->in_link.val,
            dev->out_eof_des_addr,
            dev->in_eof_des_addr,
            dev->out_eof_bfr_des_addr,
            dev->ahb_test.val,
            dev->in_link_dscr,
            dev->in_link_dscr_bf0,
            dev->in_link_dscr_bf1,
            dev->out_link_dscr,
            dev->out_link_dscr_bf0,
            dev->out_link_dscr_bf1,
            dev->lc_conf.val,
            dev->out_fifo_push.val,
            dev->in_fifo_pop.val,
            dev->lc_state0,
            dev->lc_state1,
            dev->lc_hung_conf.val,
            dev->reserved_78,
            dev->reserved_7c,
            dev->cvsd_conf0.val,
            dev->cvsd_conf1.val,
            dev->cvsd_conf2.val,
            dev->plc_conf0.val,
            dev->plc_conf1.val,
            dev->plc_conf2.val,
            dev->esco_conf0.val,
            dev->sco_conf0.val,
            dev->conf1.val,
            dev->pd_conf.val,
            dev->conf2.val,
            dev->clkm_conf.val,
            dev->sample_rate_conf.val,
            dev->pdm_conf.val,
            dev->pdm_freq_conf.val,
            dev->state.val,
            dev->reserved_c0,
            dev->reserved_c4,
            dev->reserved_c8,
            dev->reserved_cc,
            dev->reserved_d0,
            dev->reserved_d4,
            dev->reserved_d8,
            dev->reserved_dc,
            dev->reserved_e0,
            dev->reserved_e4,
            dev->reserved_e8,
            dev->reserved_ec,
            dev->reserved_f0,
            dev->reserved_f4,
            dev->reserved_f8,
            dev->date);

}

#undef TAG