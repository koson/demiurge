//
// Created by niclas on 2020-05-16.
//

#ifndef VCO_TLV320AIC3104_H
#define VCO_TLV320AIC3104_H

#include <esp32/rom/lldesc.h>
#include <hal/i2s_types.h>
#include <soc/i2s_reg.h>
#include <driver/i2s_print.h>
#include <hal/i2s_hal.h>

static const int FIFO_TX_SIZE = 32;
static const int FIFO_RX_SIZE = 32;

// PLL constants per definition in section 12.3 of https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
static const int RATE_DIV_A = 4;
static const int RATE_DIV_B = 1;
static const int RATE_DIV_N = 3;

class Tlv320aic3104 {

public:
   Tlv320aic3104(int channel, gpio_num_t bclk_pin, gpio_num_t ws_pin, gpio_num_t din_pin, gpio_num_t dout_pin );
   virtual ~Tlv320aic3104() = default;
   void setOutput(int16_t output1, int16_t output2);
   void readInput(int16_t *inputLeft, int16_t *inputRight);
private:

   int channel;
   i2s_hal_context_t i2s_context = {};
   lldesc_t *out = nullptr;
   lldesc_t *in = nullptr;

   void setup_i2s(gpio_num_t bclk_pin, gpio_num_t ws_pin, gpio_num_t din, gpio_num_t dout);

   int idle_counter = 0;
   int print_counter = 0;

   void reset_external_chip() const;

   void configure_external_chip();

   void setup_setup_ll_desc();
};


#endif //VCO_TLV320AIC3104_H
