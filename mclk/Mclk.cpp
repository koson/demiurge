
#include <soc/mcpwm_reg.h>
#include <soc/periph_defs.h>
#include <soc/gpio_sig_map.h>
#include <hal/gpio_types.h>
#include <driver/gpio.h>
#include <driver/periph_ctrl.h>
#include "Mclk.h"

static void initialize(gpio_num_t pin_out) {
   // We use MCPWM0 TIMER 0 Operator 0 for CS generation to DAC.

   PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[pin_out], PIN_FUNC_GPIO);
   gpio_set_direction(pin_out, GPIO_MODE_INPUT_OUTPUT);
   gpio_matrix_out(pin_out, PWM0_OUT0A_IDX, false, false);  // PWM0 operator 0 output A
   periph_module_enable(PERIPH_PWM0_MODULE);

   WRITE_PERI_REG(MCPWM_TIMER_SYNCI_CFG_REG(0), (1 << MCPWM_TIMER0_SYNCISEL_S));
   WRITE_PERI_REG(MCPWM_OPERATOR_TIMERSEL_REG(0), (0 << MCPWM_OPERATOR0_TIMERSEL_S));
   WRITE_PERI_REG(MCPWM_GEN0_TSTMP_A_REG(0), 5);

   // UTEZ= set PWM0A low, UTEA=set PWM0A high
   WRITE_PERI_REG(MCPWM_GEN0_A_REG(0), (1 << MCPWM_GEN0_A_UTEZ_S) | (2 << MCPWM_GEN0_A_UTEA_S));

   // Prescale=0, so timer is 80MHz, 5 clocks per cycle = 16MHz
   WRITE_PERI_REG(MCPWM_TIMER0_CFG0_REG(0), 0 << MCPWM_TIMER0_PRESCALE_S | 9 << MCPWM_TIMER0_PERIOD_S);

   // Continuously running, increase mode.
   WRITE_PERI_REG(MCPWM_TIMER0_CFG1_REG(0), (1 << MCPWM_TIMER0_MOD_S) | (2 << MCPWM_TIMER0_START_S));
}

Mclk::Mclk() {
   initialize(GPIO_NUM_0);
}