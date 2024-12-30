#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

const float V_gain = 1.0, V_offset = 0.0;
// const float V_ki = 0.00482, V_kp = 0.1; // Initial values for tuning
const float V_ki = 0.0049, V_kp = 0.11; // Initial values for tuning
const float setpoint = 1.5; // Desired voltage

// #define WRAP_VAL 832
#define WRAP_VAL 832
#define TRANSLATE_FL_WRAP(value)    (uint16_t)(value*(WRAP_VAL)/3.3f)

int main()
{
    stdio_init_all();
    adc_init();
    gpio_init(26);
    gpio_set_function(26, GPIO_FUNC_SIO);
    adc_gpio_init(26);
    adc_select_input(0);

    // Initialize PWM on GPIO 0
    gpio_set_function(0, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(0);
    pwm_config pwm_cfg = pwm_get_default_config();
    pwm_cfg.top = WRAP_VAL;
    pwm_init(slice_num, &pwm_cfg, true);
    pwm_set_enabled(slice_num, true);
    pwm_set_gpio_level(0, WRAP_VAL/2);

    const float reference_voltage = 3.3f;
    const float conversion_factor = reference_voltage / ADC_RESULT_BITS;
    const float Vi_max = 3.3; // Maximum value for Vi to prevent windup
    const float Vi_min = -3.3; // Minimum value for Vi to prevent windup
    uint16_t duty_cycle;
    float Verr, Vfb, V, Vp, Vi = 0;

    while (true) {
        uint16_t raw = adc_read();
        float voltage = raw * conversion_factor;
        
        // Calculate error
        Verr = setpoint - voltage;

        // Proportional term
        Vp = V_kp * Verr;

        // Integral term -- Sums over time (hence, integral)
        Vi += V_ki * Verr;
        if (Vi > Vi_max) Vi = Vi_max; // Limit Vi to prevent windup
        if (Vi < Vi_min) Vi = Vi_min; // Limit Vi to prevent windup

        // Control output
        V = Vp + Vi;

        // Set PWM duty cycle based on control output
        uint16_t duty_cycle = TRANSLATE_FL_WRAP(V);
        if (duty_cycle > WRAP_VAL) duty_cycle = WRAP_VAL; // Limit duty cycle to 100%
        if (duty_cycle < 0) duty_cycle = 0; // Limit duty cycle to 0%
        pwm_set_gpio_level(0, duty_cycle);

        // Debug prints
        printf("Voltage: %.2f V, Error: %.2f, Vp: %.2f, Vi: %.2f, Control Output: %.2f, Duty Cycle: %u\n",
               voltage, Verr, Vp, Vi, V, duty_cycle);

        sleep_ms(10);
    }
}
