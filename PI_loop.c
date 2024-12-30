#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

// Constants
#ifndef ADC_RESULT_BITS
#define ADC_RESULT_BITS (1 << 12)  // 12-bit ADC
#endif
#define WRAP_VAL 832
#define SAMPLE_TIME_MS 10
#define TRANSLATE_FL_WRAP(value)    (uint16_t)(value*(WRAP_VAL)/3.3f)

const float V_gain = 1.0;//, V_offset = 0.0;
const float V_ki = 0.00482, V_kp = 0.112;
const float setpoint = 1.5;

typedef struct {
    float setpoint;    // Desired value
    float Verr;        // Error
    float Vp;          // Proportional term
    float Vi;          // Integral term
    float output;      // Control output
} PIState;

typedef struct {
    float Kp;          // Proportional gain
    float Ki;          // Integral gain
    float Vi_max;      // Maximum integral term
    float Vi_min;      // Minimum integral term
} PIParams;

void pi_init(PIState *state, float setpoint) {
    state->setpoint = setpoint;
    state->Verr = 0.0f;
    state->Vp = 0.0f;
    state->Vi = 0.0f;
    state->output = 0.0f;
}

float pi_update(PIState *state, PIParams *params, float measured_value) {
    // Apply voltage gain to measured value
    float scaled_value = measured_value * V_gain; //! if you decide to use V_Offset this is where you add it
    
    // Calculate error using scaled value
    state->Verr = state->setpoint - scaled_value;

    // Proportional term
    state->Vp = params->Kp * state->Verr;

    // Integral term
    state->Vi += params->Ki * state->Verr;
    if (state->Vi > params->Vi_max) state->Vi = params->Vi_max;
    if (state->Vi < params->Vi_min) state->Vi = params->Vi_min;

    // Control output
    state->output = state->Vp + state->Vi;
    return state->output;
}

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
    // const float Vi_max = 3.3; // Maximum value for Vi to prevent windup
    // const float Vi_min = -3.3; // Minimum value for Vi to prevent windup
    const float Vi_max = reference_voltage/1.5; // Maximum value for Vi to prevent windup
    const float Vi_min = -reference_voltage/1.5; // Minimum value for Vi to prevent windup

    PIState state;
    PIParams params = {
        .Kp = V_kp,
        .Ki = V_ki,
        .Vi_max = Vi_max,
        .Vi_min = Vi_min
    };
    
    pi_init(&state, setpoint);

    while (true) {
        uint16_t raw = adc_read();
        float voltage = raw * conversion_factor;
        
        float control_output = pi_update(&state, &params, voltage);

        uint16_t duty_cycle = TRANSLATE_FL_WRAP(control_output);
        if (duty_cycle > WRAP_VAL) duty_cycle = WRAP_VAL;
        if (duty_cycle < 0) duty_cycle = 0;
        pwm_set_gpio_level(0, duty_cycle);

        printf("Voltage: %.2f V, Error: %.2f, Vp: %.2f, Vi: %.2f, Output: %.2f, Duty: %u\n",
               voltage, state.Verr, state.Vp, state.Vi, state.output, duty_cycle);

        sleep_ms(SAMPLE_TIME_MS);
    }
}
