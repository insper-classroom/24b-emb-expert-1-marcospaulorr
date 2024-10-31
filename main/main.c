/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "pico/time.h"

#define AUDIO_PIN 28     // PWM pin for audio output
#define MIC_PIN 26       // ADC pin for microphone input
#define SAMPLE_RATE 11025 // Hz, matches common audio rates
#define BUFFER_SIZE 1024  // Audio buffer size

volatile uint16_t audio_buffer[BUFFER_SIZE];
volatile int buffer_pos = 0;

// PWM Interrupt Handler
void pwm_interrupt_handler() {
    pwm_clear_irq(pwm_gpio_to_slice_num(AUDIO_PIN));
    pwm_set_gpio_level(AUDIO_PIN, audio_buffer[buffer_pos++]);
    if (buffer_pos >= BUFFER_SIZE) {
        buffer_pos = 0;
    }
}

// ADC Sampling task
bool adc_sample_callback(repeating_timer_t *t) {
    uint16_t adc_value = adc_read(); // Read microphone ADC value
    audio_buffer[buffer_pos++] = adc_value; // Store in audio buffer
    if (buffer_pos >= BUFFER_SIZE) {
        buffer_pos = 0;
    }
    return true;
}

int main() {
    stdio_init_all();

    // Setup ADC for microphone input
    adc_init();
    adc_gpio_init(MIC_PIN);
    adc_select_input(0); // Select ADC channel for MIC_PIN

    // Setup PWM for audio output
    gpio_set_function(AUDIO_PIN, GPIO_FUNC_PWM);
    int audio_pin_slice = pwm_gpio_to_slice_num(AUDIO_PIN);

    // PWM configuration for audio
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 8.0f); // Set PWM clock divider
    pwm_config_set_wrap(&config, 250);    // Set wrap value
    pwm_init(audio_pin_slice, &config, true);

    // Enable PWM interrupt
    pwm_clear_irq(audio_pin_slice);
    pwm_set_irq_enabled(audio_pin_slice, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_interrupt_handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // Set up timer for ADC sampling
    repeating_timer_t timer;
    add_repeating_timer_us(-1000000 / SAMPLE_RATE, adc_sample_callback, NULL, &timer);

    // Main loop does nothing, interrupt-driven
    while (true) {
        __wfe(); // Wait for interrupt
    }
}
