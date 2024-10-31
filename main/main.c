/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// Configurações do pino e PWM
#define MIC_ADC_PIN 26       // Pino ADC do microfone (GP26)
#define BUTTON_PIN 15        // Pino do botão para ativar o áudio
#define SPEAKER_PWM_PIN 16   // Pino PWM para o speaker (GP16)

#define ADC_MAX 4095         // Máximo valor do ADC (12 bits)
#define PWM_MAX 65535        // Máximo valor do PWM (16 bits)

void configure_adc() {
    adc_init();
    adc_gpio_init(MIC_ADC_PIN); // Inicializa o pino do microfone para ADC
    adc_select_input(0);        // Seleciona o canal ADC 0 (GP26)
}

void configure_pwm(uint slice_num) {
    pwm_set_wrap(slice_num, PWM_MAX); // Configura o PWM para 16 bits
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
    pwm_set_enabled(slice_num, true);
}

int main() {
    stdio_init_all();
    configure_adc();

    // Configura botão
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);

    // Configura PWM para o speaker
    gpio_set_function(SPEAKER_PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(SPEAKER_PWM_PIN);
    configure_pwm(slice_num);

    printf("Sistema de captura e reprodução de áudio iniciado.\n");

    while (true) {
        if (gpio_get(BUTTON_PIN) == 0) { // Verifica se o botão está pressionado
            uint16_t adc_value = adc_read(); // Lê o valor do microfone via ADC
            uint16_t pwm_value = (adc_value * PWM_MAX) / ADC_MAX; // Converte ADC para PWM
            pwm_set_chan_level(slice_num, PWM_CHAN_A, pwm_value); // Envia para o speaker
        } else {
            pwm_set_chan_level(slice_num, PWM_CHAN_A, 0); // Desativa o PWM se o botão não estiver pressionado
        }
        sleep_us(125); // Taxa de amostragem aproximada de 8 kHz (ajuste conforme necessário)
    }
}
