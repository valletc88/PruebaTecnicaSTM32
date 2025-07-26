/*
 * adc.c
 *
 *  Created on: Jul 25, 2025
 *      Author: cp3
 */

#include "adc.h"

// Array para almacenar las conversiones ADC recibidas por DMA
uint16_t adc_in[NUM_ADC];

// Referencia externa al manejador ADC definido en main.c o en otro módulo
extern ADC_HandleTypeDef hadc1;

// Referencia externa al array de registros donde se guardan valores procesados
extern uint8_t registros[];

/**
 * @brief Inicializa el ADC y lanza la conversión con DMA para NUM_ADC canales.
 *        Llamar esta función para comenzar la adquisición continua.
 */
void ADC_Init(void)
{
    // Inicia la conversión ADC en modo DMA, llenando el buffer adc_in con NUM_ADC muestras
    HAL_ADC_Start_DMA(&hadc1, adc_in, NUM_ADC);
}

/**
 * @brief Callback que se llama cuando la conversión ADC finaliza (completa el buffer).
 *        Actualiza los registros con los valores ADC convertidos a 8 bits.
 *
 * @param hadc Puntero al manejador ADC que generó el callback.
 */
void ADC_ConvCompleteCallback(ADC_HandleTypeDef* hadc)
{
    // Factor de escalado del divisor resistivo (20V → 3.3V)
    const float escala = 20.0f / 3.3f;

    // Convierte lectura ADC de 12 bits (0–4095) a tensión real de entrada
    float v_adc0 = (adc_in[0] * 3.3f) / 4095.0f;
    float v_in0 = v_adc0 * escala;

    float v_adc1 = (adc_in[1] * 3.3f) / 4095.0f;
    float v_in1 = v_adc1 * escala;

    // Guarda la tensión real (en V × 10) como entero de 8 bits // Ejemplo: 15.6V → 156
    registros[0] = (uint8_t)(v_in0 * 10.0f);
    registros[1] = (uint8_t)(v_in1 * 10.0f);
}
