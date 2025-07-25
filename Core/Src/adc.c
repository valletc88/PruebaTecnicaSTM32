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
    // Verifica que la interrupción corresponde al ADC1 (por si hay más ADCs)
    if (hadc->Instance == ADC1)
    {
        // Almacena la lectura del canal 0 en registros[0], escalando de 12 bits a 8 bits (shift 4 bits)
        registros[0] = (uint8_t)(adc_in[0] >> 4);

        // Almacena la lectura del canal 1 en registros[1], también escalada a 8 bits
        registros[1] = (uint8_t)(adc_in[1] >> 4);
    }
}
