/*
 * adc.c
 *
 *  Created on: Jul 25, 2025
 *      Author: cp3
 */

#include "adc.h"

uint16_t adc_in[NUM_ADC];
extern ADC_HandleTypeDef hadc1;
extern uint8_t registros[];
/**
 * @brief Inicializa el ADC con DMA (puede ser llamado desde main.c)
 */
void ADC_Init(void)
{
    HAL_ADC_Start_DMA(&hadc1, adc_in, NUM_ADC);
}

/**
 * @brief Callback que actualiza registros con valores ADC (8 bits)
 */
void ADC_ConvCompleteCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        registros[0] = (uint8_t)(adc_in[0] >> 4);  // Canal 0 → 8 bits
        registros[1] = (uint8_t)(adc_in[1] >> 4);  // Canal 1 → 8 bits
    }
}
