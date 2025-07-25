/*
 * adc.h
 *
 *  Created on: Jul 25, 2025
 *      Author: cp3
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "main.h"

// Número de canales ADC que se leen mediante DMA
#define NUM_ADC 2

// Buffer externo donde se almacenan las conversiones ADC (12 bits sin procesar)
extern uint16_t adc_in[NUM_ADC];

// Tabla externa de registros donde se guardan los valores procesados (8 bits)
extern uint8_t registros[];

/**
 * @brief Inicializa y arranca el ADC con DMA para capturar NUM_ADC canales.
 */
void ADC_Init(void);

/**
 * @brief Callback que se llama cuando la conversión ADC ha finalizado.
 *        Actualiza los registros con los valores ADC escalados a 8 bits.
 *
 * @param hadc Puntero al manejador ADC que llamó al callback.
 */
void ADC_ConvCompleteCallback(ADC_HandleTypeDef* hadc);

#endif /* INC_ADC_H_ */
