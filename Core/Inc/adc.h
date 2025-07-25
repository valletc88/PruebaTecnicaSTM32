/*
 * adc.h
 *
 *  Created on: Jul 25, 2025
 *      Author: cp3
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "main.h"

#define NUM_ADC 2

extern uint16_t adc_in[NUM_ADC];
extern uint8_t registros[];  // Referencia externa a la tabla de registros

void ADC_Init(void);
void ADC_ConvCompleteCallback(ADC_HandleTypeDef* hadc);

#endif /* INC_ADC_H_ */
