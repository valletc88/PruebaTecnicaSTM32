/*
 * registros.h
 *
 *  Created on: Jul 25, 2025
 *      Author: cp3
 *
 * registros[0]: ADC canal 0 (8 bits).
 * registros[1]: ADC canal 1 (8 bits).
 * registros[2]: Configuración de pines PD8-PD15 (bit=1 salida, 0 entrada).
 * registros[3]: Valores para pines de salida PD8-PD15.
 * registros[4]: Estado leído de pines PD8-PD15.
 * registros[5]: No usado.
 * registros[6]: Polaridad de salidas del ISO1H816G (bit=1 invierte, 0 normal).
 * registros[7..9]: No usados.
 *
 */

#ifndef INC_REGISTROS_H_
#define INC_REGISTROS_H_

#include <stdint.h>

#define NUM_REGISTROS 10

extern uint8_t registros[NUM_REGISTROS];

#endif /* INC_REGISTROS_H_ */
