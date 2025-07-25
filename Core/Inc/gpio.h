/*
 * gpio.h
 *
 *  Created on: Jul 25, 2025
 *      Author: cp3
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "main.h"
extern uint8_t registros[];

// Actualiza el modo (entrada o salida) de los pines PD8 a PD15
void GPIO_ActualizarModo_PD(void);

// Escribe en los pines PD8 a PD15 según el registro correspondiente
void GPIO_Escribir_PD(void);

// Lee el estado de los pines PD8 a PD15 y lo guarda en un registro
void GPIO_Leer_PD(void);

#endif /* INC_GPIO_H_ */
