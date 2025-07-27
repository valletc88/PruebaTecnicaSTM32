/*
 * iso1H816G.h
 *
 *  Created on: Jul 26, 2025
 *      Author: cp3
 */

#ifndef INC_ISO1H816G_H_
#define INC_ISO1H816G_H_

#include "main.h"  // Incluye definiciones básicas del proyecto (HAL, tipos, etc.)

/**
 * @brief Envía al ISO1H816G los valores de salida definidos en registros[3],
 * aplicando la polaridad configurada en registros[6].
 *
 * Utiliza SPI1 para la transmisión y PA4 como pin de Chip Select (CS).
 */
void ISO_ActualizarSalidas(void);

#endif /* INC_ISO1H816G_H_ */
