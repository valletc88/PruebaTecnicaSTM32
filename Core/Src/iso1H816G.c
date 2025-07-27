/*
 * iso1H816G.c
 *
 *  Created on: Jul 26, 2025
 *      Author: cp3
 */

#include "iso1H816G.h"
#include "registros.h"  // Donde está declarado el array global 'registros'

extern SPI_HandleTypeDef hspi1;  // Handle para SPI1, definido externamente

/**
 * @brief Actualiza el estado de las salidas del ISO1H816G según los registros.
 *
 * registros[3]: valores deseados en los pines PD8–PD15.
 * registros[6]: configuración de polaridad (0 = directa, 1 = invertida por bit).
 *
 * Se realiza una XOR bit a bit para aplicar la inversión cuando sea necesario.
 * Luego se transmite el byte resultante por SPI al ISO1H816G.
 */
void ISO_ActualizarSalidas(void)
{
    uint8_t salida = registros[3];     // Niveles deseados en las salidas
    uint8_t polaridad = registros[6];  // Configuración de polaridad

    uint8_t datos_a_enviar = salida ^ polaridad; // Aplicación de polaridad

    // Activar CS (bajar PA4)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    // Enviar byte por SPI
    HAL_SPI_Transmit(&hspi1, &datos_a_enviar, 1, HAL_MAX_DELAY);

    // Desactivar CS (subir PA4)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}


