/*
 * gpio.c
 *
 *  Created on: Jul 25, 2025
 *      Author: cp3
 */


#include "gpio.h"
#include "registros.h"  // Para acceder al array registros[]

/**
 * @brief Configura el modo de los pines PD8 a PD15.
 *
 * Usa el registro registros[2] para definir qué pines son salida (bit en 1)
 * o entrada (bit en 0).
 *
 * Para cada pin:
 * - Si el bit correspondiente de registros[2] es 1, se configura como salida push-pull.
 * - Si es 0, se configura como entrada sin pull-up ni pull-down.
 *
 * Se reinicializa cada pin para aplicar la nueva configuración.
 */
void GPIO_ActualizarModo_PD(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    for (int i = 0; i < 8; i++)
    {
        // Selecciona el pin PD8+i
        GPIO_InitStruct.Pin = (1 << (8 + i));
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Pull = GPIO_NOPULL;

        // Configura modo según registros[2]
        if (registros[2] & (1 << i))
        {
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Salida push-pull
        }
        else
        {
            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;      // Entrada
        }

        // Desinicializa y luego inicializa para aplicar cambios
        HAL_GPIO_DeInit(GPIOD, GPIO_InitStruct.Pin);
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    }
}

/**
 * @brief Escribe el valor de registros[3] en los pines configurados como salida.
 *
 * Solo se modifican los pines PD8 a PD15 que están configurados como salida
 * según registros[2].
 *
 * Cada bit de registros[3] indica si el pin correspondiente debe ir a nivel alto (1)
 * o bajo (0).
 */
void GPIO_Escribir_PD(void)
{
    for (int i = 0; i < 8; i++)
    {
        // Solo actúa si el pin está configurado como salida
        if (registros[2] & (1 << i))
        {
            HAL_GPIO_WritePin(GPIOD, (1 << (8 + i)),
                (registros[3] & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }
    }
}

/**
 * @brief Lee el estado actual de los pines PD8 a PD15.
 *
 * El estado de cada pin se almacena en el bit correspondiente de registros[4].
 * - Bit en 1 si el pin está a nivel alto.
 * - Bit en 0 si el pin está a nivel bajo.
 */
void GPIO_Leer_PD(void)
{
    registros[4] = 0;

    for (int i = 0; i < 8; i++)
    {
        GPIO_PinState state = HAL_GPIO_ReadPin(GPIOD, (1 << (8 + i)));
        if (state == GPIO_PIN_SET)
        {
            registros[4] |= (1 << i);
        }
    }
}
