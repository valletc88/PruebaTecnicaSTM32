#include "i2c_slave.h"

// Índice del registro seleccionado para lectura/escritura
static uint8_t i2c_reg_index = 0;

// Dirección de transferencia: 0 = maestro escribe, 1 = maestro lee
static uint8_t slave_direction = 0;

// Buffer para recibir datos I2C (registro + dato)
static uint8_t i2c_rx_buf[2];

// Dato a transmitir al maestro
static uint8_t i2c_tx_data = 0;

// Puntero al handler I2C usado (para identificar la instancia)
static I2C_HandleTypeDef* hi2c_ptr = NULL;

/**
 * @brief Inicializa la recepción I2C en modo esclavo con interrupciones.
 * Guarda el handler para comparaciones posteriores.
 */
void I2C_Slave_Init(I2C_HandleTypeDef *hi2c)
{
    hi2c_ptr = hi2c;
    // Iniciar recepción interrupt-driven de 2 bytes (registro + dato)
    HAL_I2C_Slave_Receive_IT(hi2c_ptr, i2c_rx_buf, 2);
}

/**
 * @brief Callback al detectar dirección I2C (START + dirección).
 * Determina si el maestro va a leer o escribir y prepara la siguiente acción.
 */
void I2C_Slave_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    if (hi2c->Instance == hi2c_ptr->Instance) {
        // Reiniciar índice de registro al comenzar nueva transferencia
        i2c_reg_index = 0;

        // Guardar dirección: 0 = maestro escribe, 1 = maestro lee
        slave_direction = (TransferDirection == I2C_DIRECTION_TRANSMIT) ? 0 : 1;

        if (slave_direction == 0) {
            // Maestro va a escribir: preparamos recepción de registro + dato
            HAL_I2C_Slave_Receive_IT(hi2c, i2c_rx_buf, 2);
        } else {
            // Maestro va a leer: enviamos primer byte (registro actual)
            i2c_tx_data = registros[i2c_reg_index];
            HAL_I2C_Slave_Transmit_IT(hi2c, &i2c_tx_data, 1);
        }
    }
}

/**
 * @brief Callback al completar la recepción de 2 bytes (registro + dato).
 * Actualiza el registro correspondiente y vuelve a preparar recepción.
 */
void I2C_Slave_RxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == hi2c_ptr->Instance) {
        // El primer byte es el índice de registro
        i2c_reg_index = i2c_rx_buf[0];

        // El segundo byte es el dato que escribimos en ese registro
        registros[i2c_reg_index] = i2c_rx_buf[1];

        // Re-armar la recepción para seguir escuchando datos
        HAL_I2C_Slave_Receive_IT(hi2c, i2c_rx_buf, 2);
    }
}

/**
 * @brief Callback al completar la transmisión de un byte.
 * Prepara el siguiente byte a transmitir (mismo registro).
 */
void I2C_Slave_TxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == hi2c_ptr->Instance) {
        // Verificar que el índice no exceda el número de registros
        if (i2c_reg_index < NUM_REGISTROS) {
            // Enviar nuevamente el dato actual del registro
            i2c_tx_data = registros[i2c_reg_index];
            HAL_I2C_Slave_Transmit_IT(hi2c, &i2c_tx_data, 1);
        }
    }
}
