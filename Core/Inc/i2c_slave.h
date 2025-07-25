/*
 * i2c_slave.h
 *
 *  Created on: Jul 25, 2025
 *      Author: cp3
 */

#ifndef INC_I2C_SLAVE_H_
#define INC_I2C_SLAVE_H_

#include "main.h"

#define NUM_REGISTROS 10

extern uint8_t registros[NUM_REGISTROS];

/**
 * Callbacks para gestionar eventos I2C en modo esclavo:
 * - I2C_Slave_AddrCallback: Se llama al detectar dirección I2C (START + dirección)
 * - I2C_Slave_RxCpltCallback: Se llama al completar la recepción de datos (registro + dato)
 * - I2C_Slave_TxCpltCallback: Se llama al completar la transmisión de datos
 *
 * Estos callbacks deben enlazarse con las funciones HAL correspondientes en el main.c:
 *   HAL_I2C_AddrCallback
 *   HAL_I2C_SlaveRxCpltCallback
 *   HAL_I2C_SlaveTxCpltCallback
 */

void I2C_Slave_Init(I2C_HandleTypeDef *hi2c);
void I2C_Slave_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
void I2C_Slave_RxCpltCallback(I2C_HandleTypeDef *hi2c);
void I2C_Slave_TxCpltCallback(I2C_HandleTypeDef *hi2c);

#endif /* INC_I2C_SLAVE_H_ */
