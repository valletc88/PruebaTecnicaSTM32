/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "registros.h"   // Variables globales de registros para comunicación
#include "adc.h"         // Módulo ADC con DMA
#include "i2c_slave.h"   // Módulo I2C modo esclavo
#include "gpio.h" 		// Mñodulo GPIO

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;       // Handle ADC1
DMA_HandleTypeDef hdma_adc1;   // Handle DMA para ADC1
I2C_HandleTypeDef hi2c1;       // Handle I2C1
UART_HandleTypeDef huart1;     // Handle USART1 para comunicación serial

/* USER CODE BEGIN PV */
uint8_t registros[NUM_REGISTROS] = {0};  // Buffer de registros para datos ADC y I2C
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

/**
  * @brief  Main program entry point.
  * @retval int
  */
int main(void)
{
  /* Inicialización HAL y sistema */
  HAL_Init();

  /* Configura el reloj del sistema */
  SystemClock_Config();

  /* Inicializa periféricos configurados */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  /* Inicialización de módulos específicos */
  ADC_Init();              // Inicia ADC con DMA para lectura continua
  I2C_Slave_Init(&hi2c1);  // Inicia I2C en modo esclavo con interrupciones
  uint8_t modo_anterior = registros[2];
  /* Bucle principal: sin lógica activa, todo por interrupciones */
  while (1)
  {
      // Lee el estado actual de los pines PD8 a PD15
      GPIO_Leer_PD();

      // Si el modo configurado de los pines (guardado en registros[2]) ha cambiado,
      // actualiza la configuración (entrada/salida) de cada pin.
      if (modo_anterior != registros[2])
      {
          GPIO_ActualizarModo_PD();
          modo_anterior = registros[2];  // Guarda el modo actual para próximas comparaciones
      }

      // Escribe el valor almacenado en registros[3] en los pines configurados como salida
      GPIO_Escribir_PD();

      // Retardo para evitar que el loop corra demasiado rápido y saturar el bus o procesador
      HAL_Delay(100);
  }

}

/**
  * @brief Configura el reloj del sistema para usar HSE sin PLL
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  // Configura oscilador externo HSE y desactiva PLL
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  // Configura relojes de CPU y buses para usar HSE directamente
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief Inicializa ADC1 para lectura continua con DMA, 2 canales (PA0 y PA1)
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;               // Escanea varios canales
  hadc1.Init.ContinuousConvMode = ENABLE;         // Conversión continua
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;                  // 2 canales
  hadc1.Init.DMAContinuousRequests = ENABLE;      // Usa DMA para transferir datos
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }

  // Configura canal 0 (PA0)
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  // Configura canal 1 (PA1)
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief Inicializa I2C1 en modo esclavo con dirección 0x20 (32 decimal)
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;               // 100 kHz estándar
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0x50 << 1;                  // Dirección esclavo 0x50
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief Inicializa USART1 para comunicación serial a 115200 baudios
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief Inicializa el controlador DMA y habilita la interrupción
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();

  // Configura prioridad e interrupción para DMA2_Stream0 (usado por ADC1)
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/**
  * @brief Inicializa los puertos GPIO usados
  */
static void MX_GPIO_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  /* USER CODE BEGIN MX_GPIO_Init_1 */

	  /* USER CODE END MX_GPIO_Init_1 */

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOH_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /* Configura GPIO pins : PD8 PD9 PD10 PD11
	                           PD12 PD13 PD14 PD15 */
	  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
	                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  /* USER CODE BEGIN MX_GPIO_Init_2 */
}

/* Callbacks que redirigen a funciones específicas de ADC e I2C */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    ADC_ConvCompleteCallback(hadc);  // Manejo lectura ADC completa
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    I2C_Slave_AddrCallback(hi2c, TransferDirection, AddrMatchCode);  // Dirección I2C detectada
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    I2C_Slave_RxCpltCallback(hi2c);  // Recepción I2C completa
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    I2C_Slave_TxCpltCallback(hi2c);  // Transmisión I2C completa
}

/**
  * @brief Función de manejo de errores: se detiene el sistema en bucle infinito.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief Reporta el archivo y línea donde ocurrió un fallo de aserción.
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  // Se puede implementar un mensaje por UART o similar aquí para debugging.
}
#endif /* USE_FULL_ASSERT */
