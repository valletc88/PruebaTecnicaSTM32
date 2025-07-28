# Proyecto STM32F4 – Control de GPIO, ADC e ISO1H816G vía SPI

Este proyecto implementa un sistema embebido sobre un microcontrolador "STM32F407", que permite controlar salidas digitales y leer entradas analógicas y digitales, mediante una interfaz I2C_slave. Además, utiliza el componente "ISO1H816G" conectado por "SPI" para controlar salidas.

# Descripción general

El firmware permite:

- Leer dos canales analógicos por ADC (PA0 y PA1), con conversión DMA.
- Controlar los pines PD8 a PD15 como GPIO entrada o salida.
- Enviar los estados deseados de los pines al "ISO1H816G" a través de SPI.
- Configurar la polaridad de las salidas del ISO1H816G.
- Leer y escribir todos los registros a través del bus I2C (modo esclavo).

El control del sistema se realiza escribiendo/leyendo en un array de 10 registros desde un maestro I2C.

# Estructura de registros


| Señal     | Pin STM32  | Descripción                     |
|-----------|------------|---------------------------------|
| ADC_IN0   | PA0        | Canal ADC 0                     |
| ADC_IN1   | PA1        | Canal ADC 1                     |
| SPI1_CS   | PA4        | Chip Select manual para ISO     |
| SPI1_SCK  | PA5        | Reloj SPI                       |
| SPI1_MOSI | PA7        | Datos hacia el ISO              |
| I2C1_SDA  | PB7        | Comunicación I2C esclava        |
| I2C1_SCL  | PB6        | Comunicación I2C esclava        |
| GPIO      | PD8–PD15   | Entradas/salidas digitales      |

# Pines utilizados

| Índice | Función                                        |
|--------|-----------------------------------------------|
| 0      | Valor ADC canal 0 (8 bits)                    |
| 1      | Valor ADC canal 1 (8 bits)                    |
| 2      | Configuración de pines PD8–PD15 (1 = salida)  |
| 3      | Valores a escribir en PD8–PD15 y en el ISO    |
| 4      | Estado leído de los pines PD8–PD15            |
| 5–9    | No utilizados                                  |
| 6      | Máscara de polaridad para el ISO1H816G        |

# Lógica principal

- El maestro I2C escribe en `registros[2]` para configurar los pines PD8–PD15 como entrada o salida.
- Si se configuran como salida, los valores en `registros[3]` se escriben tanto en los pines físicos como en el ISO1H816G por SPI.
- La polaridad aplicada al ISO se configura mediante `registros[6]` y se aplica con XOR.
- Se leen continuamente los valores de ADC y se almacenan en `registros[0]` y `registros[1]`.
- Se actualiza `registros[4]` con el estado de los pines PD8–PD15 si están en modo entrada.

# Consideraciones

- La tensión de entrada al ADC debe estar limitada a 0–3.3V.
- Se recomienda escalar la entrada de 5–20V mediante un divisor resistivo adecuado.
- El ISO1H816G permite manejar salidas de mayor potencia/protección que el microcontrolador.
- La comunicación I2C debe realizarse desde un maestro externo




