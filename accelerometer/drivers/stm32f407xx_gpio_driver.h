/*
 * stm32f4xx_gpio_driver.h
 *
 *  Created on: Oct 25, 2024
 *      Author: athul-muralidhar
 */

#ifndef STM32F407XX_GPIO_DRIVER_H_
#define STM32F407XX_GPIO_DRIVER_H_

#include <stm32f407xx.h>

// @GPIO_PIN_MODES
// GPIO pin possible modes
#define GPIO_MODE_INPUT     0  // Configures the GPIO pin as an input for reading external signals
#define GPIO_MODE_OUTPUT    1  // Configures the GPIO pin as an output for driving external devices
#define GPIO_MODE_ALT_FN    2  // Sets the GPIO pin for alternate function (e.g., UART, SPI, I2C)
#define GPIO_MODE_ANALOG    3  // Configures the GPIO pin for analog functionality (ADC or DAC)
#define GPIO_MODE_IT_FT     4  // Sets the GPIO pin to generate an interrupt on falling edge
#define GPIO_MODE_IT_RT     5  // Sets the GPIO pin to generate an interrupt on rising edge
#define GPIO_MODE_IT_RFT    6  // Configures the GPIO pin for interrupt on both rising and falling edges

// GPIO pin possible output types
#define GPIO_OP_TYPE_PP    0  // Configures the GPIO pin as Push-Pull output type
#define GPIO_OP_TYPE_OD    1  // Configures the GPIO pin as Open-Drain output type

// GPIO pin possible output speeds
#define GPIO_SPEED_LOW    0  // Configures the GPIO pin for low output speed
#define GPIO_SPEED_MEDIUM 1  // Configures the GPIO pin for medium output speed
#define GPIO_SPEED_FAST   2  // Configures the GPIO pin for fast output speed
#define GPIO_SPEED_HIGH   3  // Configures the GPIO pin for high output speed

// GPIO pin pull up pull down configuration macros
#define GPIO_NO_PUPD    0  // Configures the GPIO pin with no pull-up/pull-down
#define GPIO_PIN_PU     1  // Configures the GPIO pin with pull-up resistor
#define GPIO_PIN_PD     2  // Configures the GPIO pin with pull-down resistor

// @GPIO_PIN_NUM
// GPIOpin numbers
#define	GPIO_PIN_NO_0		0
#define	GPIO_PIN_NO_1		1
#define	GPIO_PIN_NO_2		2
#define	GPIO_PIN_NO_3		3
#define	GPIO_PIN_NO_4		4
#define	GPIO_PIN_NO_5		5
#define	GPIO_PIN_NO_6		6
#define	GPIO_PIN_NO_7		7
#define	GPIO_PIN_NO_8       8
#define	GPIO_PIN_NO_9 		9
#define	GPIO_PIN_NO_10		10
#define	GPIO_PIN_NO_11		11
#define	GPIO_PIN_NO_12		12
#define	GPIO_PIN_NO_13		13
#define	GPIO_PIN_NO_14		14
#define	GPIO_PIN_NO_15		15


// General pin configuration structure
typedef struct {
	uint8_t GPIO_PinNumber;          // GPIO pin number from @GPIO_PIN_NUM

	uint8_t GPIO_PinMode;            // GPIO pin mode
									 // possible values from @GPIO_PIN_MODES

	uint8_t GPIO_PinSpeed;           // GPIO pin output speed
									 // possible values: GPIO_OP_SPEED_LOW, GPIO_OP_SPEED_MEDIUM,
									 //                  GPIO_OP_SPEED_FAST, GPIO_OP_SPEED_HIGH

	uint8_t GPIO_PinPuPdControl;     // GPIO pin pull-up/pull-down configuration
									 // possible values: GPIO_NO_PUPD, GPIO_PIN_PU, GPIO_PIN_PD

	uint8_t GPIO_PinOpType;          // GPIO pin output type
									 // possible values: GPIO_OP_TYPE_PP, GPIO_OP_TYPE_OD

	uint8_t GPIO_PinAltFunMode;      // GPIO pin alternate function mode

} GPIO_PinConfig_t;

// a handler structure for GPIO pin
typedef struct {
	GPIO_RegDef_t *pGPIOx; // this pointer holds the base address of the GPIO peripheral
	GPIO_PinConfig_t GPIO_PinConfig; // this holds the GPIO pin configuration settings
} GPIO_Handle_t;

// ===================================  APIs supported by this driver =========================================

// peripheral clock
void GPIO_PClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

// initialization and de-initialization
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// data read / write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WrtiteToOutoutPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_WrtiteToOutoutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// interrupt handling and configuration
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQHandler(uint8_t PinNumber);

#endif /* STM32F407XX_GPIO_DRIVER_H_ */
