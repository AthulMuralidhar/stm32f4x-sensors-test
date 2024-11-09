/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: Oct 25, 2024
 *      Author: athul-muralidhar
 */

#include <stm32f407xx_gpio_driver.h>

// peripheral clock
/**
 * @fn					GPIO_PClockControl
 *
 * @brief				Controls the peripheral clock for a GPIO port
 *
 * @param[in] pGPIOx	Pointer to a GPIO_RegDef_t structure that contains
 * 						the configuration information for the specified GPIO port
 * @param[in] EnOrDi	ENABLE or DISABLE macros to enable or disable the clock
 *
 * @return				none
 *
 * @note				Peripheral clock should be enabled before using the GPIO port
 */
void GPIO_PClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		}
	}

	// FIXME: finish the disable part
//	if (EnOrDi == DISABLE) {
//		if (pGPIOx == GPIOA) {
//			GPIOA_PCLK_DI();
//		} else if (pGPIOx == GPIOB) {
//			GPIOB_PCLK_DI();
//		} else if (pGPIOx == GPIOC) {
//			GPIOC_PCLK_DI();
//		} else if (pGPIOx == GPIOD) {
//			GPIOD_PCLK_DI();
//		} else if (pGPIOx == GPIOE) {
//			GPIOE_PCLK_DI();
//		} else if (pGPIOx == GPIOF) {
//			GPIOF_PCLK_DI();
//		} else if (pGPIOx == GPIOG) {
//			GPIOG_PCLK_DI();
//		} else if (pGPIOx == GPIOH) {
//			GPIOH_PCLK_DI();
//		} else if (pGPIOx == GPIOI) {
//			GPIOI_PCLK_DI();
//		}
//	}

}

// initialization and de-initialization
/**
 * @fn						GPIO_Init
 *
 * @brief					Initializes the GPIO port according to the specified parameters in the GPIO_Handle_t
 *
 * @param[in] pGPIOHandle	Pointer to a GPIO_Handle_t structure that contains
 * 							the configuration information for the specified GPIO pin
 *
 * @return					none
 *
 * @note					This function should be called before using the GPIO pin
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	uint32_t tempRegister;

	// Enable peripheral clock
	GPIO_PClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. configure the mode of the pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		// non interrupt mode
		// For example, if you're configuring pin 5, the mode bits would be shifted left by 10 positions (2 * 5),
		// placing them in bits 10 and 11 of the register.
		tempRegister = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
				<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		// clear
		pGPIOHandle->pGPIOx->MODER &= ~(0x3
				<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);   // 0x3 == 11
		// store
		pGPIOHandle->pGPIOx->MODER |= tempRegister;

	} else {
		// interrupt mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear RTSR

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear FTSR

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		// 2.configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t extiRegisterSet = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber
				/ 4; // we choose the index for the array with this, so SYSCFG.EXTICR[0] == EXTICR1
		uint8_t extiBitPosition = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber
				% 4;   // this tells us about the position within each register

		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG->EXTICR[extiRegisterSet] = portCode << (extiBitPosition * 4);

		// 3.enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	// reset temporary register
	tempRegister = 0;
	// 2. configure the speed
	tempRegister = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	// clear
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);   // 0x3 == 0b11
	// store
	pGPIOHandle->pGPIOx->OSPEEDR |= tempRegister;

	// reset temporary register
	tempRegister = 0;
	// 3. configure the pull up pull down
	tempRegister = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	// clear
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);   // 0x3 == 0b11
	// store
	pGPIOHandle->pGPIOx->PUPDR |= tempRegister;

	// reset temporary register
	tempRegister = 0;
	// 4. configure the output type
	tempRegister = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType
			<< (1 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // no 2 bits here only 16bits is used
	// clear
	pGPIOHandle->pGPIOx->OTYPER &= ~(1
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	// store
	pGPIOHandle->pGPIOx->OTYPER |= tempRegister;

	// reset temporary register
	tempRegister = 0;
	// 5. configure the alternate functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT_FN) {
		uint8_t lowOrHigh = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8; // if this is 0, then low, - this decides if it is AFR low or AFR high register
		uint8_t bitPosition = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8; // this decides where the pin registers fall within low or high register

		// clear
		pGPIOHandle->pGPIOx->AFR[lowOrHigh] &= ~(0xF << (4 * bitPosition)); // 0xF == 0b1111
		// store
		pGPIOHandle->pGPIOx->AFR[lowOrHigh] |=
				(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode
						<< (4 * bitPosition));

	}

}

/**
 * @fn					GPIO_DeInit
 *
 * @brief				De-initializes the GPIO port, resetting it to its default state
 *
 * @param[in] pGPIOx	Pointer to a GPIO_RegDef_t structure that contains
 * 						the configuration information for the specified GPIO port
 *
 * @return				none
 *
 * @note				This function resets all the GPIO port registers to their default values
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	} else if (pGPIOx == GPIOI) {
		GPIOI_REG_RESET();
	}
}

// data read / write
/**
 * @fn					GPIO_ReadFromInputPin
 *
 * @brief				Reads the current state of the specified GPIO input pin
 *
 * @param[in] pGPIOx	Pointer to a GPIO_RegDef_t structure that contains
 * 						the configuration information for the specified GPIO port
 * @param[in] PinNumber	The pin number (0-15) of the GPIO port to read from
 *
 * @return				The current state of the input pin (0 or 1)
 *
 * @note				Ensure that the specified pin is configured as an input before calling this function
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	uint8_t readValue;

	readValue = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001); // first shift the register to the 0th position, then do an AND and read the last bit
	return readValue;
}

/**
 * @fn					GPIO_ReadFromInputPort
 *
 * @brief				Reads the current state of all pins of the specified GPIO port
 *
 * @param[in] pGPIOx	Pointer to a GPIO_RegDef_t structure that contains
 * 						the configuration information for the specified GPIO port
 *
 * @return				The current state of all pins of the GPIO port (16-bit value)
 *
 * @note				The returned value represents the state of all 16 pins, where each bit
 * 						corresponds to a pin (bit 0 for pin 0, bit 1 for pin 1, etc.)
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t readValue;

	readValue = (uint16_t) (pGPIOx->IDR);
	return readValue;
}

/**
 * @fn					GPIO_WriteToOutputPort
 *
 * @brief				Writes a 16-bit value to the output of the specified GPIO port
 *
 * @param[in] pGPIOx	Pointer to a GPIO_RegDef_t structure that contains
 * 						the configuration information for the specified GPIO port
 * @param[in] value		16-bit value to be written to the GPIO port
 *
 * @return				none
 *
 * @note				Each bit in the value corresponds to a pin on the port
 * 						(bit 0 for pin 0, bit 1 for pin 1, etc.)
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
	// Function implementation goes here
	pGPIOx->ODR = value;
}

/**
 * @fn					GPIO_WriteToOutputPin
 *
 * @brief				Writes a value to the specified pin of the GPIO port
 *
 * @param[in] pGPIOx	Pointer to a GPIO_RegDef_t structure that contains
 * 						the configuration information for the specified GPIO port
 * @param[in] PinNumber	The pin number (0-15) of the GPIO port to write to
 * @param[in] value		Value to be written to the pin (0 or 1)
 *
 * @return				none
 *
 * @note				Ensure that the specified pin is configured as an output before calling this function
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t value) {

	if (value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		// write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}

/**
 * @fn					GPIO_ToggleOutputPin
 *
 * @brief				Toggles the state of the specified pin of the GPIO port
 *
 * @param[in] pGPIOx	Pointer to a GPIO_RegDef_t structure that contains
 * 						the configuration information for the specified GPIO port
 * @param[in] PinNumber	The pin number (0-15) of the GPIO port to toggle
 *
 * @return				none
 *
 * @note				This function changes the state of the pin from high to low or low to high
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	//	For example, if PinNumber is 5 and the current state of ODR is 0b11110000,
	//	after this operation, the new state would be 0b11110000 ^ 0b00100000 = 0b11010000.

	pGPIOx->ODR ^= (1 << PinNumber);
}

/**
 * @fn                  GPIO_IRQInterruptConfig
 *
 * @brief               Configures the interrupt for a GPIO pin in the NVIC (Nested Vectored Interrupt Controller)
 *
 * @param[in] IRQNumber The IRQ (Interrupt Request) number for the GPIO pin (0-95)
 * @param[in] EnOrDi    ENABLE or DISABLE macros to enable or disable the interrupt
 *
 * @return              none
 *
 * @note                This function enables or disables the specified GPIO interrupt in the NVIC.
 *                      It uses the appropriate NVIC_ISERx (Interrupt Set-Enable Register) to enable
 *                      or NVIC_ICERx (Interrupt Clear-Enable Register) to disable the interrupt.
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		if (IRQNumber <= 31) {
			// ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			// ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			// ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));
		}
	} else if (EnOrDi == DISABLE) {
		if (IRQNumber <= 31) {
			// ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			// ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			// ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));
		}
	}
}

/**
 * @fn                  GPIO_IRQPriorityConfig
 *
 * @brief               Configures the priority of a GPIO interrupt in the NVIC
 *
 * @param[in] IRQNumber The IRQ (Interrupt Request) number for the GPIO pin (0-95)
 * @param[in] IRQPriority The priority of the interrupt (0-255, with 0 being the highest priority)
 *
 * @return              none
 *
 * @note                This function sets the priority of the specified GPIO interrupt in the NVIC.
 *                      It calculates the appropriate Interrupt Priority Register (IPR) and applies
 *                      the priority value to the correct bit field within that register.
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	// Calculate which IPR register to use
	uint64_t iprx = IRQNumber / 4;

	// Calculate which section of the IPR register to use
	uint64_t iprx_section = (uint32_t) IRQNumber % 4;

	// Calculate the shift amount based on the section and number of priority bits implemented
	uint64_t shift_amount = (8 * iprx_section) + (8 -  (uint32_t)NVIC_PR_BASE_ADDR);

	// Set the priority in the appropriate IPR register
	*( NVIC_PR_BASE_ADDR + iprx)  |= (IRQPriority << shift_amount);
}

/**
 * @fn						GPIO_IRQHandler
 *
 * @brief					Handles the interrupt for a specific GPIO pin
 *
 * @param[in] PinNumber		The pin number (0-15) of the GPIO port that triggered the interrupt
 *
 * @return					none
 *
 * @note					This function should be called in the interrupt service routine (ISR)
 * 							for the GPIO pin. It typically clears the interrupt flag and performs
 * 							any necessary interrupt handling operations.
 */
void GPIO_IRQHandler(uint8_t PinNumber) {
// clear the EXTI PR register corresponding to the pin number
	if (EXTI->PR & (1 << PinNumber)) { // The result will be non-zero if the interrupt flag for the specified pin is set
		// clear
		EXTI->PR |= (1 << PinNumber);

	}
}

