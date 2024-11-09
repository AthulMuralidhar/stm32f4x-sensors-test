/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: Oct 31, 2024
 *      Author: athul-muralidhar
 */

#include "stm32f407xx_spi_driver.h"

// local helper functions
static void spi_ovr_interrupt_handler(SPI_Handle_t *pSPIHandle);
static void spi_txe_interrupt_handler(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handler(SPI_Handle_t *pSPIHandle);


// helper functions

/**
 * @fn                  SPI_GetFlagStatus
 *
 * @brief               Checks the status of a specified SPI flag
 *
 * @param[in] pSPIx     Pointer to an SPI_RegDef_t structure that contains
 *                      the configuration information for the specified SPI port
 * @param[in] FlagName  Flag to check status for
 *
 * @return              FLAG_SET if the flag is set, FLAG_RESET otherwise
 *
 * @note                This function should be used to check SPI status flags before performing operations
 *
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {
	if (pSPIx->SR & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

// peripheral clock
/**
 * @fn                  SPI_PClockControl
 *
 * @brief               Controls the peripheral clock for an SPI port
 *
 * @param[in] pSPIx     Pointer to an SPI_RegDef_t structure that contains
 *                      the configuration information for the specified SPI port
 * @param[in] EnOrDi    ENABLE or DISABLE macros to enable or disable the clock
 *
 * @return              none
 *
 * @note                Peripheral clock should be enabled before using the SPI port
 *
 */
void SPI_PClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		}
//		else if (pSPIx == SPI4) {
//			SPI4_PCLK_EN();
//		}
	}

	if (EnOrDi == DISABLE) {
		// TODO: do the disable bits
	}
}

// initialization and de-initialization
/**
 * @fn                  SPI_Init
 *
 * @brief               Initializes the SPI peripheral according to the specified parameters
 *                      in the SPI_Handle_t structure
 *
 * @param[in] pSPIHandle Pointer to an SPI_Handle_t structure that contains
 *                       the configuration information for the specified SPI peripheral
 *
 * @return              none
 *
 * @note                This function must be called before using the SPI peripheral
 */
void SPI_Init(SPI_Handle_t *pSPIHandle) {
	// peripheral clock enable
	SPI_PClockControl(pSPIHandle->pSPIx, ENABLE);

	uint32_t tempRegister = 0;
	// configure the SPI CR1 register

	// 1.configure the device mode
	tempRegister |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	// 2.configure the bus
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		// full duplex
		//  BIDI mode should be cleared
		tempRegister &= ~(1 << SPI_CR1_BIDIMODE);

	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		// half duplex
		// BIDI mode should be enabled
		tempRegister |= (1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig
			== SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		// simplex receive only
		// BIDI mode should be cleared
		tempRegister &= ~(1 << SPI_CR1_BIDIMODE);
		// RXONLY bit must be set
		tempRegister |= (1 << SPI_CR1_RXONLY);
	}

	// 3.  configure the clock speed
	tempRegister |= pSPIHandle->SPIConfig.SPI_SClkSpeed << SPI_CR1_BR;

	// 4.  configure the clock polarity
	tempRegister |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// 5.  configure the clock phase
	tempRegister |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	// 6.  configure the data frame
	tempRegister |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// 7.  configure the slave management
	tempRegister |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempRegister;

}

/**
 * @fn                  SPI_DeInit
 *
 * @brief               De-initializes the SPI peripheral registers to their default reset values
 *
 * @param[in] pSPIx     Pointer to an SPI_RegDef_t structure that contains
 *                      the configuration information for the specified SPI peripheral
 *
 * @return              none
 *
 * @note                This function should be called when the SPI peripheral is no longer needed
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if (pSPIx == SPI1) {
		GPIOA_REG_RESET();
	} else if (pSPIx == SPI2) {
		GPIOB_REG_RESET();
	} else if (pSPIx == SPI3) {
		GPIOC_REG_RESET();
	}
//	else if (pSPIx == SPI4) {
//		GPIOD_REG_RESET();
//	}
}

// data read / write
/**
 * @fn                  SPI_SendData
 *
 * @brief               Sends data through the SPI peripheral
 *
 * @param[in] pSPIx     Pointer to an SPI_RegDef_t structure that contains
 *                      the configuration information for the specified SPI peripheral
 * @param[in] pTxBuffer Pointer to the data buffer to be transmitted
 * @param[in] Len       Length of data to be transmitted in bytes
 *
 * @return              none
 *
 * @note                This function is blocking and will wait until all data is transmitted
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {
	while (Len > 0) {
		// 1.wait till TXE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET){};

		// 2.check the DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			// 16 bit data
			// 1. load the data
			pSPIx->DR = (uint32_t)*pTxBuffer;
			Len--;
			Len--;
			(uint16_t*) pTxBuffer++;
		} else {
			// 8 bit data
			// 1. load the data
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/**
 * @fn                  SPI_ReceiveData
 *
 * @brief               Receives data through the SPI peripheral
 *
 * @param[in] pSPIx     Pointer to an SPI_RegDef_t structure that contains
 *                      the configuration information for the specified SPI peripheral
 * @param[out] pRxBuffer Pointer to the buffer where received data will be stored
 * @param[in] Len       Length of data to be received in bytes
 *
 * @return              none
 *
 * @note                This function is blocking and will wait until all data is received
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {
	while (Len > 0) {
		// 1.wait till RXNE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET)
			;

		// 2.check the DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			// 16 bit data
			// 1. load the data
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*) pRxBuffer++;
		} else {
			// 8 bit data
			// 1. load the data
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

// interrupt handling and configuration
/**
 * @fn                  SPI_IRQPriorityConfig
 *
 * @brief               Configures the priority of the SPI interrupt
 *
 * @param[in] IRQNumber IRQ number of the SPI interrupt
 * @param[in] IRQPriority Priority to be set for the SPI interrupt
 *
 * @return              none
 *
 * @note                This function should be called before enabling the SPI interrupt
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	// Calculate which IPR register to use
	uint64_t iprx = IRQNumber / 4;

	// Calculate which section of the IPR register to use
	uint64_t iprx_section = IRQNumber % 4;

	// Calculate the shift amount based on the section and number of priority bits implemented
	uint64_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	// Set the priority in the appropriate IPR register
	*( NVIC_PR_BASE_ADDR + iprx)  |= (IRQPriority << shift_amount);
}

/**
 * @fn                  SPI_IRQInterruptConfig
 *
 * @brief               Enables or disables the SPI interrupt
 *
 * @param[in] IRQNumber IRQ number of the SPI interrupt
 * @param[in] EnOrDi    ENABLE or DISABLE macros to enable or disable the interrupt
 *
 * @return              none
 *
 * @note                This function should be called after configuring the SPI interrupt priority
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi){
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
 * @fn                  SPI_IRQHandler
 *
 * @brief               Handles the SPI interrupt
 *
 * @param[in] pSPIHandle Pointer to an SPI_Handle_t structure that contains
 *                       the configuration information for the specified SPI peripheral
 *
 * @return              none
 *
 * @note                This function should be called in the SPI ISR
 */
void SPI_IRQHandler(SPI_Handle_t *pSPIHandle){
	uint8_t isTXESet, isTXEIESet;

	// check for TXE
	isTXESet = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	isTXEIESet = pSPIHandle->pSPIx->SR & (1 << SPI_CR2_TXEIE);

	if (isTXESet && isTXEIESet) {
		// handle TXE interrupt
		spi_txe_interrupt_handler(pSPIHandle);
	}

	uint8_t isRXESet, isRXNEIESet;

	// check for RXNE
	isRXESet = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	isRXNEIESet = pSPIHandle->pSPIx->SR & (1 << SPI_CR2_RXNEIE);

	if (isRXESet && isRXNEIESet) {
		// handle RXNE interrupt
		spi_rxne_interrupt_handler(pSPIHandle);
	}

	uint8_t isOVRSet, isERRIESet;

	// check for RXNE
	isOVRSet = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	isERRIESet = pSPIHandle->pSPIx->SR & (1 << SPI_CR2_ERRIE);

	if (isOVRSet && isERRIESet) {
		// handle RXNE interrupt
		spi_ovr_interrupt_handler(pSPIHandle);
	}



}

/**
 * @fn                  SPI_PeripheralControl
 *
 * @brief               Enables or disables the SPI peripheral
 *
 * @param[in] pSPIx     Pointer to an SPI_RegDef_t structure that contains
 *                      the configuration information for the specified SPI port
 * @param[in] EnOrDi    ENABLE or DISABLE macros to enable or disable the SPI peripheral
 *
 * @return              none
 *
 * @note                This function should be called after SPI initialization and before any data transfer
 *
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}

	if (EnOrDi == DISABLE) {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


/**
 * @fn                  SPI_SSIConfig
 *
 * @brief               Configures the Slave Select Internal (SSI) bit in SPI control register
 *
 * @param[in] pSPIx     Pointer to an SPI_RegDef_t structure that contains
 *                      the configuration information for the specified SPI port
 * @param[in] EnOrDi    ENABLE or DISABLE macros to set or reset the SSI bit
 *
 * @return              none
 *
 * @note                This function is typically used in software slave management mode
 *
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}

	if (EnOrDi == DISABLE) {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}



// data read / write interrupt based
/**
 * @fn                  SPI_SendDataIT
 *
 * @brief               Initiates an interrupt-based SPI data transmission
 *
 * @param[in] pSPIHandle Pointer to SPI_Handle_t structure that contains
 *                       the configuration information and state for the specified SPI module
 * @param[in] pTxBuffer  Pointer to the buffer containing data to be transmitted
 * @param[in] Len        Length of the data to be transmitted
 *
 * @return              Current state of the SPI transmission
 *                      (SPI_BUSY_IN_TX if transmission initiated, otherwise previous state)
 *
 * @note                This function sets up the transmission parameters and enables the TXE interrupt.
 *                      The actual data transmission occurs in the ISR, which must be implemented separately.
 *
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len) {

	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_TX) {
		// 1. save the Tx buffer address and Len information in a global variable
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		// 2. Mark the SPI in busy state so that no other code can take over the SPI peripheral until Tx is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3. enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		// 4. handle he data transmission with the ISR code
		// TODO
	}
	return state;
}


/**
 * @fn                  SPI_ReceiveDataIT
 *
 * @brief               Initiates an interrupt-based SPI data reception
 *
 * @param[in] pSPIHandle Pointer to SPI_Handle_t structure that contains
 *                       the configuration information and state for the specified SPI module
 * @param[out] pRxBuffer Pointer to the buffer where received data will be stored
 * @param[in] Len        Length of the data to be received
 *
 * @return              Current state of the SPI reception
 *                      (SPI_BUSY_IN_RX if reception initiated, otherwise previous state)
 *
 * @note                This function sets up the reception parameters and enables the RXNE interrupt.
 *                      The actual data reception occurs in the ISR, which must be implemented separately.
 *
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len) {
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_RX) {
		// 1. save the Rx buffer address and Len information in a global variable
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		// 2. Mark the SPI in busy state so that no other code can take over the SPI peripheral until Rx is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3. enable the RXNEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		// 4. handle he data transmission with the ISR code
		// TODO
	}
	return state;
}

/**
 * @fn                  SPI_CloseTransmisson
 *
 * @brief               Closes an ongoing SPI transmission
 *
 * @param[in] pSPIHandle Pointer to SPI_Handle_t structure that contains
 *                       the configuration information for the specified SPI module
 *
 * @return              None
 *
 * @note                This function disables the TXE interrupt, resets transmission
 *                      parameters, and sets the SPI state to ready
 */
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

/**
 * @fn                  spi_ovr_interrupt_handler
 *
 * @brief               Handles the SPI overrun interrupt
 *
 * @param[in] pSPIHandle Pointer to SPI_Handle_t structure that contains
 *                       the configuration information for the specified SPI module
 *
 * @return              None
 *
 * @note                This function clears the overrun flag and calls the application
 *                      callback function with the overrun error event
 */
static void spi_ovr_interrupt_handler(SPI_Handle_t *pSPIHandle) {
	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;

	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}

/**
 * @fn                  spi_txe_interrupt_handler
 *
 * @brief               Handles the SPI transmit buffer empty interrupt
 *
 * @param[in] pSPIHandle Pointer to SPI_Handle_t structure that contains
 *                       the configuration information for the specified SPI module
 *
 * @return              None
 *
 * @note                This function loads data into the SPI data register for transmission,
 *                      handles both 8-bit and 16-bit data formats, and manages the transmission completion
 */
static void spi_txe_interrupt_handler(SPI_Handle_t *pSPIHandle) {

	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		// 16 bit data
		// 1. load the data
		pSPIHandle->pSPIx->DR = (uint16_t)*pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*) pSPIHandle->pTxBuffer++;
	} else {
		// 8 bit data
		// 1. load the data
		pSPIHandle->pSPIx->DR =*pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if (!pSPIHandle->TxLen) {
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}

}


/**
 * @fn                  spi_rxne_interrupt_handler
 *
 * @brief               Handles the SPI receive buffer not empty interrupt
 *
 * @param[in] pSPIHandle Pointer to SPI_Handle_t structure that contains
 *                       the configuration information for the specified SPI module
 *
 * @return              None
 *
 * @note                This function reads data from the SPI data register,
 *                      handles both 8-bit and 16-bit data formats, and manages the reception completion
 */
static void  spi_rxne_interrupt_handler(SPI_Handle_t *pSPIHandle)
{
	//do rxing as per the dff
	if(pSPIHandle->pSPIx->CR1 & ( 1 << 11))
	{
		//16 bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;

	}else
	{
		//8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(! pSPIHandle->RxLen)
	{
		//reception is complete
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}

}

/**
 * @fn                  SPI_ClearOVRFlag
 *
 * @brief               Clears the SPI overrun flag
 *
 * @param[in] pSPIx     Pointer to SPI_RegDef_t structure that contains
 *                      the configuration information for the specified SPI module
 *
 * @return              None
 *
 * @note                This function clears the overrun flag by reading the DR and SR registers
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}

/**
 * @fn                  SPI_CloseReception
 *
 * @brief               Closes an ongoing SPI reception
 *
 * @param[in] pSPIHandle Pointer to SPI_Handle_t structure that contains
 *                       the configuration information for the specified SPI module
 *
 * @return              None
 *
 * @note                This function disables the RXNE interrupt, resets reception
 *                      parameters, and sets the SPI state to ready
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}


/**
 * @fn                  SPI_ApplicationEventCallback
 *
 * @brief               Weak implementation of the SPI application event callback
 *
 * @param[in] pSPIHandle Pointer to SPI_Handle_t structure that contains
 *                       the configuration information for the specified SPI module
 * @param[in] AppEv      Application event that occurred
 *
 * @return              None
 *
 * @note                This is a weak implementation that can be overridden by the application
 */
__attribute((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv) {
	// weak implementation
}
