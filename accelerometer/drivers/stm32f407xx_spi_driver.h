/*
 * stm32f4xx_spi_driver.h
 *
 *  Created on: Oct 31, 2024
 *      Author: athul-muralidhar
 */

#ifndef STM32F407XX_SPI_DRIVER_H_
#define STM32F407XX_SPI_DRIVER_H_

#include <stm32f407xx.h>



#define SPI_DEVICE_MODE_MASTER          1       // SPI device mode: Master
#define SPI_DEVICE_MODE_SLAVE           0       // SPI device mode: Slave

#define SPI_BUS_CONFIG_FD               1       // SPI bus configuration: Full-duplex
#define SPI_BUS_CONFIG_HD               2       // SPI bus configuration: Half-duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY   3       // SPI bus configuration: Simplex RX only

#define SPI_SCLK_SPEED_DIV2             0       // SPI clock speed: Divide by 2
#define SPI_SCLK_SPEED_DIV4             1       // SPI clock speed: Divide by 4
#define SPI_SCLK_SPEED_DIV8             2       // SPI clock speed: Divide by 8
#define SPI_SCLK_SPEED_DIV16            3       // SPI clock speed: Divide by 16
#define SPI_SCLK_SPEED_DIV32            4       // SPI clock speed: Divide by 32
#define SPI_SCLK_SPEED_DIV64            5       // SPI clock speed: Divide by 64
#define SPI_SCLK_SPEED_DIV128           6       // SPI clock speed: Divide by 128
#define SPI_SCLK_SPEED_DIV256           7       // SPI clock speed: Divide by 256

#define SPI_DFF_8BITS                   0       // SPI data frame format: 8-bit
#define SPI_DFF_16BITS                  1       // SPI data frame format: 16-bit

#define SPI_CPOL_HIGH                   1       // SPI clock polarity: High when idle
#define SPI_CPOL_LOW                    0       // SPI clock polarity: Low when idle

#define SPI_CPHA_HIGH                   1       // SPI clock phase: Capture on second edge
#define SPI_CPHA_LOW                    0       // SPI clock phase: Capture on first edge

#define SPI_SSM_EN                      1       // SPI slave select management: Hardware
#define SPI_SSM_DI                      0       // SPI slave select management: Software

/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG    ( 1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG   ( 1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG   ( 1 << SPI_SR_BSY)


/*
 * SPI application states
 */
#define SPI_READY 					0
#define SPI_BUSY_IN_RX 				1
#define SPI_BUSY_IN_TX 				2

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3
#define SPI_EVENT_CRC_ERR    4



// configuration structure for SPIx peripheral
typedef struct {
    uint8_t SPI_DeviceMode;  // Specifies the SPI device mode (Master/Slave)
    uint8_t SPI_BusConfig;   // Defines the SPI bus configuration (Full-duplex/Half-duplex/Simplex)
    uint8_t SPI_SClkSpeed;   // Sets the SPI clock speed
    uint8_t SPI_DFF;         // Specifies the SPI data frame format (8-bit/16-bit)
    uint8_t SPI_CPOL;        // Defines the clock polarity (Idle High/Idle Low)
    uint8_t SPI_CPHA;        // Sets the clock phase (1st/2nd clock transition)
    uint8_t SPI_SSM;         // Enables/Disables software slave management
} SPI_Config_t;

// Handler structure for SPIx
typedef struct {
    SPI_RegDef_t *pSPIx;    // Pointer to SPI peripheral register definition
    SPI_Config_t SPIConfig;  // SPI configuration settings
    uint8_t  *pTxBuffer;
    uint8_t  *pRxBuffer;
    uint32_t  TxLen;
    uint32_t  RxLen;
    uint32_t  TxState;
    uint32_t  RxState;
} SPI_Handle_t;


// ===================================  APIs supported by this driver =========================================

// peripheral clock
void SPI_PClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

// initialization and de-initialization
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

// enable or disable SPI
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

// SSI config
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

// data read / write
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

// data read / write interrupt based
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

// interrupt handling and configuration
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQHandler(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);

#endif /* STM32F407XX_SPI_DRIVER_H_ */
