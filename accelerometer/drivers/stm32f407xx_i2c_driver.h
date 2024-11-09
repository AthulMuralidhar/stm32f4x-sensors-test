/*
 * stm32f4xx_i2c_driver.h
 *
 *  Created on: Nov 6, 2024
 *      Author: athul-muralidhar
 */

#ifndef STM32F407XX_I2C_DRIVER_H_
#define STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx_spi_driver.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
    uint32_t I2C_SCLSpeed;      // Clock speed for I2C SCL line
    uint8_t  I2C_DeviceAddress; // 7-bit device address for I2C communication
    uint8_t  I2C_AckControl;    // Control for ACK bit (enable/disable)
    uint8_t  I2C_FMDutyCycle;   // Duty cycle for Fast Mode (FM) I2C communication
} I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */
typedef struct
{
    I2C_RegDef_t    *pI2Cx;     // Pointer to I2C peripheral registers
    I2C_Config_t    I2C_Config; // Configuration settings for I2C
    uint8_t         *pTxBuffer; // Pointer to transmit buffer
    uint8_t         *pRxBuffer; // Pointer to receive buffer
    uint32_t        TxLen;      // Length of data to transmit
    uint32_t        RxLen;      // Length of data to receive
    uint8_t         TxRxState;  // Current state of I2C communication
    uint8_t         DevAddr;    // Address of the target slave device
    uint32_t        RxSize;     // Size of data to receive
    uint8_t         Sr;         // Repeated start condition flag
} I2C_Handle_t;


/*
 * I2C application states
 */
#define I2C_READY 					0 // I2C peripheral is ready for communication
#define I2C_BUSY_IN_RX 				1 // I2C peripheral is busy receiving data
#define I2C_BUSY_IN_TX 				2 // I2C peripheral is busy transmitting data

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM 	100000    // Standard Mode I2C speed (100 kHz)
#define I2C_SCL_SPEED_FM4K 	400000    // Fast Mode I2C speed (400 kHz)
#define I2C_SCL_SPEED_FM2K  200000    // Fast Mode I2C speed (200 kHz)

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE        1       // Enable I2C acknowledgment
#define I2C_ACK_DISABLE       0       // Disable I2C acknowledgment

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2        0        // Fast Mode duty cycle 2
#define I2C_FM_DUTY_16_9     1        // Fast Mode duty cycle 16/9

/*
 * I2C related status flags definitions
 */
#define I2C_FLAG_TXE   		( 1 << I2C_SR1_TXE)    // Data register empty (transmitters)
#define I2C_FLAG_RXNE   	( 1 << I2C_SR1_RXNE)   // Data register not empty (receivers)
#define I2C_FLAG_SB			( 1 << I2C_SR1_SB)     // Start bit generated
#define I2C_FLAG_OVR  		( 1 << I2C_SR1_OVR)    // Overrun/Underrun
#define I2C_FLAG_AF   		( 1 << I2C_SR1_AF)     // Acknowledge failure
#define I2C_FLAG_ARLO 		( 1 << I2C_SR1_ARLO)   // Arbitration lost
#define I2C_FLAG_BERR 		( 1 << I2C_SR1_BERR)   // Bus error
#define I2C_FLAG_STOPF 		( 1 << I2C_SR1_STOPF)  // Stop detection flag
#define I2C_FLAG_ADD10 		( 1 << I2C_SR1_ADD10)  // 10-bit header sent (Master mode)
#define I2C_FLAG_BTF  		( 1 << I2C_SR1_BTF)    // Byte transfer finished
#define I2C_FLAG_ADDR 		( 1 << I2C_SR1_ADDR)   // Address sent (master mode) / matched (slave mode)
#define I2C_FLAG_TIMEOUT 	( 1 << I2C_SR1_TIMEOUT) // Timeout or Tlow error

#define I2C_DISABLE_SR  	RESET               // Disable I2C repeated start
#define I2C_ENABLE_SR   	SET                 // Enable I2C repeated start

/*
 * I2C application events macros
 */
#define I2C_EV_TX_CMPLT  	 	0    // Transmission complete event
#define I2C_EV_RX_CMPLT  	 	1    // Reception complete event
#define I2C_EV_STOP       		2    // Stop condition detected event
#define I2C_ERROR_BERR 	 		3    // Bus error event
#define I2C_ERROR_ARLO  		4    // Arbitration lost event
#define I2C_ERROR_AF    		5    // Acknowledge failure event
#define I2C_ERROR_OVR   		6    // Overrun/Underrun error event
#define I2C_ERROR_TIMEOUT 		7    // Timeout error event
#define I2C_EV_DATA_REQ         8    // Data request event (slave mode)
#define I2C_EV_DATA_RCV         9    // Data received event (slave mode)
/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


/*
 * Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);


void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);


/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);

/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);


#endif /* STM32F407XX_I2C_DRIVER_H_ */
