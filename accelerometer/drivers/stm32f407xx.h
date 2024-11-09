/*
 * stmf407xx.h
 *
 *  Created on: Oct 23, 2024
 *      Author: athul-muralidhar
 */

#ifndef STM32F407XX_H_
#define STM32F407XX_H_

#include<stdint.h>
#include <stddef.h>

#include<stddef.h>
#include<stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))


/**********************************START:Processor Specific Details **********************************/

/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */
#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 ) /*!< Interrupt Set-Enable Register 0 address */
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 ) /*!< Interrupt Set-Enable Register 1 address */
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 ) /*!< Interrupt Set-Enable Register 2 address */
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c ) /*!< Interrupt Set-Enable Register 3 address */

/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180) /*!< Interrupt Clear-Enable Register 0 address */
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184) /*!< Interrupt Clear-Enable Register 1 address */
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188) /*!< Interrupt Clear-Enable Register 2 address */
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C) /*!< Interrupt Clear-Enable Register 3 address */

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400) /*!< Base address for priority registers */

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4 /*!< Number of priority bits implemented in the priority register */

/*
 * Base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR						0x08000000U   	/*!< Base address of Flash memory */
#define SRAM1_BASEADDR						0x20000000U  	/*!< Base address of SRAM1 memory */
#define SRAM2_BASEADDR						0x2001C000U 	/*!< Base address of SRAM2 memory */
#define ROM_BASEADDR						0x1FFF0000U    /*!< Base address of ROM memory */
#define SRAM 								SRAM1_BASEADDR /*!< Default SRAM base address set to SRAM1 */

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASEADDR 						0x40000000U /*!< Base address for peripheral buses */
#define APB1PERIPH_BASEADDR						PERIPH_BASEADDR /*!< Base address for APB1 peripherals */
#define APB2PERIPH_BASEADDR						0x40010000U /*!< Base address for APB2 peripherals */
#define AHB1PERIPH_BASEADDR						0x40020000U /*!< Base address for AHB1 peripherals */
#define AHB2PERIPH_BASEADDR						0x50000000U /*!< Base address for AHB2 peripherals */

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO : Complete for all other peripherals
 */

#define GPIOA_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0000) /*!< Base address of GPIO Port A */
#define GPIOB_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0400) /*!< Base address of GPIO Port B */
#define GPIOC_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0800) /*!< Base address of GPIO Port C */
#define GPIOD_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0C00) /*!< Base address of GPIO Port D */
#define GPIOE_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1000) /*!< Base address of GPIO Port E */
#define GPIOF_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1400) /*!< Base address of GPIO Port F */
#define GPIOG_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1800) /*!< Base address of GPIO Port G */
#define GPIOH_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1C00) /*!< Base address of GPIO Port H */
#define GPIOI_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x2000) /*!< Base address of GPIO Port I */
#define RCC_BASEADDR                     (AHB1PERIPH_BASEADDR + 0x3800) /*!< Base address of Reset and Clock Control */

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO : Complete for all other peripherals
 */
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400) /*!< Base address of I2C1 */
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800) /*!< Base address of I2C2 */
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00) /*!< Base address of I2C3 */

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800) /*!< Base address of SPI2 */
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00) /*!< Base address of SPI3 */

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400) /*!< Base address of USART2 */
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800) /*!< Base address of USART3 */
#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00) /*!< Base address of UART4 */
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000) /*!< Base address of UART5 */


/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO : Complete for all other peripherals
 */
#define EXTI_BASEADDR   (APB2PERIPH_BASEADDR + 0x3C00)  // Base address of External Interrupt/Event Controller (EXTI)
#define SPI1_BASEADDR   (APB2PERIPH_BASEADDR + 0x3000)  // Base address of Serial Peripheral Interface 1 (SPI1)
#define SYSCFG_BASEADDR (APB2PERIPH_BASEADDR + 0x3800)  // Base address of System Configuration Controller (SYSCFG)
#define USART1_BASEADDR (APB2PERIPH_BASEADDR + 0x1000)  // Base address of USART1
#define USART6_BASEADDR (APB2PERIPH_BASEADDR + 0x1400)  // Base address of USART6


/*
 * Base addresses of peripherals
 * *
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */
#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR) /*!< Pointer to GPIO Port A registers */
#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR) /*!< Pointer to GPIO Port B registers */
#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR) /*!< Pointer to GPIO Port C registers */
#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR) /*!< Pointer to GPIO Port D registers */
#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR) /*!< Pointer to GPIO Port E registers */
#define GPIOF  				((GPIO_RegDef_t*)GPIOF_BASEADDR) /*!< Pointer to GPIO Port F registers */
#define GPIOG  				((GPIO_RegDef_t*)GPIOG_BASEADDR) /*!< Pointer to GPIO Port G registers */
#define GPIOH  				((GPIO_RegDef_t*)GPIOH_BASEADDR) /*!< Pointer to GPIO Port H registers */
#define GPIOI  				((GPIO_RegDef_t*)GPIOI_BASEADDR) /*!< Pointer to GPIO Port I registers */

#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR) /*!< Pointer to RCC registers */
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR) /*!< Pointer to EXTI registers */
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR) /*!< Pointer to SYSCFG registers */

#define SPI1  				((SPI_RegDef_t*)SPI1_BASEADDR) /*!< Pointer to SPI1 registers */
#define SPI2  				((SPI_RegDef_t*)SPI2_BASEADDR) /*!< Pointer to SPI2 registers */
#define SPI3  				((SPI_RegDef_t*)SPI3_BASEADDR) /*!< Pointer to SPI3 registers */

#define I2C1  				((I2C_RegDef_t*)I2C1_BASEADDR) /*!< Pointer to I2C1 registers */
#define I2C2  				((I2C_RegDef_t*)I2C2_BASEADDR) /*!< Pointer to I2C2 registers */
#define I2C3  				((I2C_RegDef_t*)I2C3_BASEADDR) /*!< Pointer to I2C3 registers */

#define USART1  			((USART_RegDef_t*)USART1_BASEADDR) /*!< Pointer to USART1 registers */
#define USART2  			((USART_RegDef_t*)USART2_BASEADDR) /*!< Pointer to USART2 registers */
#define USART3  			((USART_RegDef_t*)USART3_BASEADDR) /*!< Pointer to USART3 registers */
#define UART4  				((USART_RegDef_t*)UART4_BASEADDR) /*!< Pointer to UART4 registers */
#define UART5  				((USART_RegDef_t*)UART5_BASEADDR) /*!< Pointer to UART5 registers */
#define USART6  			((USART_RegDef_t*)USART6_BASEADDR) /*!< Pointer to USART6 registers */

/* Clock Enable Macros for GPIOx peripherals */
#define GPIOA_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 0)) /*!< Enable GPIOA peripheral clock */
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1)) /*!< Enable GPIOB peripheral clock */
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2)) /*!< Enable GPIOC peripheral clock */
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3)) /*!< Enable GPIOD peripheral clock */
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4)) /*!< Enable GPIOE peripheral clock */
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5)) /*!< Enable GPIOF peripheral clock */
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6)) /*!< Enable GPIOG peripheral clock */
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7)) /*!< Enable GPIOH peripheral clock */
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8)) /*!< Enable GPIOI peripheral clock */

/* Clock Enable Macros for I2Cx peripherals */
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21)) /*!< Enable I2C1 peripheral clock */
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22)) /*!< Enable I2C2 peripheral clock */
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23)) /*!< Enable I2C3 peripheral clock */

/* Clock Enable Macros for SPIx peripherals */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12)) /*!< Enable SPI1 peripheral clock */
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14)) /*!< Enable SPI2 peripheral clock */
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15)) /*!< Enable SPI3 peripheral clock */
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13)) /*!< Enable SPI4 peripheral clock */

/* Clock Enable Macros for USARTx peripherals */
#define USART1_PCCK_EN() (RCC->APB2ENR |= (1 << 4)) /*!< Enable USART1 peripheral clock */
#define USART2_PCCK_EN() (RCC->APB1ENR |= (1 << 17)) /*!< Enable USART2 peripheral clock */
#define USART3_PCCK_EN() (RCC->APB1ENR |= (1 << 18)) /*!< Enable USART3 peripheral clock */
#define UART4_PCCK_EN()  (RCC->APB1ENR |= (1 << 19)) /*!< Enable UART4 peripheral clock */
#define UART5_PCCK_EN()  (RCC->APB1ENR |= (1 << 20)) /*!< Enable UART5 peripheral clock */
#define USART6_PCCK_EN() (RCC->APB1ENR |= (1 << 5)) /*!< Enable USART6 peripheral clock */

/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14)) /*!< Enable SYSCFG peripheral clock */


/*
 * Clock Disable Macros for GPIOx peripherals
 */
//#define GPIOA_PCLK_DI()

/*
 * Clock Disable Macros for SPIx peripherals
 */

/*
 * Clock Disable Macros for USARTx peripherals
 */


/*
 * Clock Disable Macros for SYSCFG peripheral
 */


/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0) /*!< Reset GPIOA peripheral */
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0) /*!< Reset GPIOB peripheral */
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0) /*!< Reset GPIOC peripheral */
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0) /*!< Reset GPIOD peripheral */
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0) /*!< Reset GPIOE peripheral */
#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0) /*!< Reset GPIOF peripheral */
#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0) /*!< Reset GPIOG peripheral */
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0) /*!< Reset GPIOH peripheral */
#define GPIOI_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0) /*!< Reset GPIOI peripheral */

/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:\
								        (x == GPIOH)?7: \
								        (x == GPIOI)?8:0) /*!< Convert GPIO base address to port code */

/* IRQ(Interrupt Request) Numbers of STM32F407x MCU */
#define IRQ_NO_EXTI0 		6 /*!< IRQ number for EXTI Line0 interrupt */
#define IRQ_NO_EXTI1 		7 /*!< IRQ number for EXTI Line1 interrupt */
#define IRQ_NO_EXTI2 		8 /*!< IRQ number for EXTI Line2 interrupt */
#define IRQ_NO_EXTI3 		9 /*!< IRQ number for EXTI Line3 interrupt */
#define IRQ_NO_EXTI4 		10 /*!< IRQ number for EXTI Line4 interrupt */
#define IRQ_NO_EXTI9_5 		23 /*!< IRQ number for EXTI Line[9:5] interrupts */
#define IRQ_NO_EXTI15_10 	40 /*!< IRQ number for EXTI Line[15:10] interrupts */
#define IRQ_NO_SPI1			35 /*!< IRQ number for SPI1 global interrupt */
#define IRQ_NO_SPI2         36 /*!< IRQ number for SPI2 global interrupt */
#define IRQ_NO_SPI3         51 /*!< IRQ number for SPI3 global interrupt */
#define IRQ_NO_SPI4 /*!< IRQ number for SPI4 global interrupt (to be defined) */
#define IRQ_NO_I2C1_EV     31 /*!< IRQ number for I2C1 event interrupt */
#define IRQ_NO_I2C1_ER     32 /*!< IRQ number for I2C1 error interrupt */
#define IRQ_NO_USART1	    37 /*!< IRQ number for USART1 global interrupt */
#define IRQ_NO_USART2	    38 /*!< IRQ number for USART2 global interrupt */
#define IRQ_NO_USART3	    39 /*!< IRQ number for USART3 global interrupt */
#define IRQ_NO_UART4	    52 /*!< IRQ number for UART4 global interrupt */
#define IRQ_NO_UART5	    53 /*!< IRQ number for UART5 global interrupt */
#define IRQ_NO_USART6	    71 /*!< IRQ number for USART6 global interrupt */

/* Macros for all the possible priority levels */
#define NVIC_IRQ_PRI0    0 /*!< Highest interrupt priority level */
#define NVIC_IRQ_PRI15    15 /*!< Lowest interrupt priority level */

/* Generic macros */
#define ENABLE 				1 /*!< Enable state */
#define DISABLE 			0 /*!< Disable state */
#define SET 				ENABLE /*!< Set state (same as ENABLE) */
#define RESET 				DISABLE /*!< Reset state (same as DISABLE) */
#define GPIO_PIN_SET        SET /*!< GPIO pin set state */
#define GPIO_PIN_RESET      RESET /*!< GPIO pin reset state */
#define FLAG_RESET         RESET /*!< Flag reset state */
#define FLAG_SET 			SET /*!< Flag set state */

/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0 /*!< Clock phase */
#define SPI_CR1_CPOL      				 1 /*!< Clock polarity */
#define SPI_CR1_MSTR     				 2 /*!< Master selection */
#define SPI_CR1_BR   					 3 /*!< Baud rate control */
#define SPI_CR1_SPE     				 6 /*!< SPI enable */
#define SPI_CR1_LSBFIRST   			 	 7 /*!< Frame format */
#define SPI_CR1_SSI     				 8 /*!< Internal slave select */
#define SPI_CR1_SSM      				 9 /*!< Software slave management */
#define SPI_CR1_RXONLY      		 	10 /*!< Receive only */
#define SPI_CR1_DFF     			 	11 /*!< Data frame format */
#define SPI_CR1_CRCNEXT   			 	12 /*!< CRC transfer next */
#define SPI_CR1_CRCEN   			 	13 /*!< Hardware CRC calculation enable */
#define SPI_CR1_BIDIOE     			 	14 /*!< Output enable in bidirectional mode */
#define SPI_CR1_BIDIMODE      			15 /*!< Bidirectional data mode enable */

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0 /*!< Rx buffer DMA enable */
#define SPI_CR2_TXDMAEN				 	1 /*!< Tx buffer DMA enable */
#define SPI_CR2_SSOE				 	2 /*!< SS output enable */
#define SPI_CR2_FRF						4 /*!< Frame format */
#define SPI_CR2_ERRIE					5 /*!< Error interrupt enable */
#define SPI_CR2_RXNEIE				 	6 /*!< RX buffer not empty interrupt enable */
#define SPI_CR2_TXEIE					7 /*!< Tx buffer empty interrupt enable */

/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0 /*!< Receive buffer not empty */
#define SPI_SR_TXE				 		1 /*!< Transmit buffer empty */
#define SPI_SR_CHSIDE				 	2 /*!< Channel side */
#define SPI_SR_UDR					 	3 /*!< Underrun flag */
#define SPI_SR_CRCERR				 	4 /*!< CRC error flag */
#define SPI_SR_MODF					 	5 /*!< Mode fault */
#define SPI_SR_OVR					 	6 /*!< Overrun flag */
#define SPI_SR_BSY					 	7 /*!< Busy flag */
#define SPI_SR_FRE					 	8 /*!< Frame format error */

/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0 /*!< Peripheral enable */
#define I2C_CR1_NOSTRETCH  				7 /*!< Clock stretching disable (Slave mode) */
#define I2C_CR1_START 					8 /*!< Start generation */
#define I2C_CR1_STOP  				 	9 /*!< Stop generation */
#define I2C_CR1_ACK 				 	10 /*!< Acknowledge enable */
#define I2C_CR1_SWRST  				 	15 /*!< Software reset */

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0 /*!< Peripheral clock frequency */
#define I2C_CR2_ITERREN				 	8 /*!< Error interrupt enable */
#define I2C_CR2_ITEVTEN				 	9 /*!< Event interrupt enable */
#define I2C_CR2_ITBUFEN 			    10 /*!< Buffer interrupt enable */

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0 /*!< Interface address bit 0 */
#define I2C_OAR1_ADD71 				 	 1 /*!< Interface address bits [7:1] */
#define I2C_OAR1_ADD98  			 	 8 /*!< Interface address bits [9:8] */
#define I2C_OAR1_ADDMODE   			 	15 /*!< Addressing mode (slave mode) */

/*
 * Bit position definitions I2C_SR1
 */
#define I2C_SR1_SB 					 	0 /*!< Start bit (Master mode) */
#define I2C_SR1_ADDR 				 	1 /*!< Address sent (master mode)/matched (slave mode) */
#define I2C_SR1_BTF 					2 /*!< Byte transfer finished */
#define I2C_SR1_ADD10 					3 /*!< 10-bit header sent (Master mode) */
#define I2C_SR1_STOPF 					4 /*!< Stop detection (slave mode) */
#define I2C_SR1_RXNE 					6 /*!< Data register not empty (receivers) */
#define I2C_SR1_TXE 					7 /*!< Data register empty (transmitters) */
#define I2C_SR1_BERR 					8 /*!< Bus error */
#define I2C_SR1_ARLO 					9 /*!< Arbitration lost (master mode) */
#define I2C_SR1_AF 					 	10 /*!< Acknowledge failure */
#define I2C_SR1_OVR 					11 /*!< Overrun/Underrun */
#define I2C_SR1_TIMEOUT 				14 /*!< Timeout or Tlow error */

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0 /*!< Master/slave */
#define I2C_SR2_BUSY 					1 /*!< Bus busy */
#define I2C_SR2_TRA 					2 /*!< Transmitter/receiver */
#define I2C_SR2_GENCALL 				4 /*!< General call address (Slave mode) */
#define I2C_SR2_DUALF 					7 /*!< Dual flag (Slave mode) */

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0 /*!< Clock control register in Fm/Sm mode (Master mode) */
#define I2C_CCR_DUTY 					14 /*!< Fast mode duty cycle */
#define I2C_CCR_FS  				 	15 /*!< I2C master mode selection */

/******************************************************************************************
 *Bit position definitions of USART peripheral
 *******************************************************************************************
*/
/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0 /*!< Send break */
#define USART_CR1_RWU 					1 /*!< Receiver wakeup */
#define USART_CR1_RE  					2 /*!< Receiver enable */
#define USART_CR1_TE 					3 /*!< Transmitter enable */
#define USART_CR1_IDLEIE 				4 /*!< IDLE interrupt enable */
#define USART_CR1_RXNEIE  				5 /*!< RXNE interrupt enable */
#define USART_CR1_TCIE					6 /*!< Transmission complete interrupt enable */
#define USART_CR1_TXEIE					7 /*!< TXE interrupt enable */
#define USART_CR1_PEIE 					8 /*!< PE interrupt enable */
#define USART_CR1_PS 					9 /*!< Parity selection */
#define USART_CR1_PCE 					10 /*!< Parity control enable */
#define USART_CR1_WAKE  				11 /*!< Wakeup method */
#define USART_CR1_M 					12 /*!< Word length */
#define USART_CR1_UE 					13 /*!< USART enable */
#define USART_CR1_OVER8  				15 /*!< Oversampling mode */

/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0 /*!< Address of the USART node */
#define USART_CR2_LBDL   				5 /*!< LIN break detection length */
#define USART_CR2_LBDIE  				6 /*!< LIN break detection interrupt enable */
#define USART_CR2_LBCL   				8 /*!< Last bit clock pulse */
#define USART_CR2_CPHA   				9 /*!< Clock phase */
#define USART_CR2_CPOL   				10 /*!< Clock polarity */
#define USART_CR2_STOP   				12 /*!< STOP bits */
#define USART_CR2_LINEN   				14 /*!< LIN mode enable */

/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0 /*!< Error interrupt enable */
#define USART_CR3_IREN   				1 /*!< IrDA mode enable */
#define USART_CR3_IRLP  				2 /*!< IrDA low-power */
#define USART_CR3_HDSEL   				3 /*!< Half-duplex selection */
#define USART_CR3_NACK   				4 /*!< Smartcard NACK enable */
#define USART_CR3_SCEN   				5 /*!< Smartcard mode enable */
#define USART_CR3_DMAR  				6 /*!< DMA enable receiver */
#define USART_CR3_DMAT   				7 /*!< DMA enable transmitter */
#define USART_CR3_RTSE   				8 /*!< RTS enable */
#define USART_CR3_CTSE   				9 /*!< CTS enable */
#define USART_CR3_CTSIE   				10 /*!< CTS interrupt enable */
#define USART_CR3_ONEBIT   				11 /*!< One sample bit method enable */

/*
 * Bit position definitions USART_SR
 */
#define USART_SR_PE        				0 /*!< Parity error */
#define USART_SR_FE        				1 /*!< Framing error */
#define USART_SR_NE        				2 /*!< Noise error flag */
#define USART_SR_ORE       				3 /*!< Overrun error */
#define USART_SR_IDLE       			4 /*!< IDLE line detected */
#define USART_SR_RXNE        			5 /*!< Read data register not empty */
#define USART_SR_TC        				6 /*!< Transmission complete */
#define USART_SR_TXE        			7 /*!< Transmit data register empty */
#define USART_SR_LBD        			8 /*!< LIN break detection flag */
#define USART_SR_CTS        			9 /*!< CTS flag */

/**********************************peripheral register definition structures **********************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */
typedef struct
{
    __vo uint32_t MODER;    /*!< GPIO port mode register,                    Address offset: 0x00      */
    __vo uint32_t OTYPER;   /*!< GPIO port output type register,             Address offset: 0x04      */
    __vo uint32_t OSPEEDR;  /*!< GPIO port output speed register,            Address offset: 0x08      */
    __vo uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,       Address offset: 0x0C      */
    __vo uint32_t IDR;      /*!< GPIO port input data register,              Address offset: 0x10      */
    __vo uint32_t ODR;      /*!< GPIO port output data register,             Address offset: 0x14      */
    __vo uint32_t BSRR;     /*!< GPIO port bit set/reset register,           Address offset: 0x18      */
    __vo uint32_t LCKR;     /*!< GPIO port configuration lock register,      Address offset: 0x1C      */
    __vo uint32_t AFR[2];   /*!< GPIO alternate function registers,          Address offset: 0x20-0x24 */
}GPIO_RegDef_t;

/*
 * Peripheral register definition structure for RCC
 */
typedef struct
{
  __vo uint32_t CR;            /*!< Clock control register,                                  Address offset: 0x00 */
  __vo uint32_t PLLCFGR;       /*!< PLL configuration register,                              Address offset: 0x04 */
  __vo uint32_t CFGR;          /*!< Clock configuration register,                            Address offset: 0x08 */
  __vo uint32_t CIR;           /*!< Clock interrupt register,                                Address offset: 0x0C */
  __vo uint32_t AHB1RSTR;      /*!< AHB1 peripheral reset register,                          Address offset: 0x10 */
  __vo uint32_t AHB2RSTR;      /*!< AHB2 peripheral reset register,                          Address offset: 0x14 */
  __vo uint32_t AHB3RSTR;      /*!< AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                */
  __vo uint32_t APB1RSTR;      /*!< APB1 peripheral reset register,                          Address offset: 0x20 */
  __vo uint32_t APB2RSTR;      /*!< APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                           */
  __vo uint32_t AHB1ENR;       /*!< AHB1 peripheral clock enable register,                   Address offset: 0x30 */
  __vo uint32_t AHB2ENR;       /*!< AHB2 peripheral clock enable register,                   Address offset: 0x34 */
  __vo uint32_t AHB3ENR;       /*!< AHB3 peripheral clock enable register,                   Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                */
  __vo uint32_t APB1ENR;       /*!< APB1 peripheral clock enable register,                   Address offset: 0x40 */
  __vo uint32_t APB2ENR;       /*!< APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                           */
  __vo uint32_t AHB1LPENR;     /*!< AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  __vo uint32_t AHB2LPENR;     /*!< AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  __vo uint32_t AHB3LPENR;     /*!< AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                */
  __vo uint32_t APB1LPENR;     /*!< APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  __vo uint32_t APB2LPENR;     /*!< APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                           */
  __vo uint32_t BDCR;          /*!< Backup domain control register,                          Address offset: 0x70 */
  __vo uint32_t CSR;           /*!< Clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                           */
  __vo uint32_t SSCGR;         /*!< Spread spectrum clock generation register,               Address offset: 0x80 */
  __vo uint32_t PLLI2SCFGR;    /*!< PLLI2S configuration register,                           Address offset: 0x84 */
  __vo uint32_t PLLSAICFGR;    /*!< PLLSAI configuration register,                           Address offset: 0x88 */
  __vo uint32_t DCKCFGR;       /*!< Dedicated Clocks configuration register,                 Address offset: 0x8C */
  __vo uint32_t CKGATENR;      /*!< Clocks gated enable register,                            Address offset: 0x90 */
  __vo uint32_t DCKCFGR2;      /*!< Dedicated Clocks configuration register 2,               Address offset: 0x94 */

} RCC_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */
typedef struct
{
    __vo uint32_t IMR;    /*!< Interrupt mask register,                 Address offset: 0x00 */
    __vo uint32_t EMR;    /*!< Event mask register,                     Address offset: 0x04 */
    __vo uint32_t RTSR;   /*!< Rising trigger selection register,       Address offset: 0x08 */
    __vo uint32_t FTSR;   /*!< Falling trigger selection register,      Address offset: 0x0C */
    __vo uint32_t SWIER;  /*!< Software interrupt event register,       Address offset: 0x10 */
    __vo uint32_t PR;     /*!< Pending register,                        Address offset: 0x14 */
}EXTI_RegDef_t;

/*
 * Peripheral register definition structure for SPI
 */
typedef struct
{
    __vo uint32_t CR1;        /*!< Control register 1,                   Address offset: 0x00 */
    __vo uint32_t CR2;        /*!< Control register 2,                   Address offset: 0x04 */
    __vo uint32_t SR;         /*!< Status register,                      Address offset: 0x08 */
    __vo uint32_t DR;         /*!< Data register,                        Address offset: 0x0C */
    __vo uint32_t CRCPR;      /*!< CRC polynomial register,              Address offset: 0x10 */
    __vo uint32_t RXCRCR;     /*!< RX CRC register,                      Address offset: 0x14 */
    __vo uint32_t TXCRCR;     /*!< TX CRC register,                      Address offset: 0x18 */
    __vo uint32_t I2SCFGR;    /*!< I2S configuration register,           Address offset: 0x1C */
    __vo uint32_t I2SPR;      /*!< I2S prescaler register,               Address offset: 0x20 */
} SPI_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct
{
    __vo uint32_t MEMRMP;       /*!< Memory remap register,              Address offset: 0x00 */
    __vo uint32_t PMC;          /*!< Peripheral mode configuration register, Address offset: 0x04 */
    __vo uint32_t EXTICR[4];    /*!< External interrupt configuration registers, Address offset: 0x08-0x14 */
    uint32_t      RESERVED1[2]; /*!< Reserved,                           Address offset: 0x18-0x1C */
    __vo uint32_t CMPCR;        /*!< Compensation cell control register, Address offset: 0x20 */
    uint32_t      RESERVED2[2]; /*!< Reserved,                           Address offset: 0x24-0x28 */
    __vo uint32_t CFGR;         /*!< Configuration register,             Address offset: 0x2C */
} SYSCFG_RegDef_t;

/*
 * Peripheral register definition structure for I2C
 */
typedef struct
{
    __vo uint32_t CR1;        /*!< Control register 1,                   Address offset: 0x00 */
    __vo uint32_t CR2;        /*!< Control register 2,                   Address offset: 0x04 */
    __vo uint32_t OAR1;       /*!< Own address register 1,               Address offset: 0x08 */
    __vo uint32_t OAR2;       /*!< Own address register 2,               Address offset: 0x0C */
    __vo uint32_t DR;         /*!< Data register,                        Address offset: 0x10 */
    __vo uint32_t SR1;        /*!< Status register 1,                    Address offset: 0x14 */
    __vo uint32_t SR2;        /*!< Status register 2,                    Address offset: 0x18 */
    __vo uint32_t CCR;        /*!< Clock control register,               Address offset: 0x1C */
    __vo uint32_t TRISE;      /*!< TRISE register,                       Address offset: 0x20 */
    __vo uint32_t FLTR;       /*!< FLTR register,                        Address offset: 0x24 */
}I2C_RegDef_t;

/*
 * Peripheral register definition structure for USART
 */
typedef struct
{
    __vo uint32_t SR;         /*!< Status register,                      Address offset: 0x00 */
    __vo uint32_t DR;         /*!< Data register,                        Address offset: 0x04 */
    __vo uint32_t BRR;        /*!< Baud rate register,                   Address offset: 0x08 */
    __vo uint32_t CR1;        /*!< Control register 1,                   Address offset: 0x0C */
    __vo uint32_t CR2;        /*!< Control register 2,                   Address offset: 0x10 */
    __vo uint32_t CR3;        /*!< Control register 3,                   Address offset: 0x14 */
    __vo uint32_t GTPR;       /*!< Guard time and prescaler register,    Address offset: 0x18 */
} USART_RegDef_t;


#endif /* STM32F407XX_H_ */
