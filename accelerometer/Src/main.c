
#include <stdint.h>
#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_FN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK  - PA5
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPIPins);

	//MOSI - PA7
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPIPins);

	//MISO  - PA6
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SPIPins);


	//NSS  - PE3

	SPIPins.pGPIOx = GPIOE;
//	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&SPIPins);


}



int main(void)
{
    /* Loop forever */

	// TODO: make the accelerometer work
	// work with accelerometer via SPI at PE3: CS I2C/SPI - given in user manual
	// so basically PE3 is NSS for this accelerometer
	// these are the general labeling:
	//  CS		CS_I2C/SPI		PE3  ---> NSS
	//	MISO	SDO				PA6  ---> SDO is the output line for sensors
	//	MOSI	SDA/SDI/SDO		PA7
	//	Clock	SCL/SPC			PA5



	char user_data[] = "Hello world";

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	//this makes NSS signal internally high and avoids MODF error
	SPI_SSIConfig(SPI2,ENABLE);

	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,ENABLE);

	//to send data
	SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

	//lets confirm SPI is not busy
	while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

	//Disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,DISABLE);

	while(1);

	return 0;
}
