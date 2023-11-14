/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Nov 9, 2023
 *      Author: macbook
 */

#include "stm32f446xx_gpio_driver.h"


/*
 * Peripheral Clock setup
 */

/*******************************************************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]		- base address of the GPIO peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- none
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}else{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}

	}
}

/*
 * Init and De-init
 */

/*******************************************************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- This function initialize given GPIO port
 *
 * @param[in]		- address of structure with configuration of GPIO peripheral
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- none
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	//1. configure the mode of gpio pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &- ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}else
	{

	}

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER &- ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;
	temp = 0;

	//3. configure pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &- ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//4. configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &- ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint32_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= (0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

/*******************************************************************************************
 * @fn				- GPIO_DeInit
 *
 * @brief			- This function de-initialize given GPIO port
 *
 * @param[in]		- base address of the GPIO peripheral
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- none
 *
 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

/*
 * Data rad and write
 */

/*******************************************************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief			- This function reads value from GPIO pin
 *
 * @param[in]		- base address of the GPIO peripheral
 * @param[in]		- number of pin
 * @param[in]		-
 *
 * @return			- 0 or 1
 *
 * @note			- none
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/*******************************************************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief			- This function reads value from GPIO port
 *
 * @param[in]		- base address of the GPIO peripheral
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- Red value from GPIO PORT
 *
 * @note			- none
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/*******************************************************************************************
 * @fn				- GPIO_WriteToOutputPin
 *
 * @brief			- This function writes value to given GPIO pin
 *
 * @param[in]		- base address of the GPIO peripheral
 * @param[in]		- number of pin
 * @param[in]		- written value
 *
 * @return			- none
 *
 * @note			- none
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*******************************************************************************************
 * @fn				- GPIO_WriteToOutputPort
 *
 * @brief			- This function writes value to given GPIO port
 *
 * @param[in]		- base address of the GPIO peripheral
 * @param[in]		- written value
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- none
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/*******************************************************************************************
 * @fn				- GPIO_ToggleOutputPin
 *
 * @brief			- This function toggles given GPIO pin
 *
 * @param[in]		- base address of the GPIO peripheral
 * @param[in]		- number of pin
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- none
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Configuration and ISR handling
 */

/*******************************************************************************************
 * @fn				- GPIO_IRQConfig
 *
 * @brief			- This function configures IRQ
 *
 * @param[in]		- number of IRQ
 * @param[in]		- given priority
 * @param[in]		- ENABLE or DISABLE given IRQ
 *
 * @return			- none
 *
 * @note			- none
 *
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi)
{

}

/*******************************************************************************************
 * @fn				- GPIO_IRQHandling
 *
 * @brief			- This function handle given GPIO pin
 *
 * @param[in]		- number of pin
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- none
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{

}


