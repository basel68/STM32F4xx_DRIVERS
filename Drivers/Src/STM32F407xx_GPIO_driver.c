/*
 * STM32F407xx_GPIO_driver.c
 *
 *  Created on: Oct 24, 2023
 *      Author: Bassel
 */


#include <STM32F407xx_GPIO_driver.h>
/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi){

				if(pGPIOx == GPIOA)
				{
					GPIOA_PCLK_EN();
				}else if (pGPIOx == GPIOB)
				{
					GPIOB_PCLK_EN();
				}else if (pGPIOx == GPIOC)
				{
					GPIOC_PCLK_EN();
				}else if (pGPIOx == GPIOD)
				{
					GPIOD_PCLK_EN();
				}else if (pGPIOx == GPIOE)
				{
					GPIOE_PCLK_EN();
				}else if (pGPIOx == GPIOF)
				{
					GPIOF_PCLK_EN();
				}else if (pGPIOx == GPIOG)
				{
					GPIOG_PCLK_EN();
				}else if (pGPIOx == GPIOH)
				{
					GPIOH_PCLK_EN();
				}else if (pGPIOx == GPIOI)
				{
					GPIOI_PCLK_EN();
				}

	}
	else{
					if(pGPIOx == GPIOA)
					{
						GPIOA_PCLK_DI();
					}else if (pGPIOx == GPIOB)
					{
						GPIOB_PCLK_DI();
					}else if (pGPIOx == GPIOC)
					{
						GPIOC_PCLK_DI();
					}else if (pGPIOx == GPIOD)
					{
						GPIOD_PCLK_DI();
					}else if (pGPIOx == GPIOE)
					{
						GPIOE_PCLK_DI();
					}else if (pGPIOx == GPIOF)
					{
						GPIOF_PCLK_DI();
					}else if (pGPIOx == GPIOG)
					{
						GPIOG_PCLK_DI();
					}else if (pGPIOx == GPIOH)
					{
						GPIOH_PCLK_DI();
					}else if (pGPIOx == GPIOI)
					{
						GPIOI_PCLK_DI();
					}
	}
}

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
		uint32_t temp=0; //temp. register

		 //enable the peripheral clock

		 GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

		// configure the mode of gpio pin

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
		{
			//the non interrupt mode
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
			pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
			pGPIOHandle->pGPIOx->MODER |= temp; //setting
			}
		else{

		}
		// configure the speed
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
		pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->OSPEEDR |= temp; //setting

	    //configure the pupd settings
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->PUPDR |= temp;

		//configure the optype
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->OTYPER |= temp;
	    //configure the alt functionality
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
		{
			//configure the alt function registers.
			uint8_t temp1, temp2;

			temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
			temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;
			pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
			pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );
		}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

		if(pGPIOx == GPIOA)
		{
			GPIOA_REG_RESET();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_REG_RESET();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_REG_RESET();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_REG_RESET();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_REG_RESET();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_REG_RESET();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_REG_RESET();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_REG_RESET();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_REG_RESET();
		}


}


/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value=(uint8_t)((pGPIOx->IDR>>PinNumber) & 0x1);
	return value;
}




uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value=(uint16_t)pGPIOx->IDR;
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if(Value == GPIO_PIN_SET)
		{

			pGPIOx->ODR |= ( 1 << PinNumber);
		}
	else
		{

			pGPIOx->ODR &= ~( 1 << PinNumber);
		}

}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR =Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^=(1<<PinNumber);
}


/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);
