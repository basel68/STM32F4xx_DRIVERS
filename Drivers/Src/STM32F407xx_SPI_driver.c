/*
 * STM32F407xx_SPI_driver.c
 *
 *  Created on: Nov 9, 2023
 *      Author: Bassel
 */

#include <STM32F407xx_SPI_driver.h>
/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if (EnorDi){
		if(pSPIx == SPI1)
				{
					SPI1_PCLK_EN();
				}else if (pSPIx == SPI2)
				{
					SPI2_PCLK_EN();
				}else if (pSPIx == SPI3)
				{
					SPI3_PCLK_EN();
				}
		}
	else{
		if(pSPIx == SPI1)
					{
						SPI1_PCLK_DI();
					}else if (pSPIx == SPI2)
					{
						SPI2_PCLK_DI();
					}else if (pSPIx == SPI3)
					{
						SPI3_PCLK_DI();
					}
		}
}
/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){
	//peripheral clock enable

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//first lets configure the SPI_CR1 register

	uint32_t tempreg = 0;

	//1. configuring the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR ;

	//2. Configuring the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= ( 1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= ( 1 << SPI_CR1_RXONLY);
	}

	// 3. Configuring the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4.  Configuring the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configuring the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6 . configuring the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;

}
void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}


/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len){

	while(Len<0){
			//1. wait until TXE is set
			while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)  == FLAG_RESET );

			//2. check the DFF bit in CR1
			if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
			{
				//16 bit DFF
				//1. load the data in to the DR
				pSPIx->DR =   *((uint16_t*)pTxBuffer);
				Len--;
				Len--;
				(uint16_t*)pTxBuffer++;
			}else
			{
				//8 bit DFF
				pSPIx->DR =   *pTxBuffer;
				Len--;
				pTxBuffer++;
			}
		}


	}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){
	while(Len > 0)
			{
				//1. wait until RXNE is set
				while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)  == (uint8_t)FLAG_RESET );

				//2. check the DFF bit in CR1
				if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
				{
					//16 bit DFF
					//1. load the data from DR to Rxbuffer address
					 *((uint16_t*)pRxBuffer) = pSPIx->DR ;
					Len--;
					Len--;
					(uint16_t*)pRxBuffer++;
				}else
				{
					//8 bit DFF
					*(pRxBuffer) = pSPIx->DR ;
					Len--;
					pRxBuffer++;
				}
			}

}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1 . Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2.  Mark the SPI state as busy in transmission so that
		//    no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );

	}


	return state;
}


uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1 . Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2.  Mark the SPI state as busy in reception so that
		//    no other code can take over same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );

	}


	return state;

}

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 32) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 32) );
		}
	}
}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//getting the ipr register
		uint8_t iprx = IRQNumber / 4;
		uint8_t iprx_section  = IRQNumber %4 ;

		uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

		*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
}
void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	 SPI_ClearOVRFlag(pSPIHandle->pSPIx);

	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	// check the DFF bit in CR1
	if( (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
	{
		//16 bit DFF
		//1. load the data in to the DR
		pSPIHandle->pSPIx->DR =   *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR =   *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen)
	{
		//TxLen is zero , so close the spi transmission and inform the application that
		//TX is over.

		//this prevents interrupts from setting up of TXE flag
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
	// check the DFF bit in CR1
	if( (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
	{
		//16 bit DFF
		//1. load the data in to the DR
		*((uint16_t*)pSPIHandle->pTxBuffer) =  pSPIHandle->pSPIx->DR ;
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
		//8 bit DFF
		*pSPIHandle->pTxBuffer= pSPIHandle->pSPIx->DR;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen)
	{
		//TxLen is zero , so close the spi transmission and inform the application that
		//TX is over.

		//this prevents interrupts from setting up of TXE flag
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}
void SPI_IRQHandling(SPI_Handle_t *pHandle){

		uint8_t temp1,temp2;
		temp1=pHandle->pSPIx->SR&(1<<SPI_SR_TXE);
		temp2=pHandle->pSPIx->CR2&(1<<SPI_CR2_TXEIE);

		if(temp1 && temp2){
			spi_txe_interrupt_handle(pHandle);
		}

		temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
		temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

		if( temp1 && temp2)
		{

			spi_rxne_interrupt_handle(pHandle);
		}


		temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
		temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

		if( temp1 && temp2)
		{

			spi_ovr_err_interrupt_handle(pHandle);
		}
}

/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if (EnOrDi)
		pSPIx->CR1|=1<<SPI_CR1_SPE;
	else
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
}
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if (EnOrDi)
		pSPIx->CR1|=1<<SPI_CR1_SSI;
	else
		pSPIx->CR1 &= ~(1<<SPI_CR1_SSI);
}
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if (EnOrDi)
			pSPIx->CR1|=1<<SPI_CR2_SSOE;
	else
			pSPIx->CR1 &= ~(1<<SPI_CR2_SSOE);
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName){
	if (pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle){

	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_TXEIE);
	pSPIHandle->TxLen=0;
	pSPIHandle->TxState=SPI_READY;
	pSPIHandle->pTxBuffer = NULL;

}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){

		pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
		pSPIHandle->pRxBuffer = NULL;
		pSPIHandle->RxLen = 0;
		pSPIHandle->RxState = SPI_READY;
}
//uint8_t I2C_DeviceMode(I2C_RegDef_t *I2Cx);

/*
 * Application callback
 */

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv){

}
