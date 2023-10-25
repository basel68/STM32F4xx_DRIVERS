/*
 * 02led_button.c
 *
 *  Created on: Oct 25, 2023
 *      Author: Bassel
 */


#include "STM32F407xx.h"
#include <stdint.h>

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}


int main(void)
{

	GPIO_Handle_t GpioLed, Gpiobutton;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);

	Gpiobutton.pGPIOx = GPIOA;
	Gpiobutton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	Gpiobutton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	Gpiobutton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;


	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&Gpiobutton);

	while(1)
	{
		uint8_t btn_value=GPIO_ReadFromInputPin(Gpiobutton.pGPIOx,Gpiobutton.GPIO_PinConfig.GPIO_PinNumber);
		if(btn_value){
			delay();
			GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
		}
	}
	return 0;
}
