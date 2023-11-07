/*
 * 02led_button.c
 *
 *  Created on: Oct 25, 2023
 *      Author: Bassel
 */


#include "STM32F407xx.h"
#include <string.h>
#include <stdint.h>

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}


int main(void)
{

	GPIO_Handle_t GpioLed, Gpiobutton;
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&Gpiobutton,0,sizeof(Gpiobutton));
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
	Gpiobutton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Gpiobutton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&Gpiobutton);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0,NVIC_IRQ_PRI7);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0,ENABLE);
	while(1);

	return 0;
}
void EXTI0_IRQHandler(void)
{
    delay(); //200ms . wait till button de-bouncing gets over
	GPIO_IRQHandling(GPIO_PIN_NO_0); //clear the pending event from exti line
	GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
}
