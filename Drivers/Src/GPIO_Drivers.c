/*
 * GPIO_Driver.c
 *
 *  Created on: Jun 23, 2025
 *      Author: vijay
 */

#include "../Inc/GPIO_Driver.h"

void gpio_init(GPIO_PinConfig_t *pGPIOx, uint8_t EnOrDi)

{
	if(EnOrDi == ENABLE)
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_Clk_En();
		}
		else if(pGPIOx==GPIOB)
				{
				GPIOB_Clk_En();
				}
		else if(pGPIOx==GPIOC)
				{
			GPIOC_Clk_En();
				}
		else if(pGPIOx==GPIOD)
				{
			GPIOD_Clk_En();
				}
		else if(pGPIOx==GPIOE)
				{
			GPIOE_Clk_En();
				}

	}
	else
	{
		if(pGPIOx==GPIOA)
				{
					GPIOA_Clk_Di();
				}
				else if(pGPIOx==GPIOB)
						{
						GPIOB_Clk_Di();
						}
				else if(pGPIOx==GPIOC)
						{
					GPIOC_Clk_Di();
						}
				else if(pGPIOx==GPIOD)
						{
					GPIOD_Clk_Di();
						}
				else if(pGPIOx==GPIOE)
						{
					GPIOE_Clk_Di();
						}

	}
	gpio_config_pin(pGPIOx->port, pGPIOx->pin, pGPIOx->mode);
	gpio_config_pin_speed(pGPIOx->port, pGPIOx->pin, pGPIOx->speed, pGPIOx->mode);


}

uint32_t PINPOS[16] = {
  (0x00), //pin0
  (0x04), //pin1
  (0x08), //pin2
  (0x0C), //pin3
  (0x10), //pin4
  (0x14), //pin5
  (0x18), //pin6
  (0x1C), //pin7
  (0x00), //pin8
  (0x04), //pin9
  (0x08), //pin10
  (0x0C), //pin11
  (0x10), //pin12
  (0x14), //pin13
  (0x18), //pin14
  (0x1C)  //pin15
};
void gpio_config_pin(GPIO_RegDef_t *pGPIOx, uint32_t pinNumber, uint32_t mode_type)
{
	if(pinNumber>7)
	{
		switch(mode_type){
		   case GPIO_PUPD | GPIO_ANALOGMODE:
		    pGPIOx->CRH &= ~((1 << CNF_POS_BIT1) | (1<<CNF_POS_BIT2));
		    break;
		   case GPIO_OPD | GPIO_FLOATINGIN:
		    pGPIOx->CRH &= ~( 1 << CNF_POS_BIT2);
		    pGPIOx->CRH |= (1 << CNF_POS_BIT1);
		    break;
		   case GPIO_AFPUPD | GPIO_INPUPD:
		   pGPIOx->CRH &= (1 << CNF_POS_BIT1);
		   pGPIOx->CRH |= ~( 1 << CNF_POS_BIT2);
		    break;
		   case GPIO_AFUOPD|GPIO_RESERVED:
		     pGPIOx->CRH |= ((1<< CNF_POS_BIT1)| (1<<CNF_POS_BIT2)) ;
		    break;
		  }
	}
	else
	{
		switch(mode_type){
				   case GPIO_PUPD | GPIO_ANALOGMODE:
				    pGPIOx->CRH &= ~((1 << CNF_POS_BIT1) | (1<<CNF_POS_BIT2));
				    break;
				   case GPIO_OPD | GPIO_FLOATINGIN:
				    pGPIOx->CRH &= ~( 1 << CNF_POS_BIT2);
				    pGPIOx->CRH |= (1 << CNF_POS_BIT1);
				    break;
				   case GPIO_AFPUPD | GPIO_INPUPD:
				   pGPIOx->CRH &= (1 << CNF_POS_BIT1);
				   pGPIOx->CRH |= ~( 1 << CNF_POS_BIT2);
				    break;
				   case GPIO_AFUOPD|GPIO_RESERVED:
				     pGPIOx->CRH |= ((1<< CNF_POS_BIT1)| (1<<CNF_POS_BIT2)) ;
				    break;
				  }
	}
}

void gpio_config_pin_speed (GPIO_RegDef_t *pGPIOx, uint32_t pinNumber, uint32_t pinSpeed, uint32_t mode)
{

 if(pinNumber >= 8)
 {
  if(mode == INPUT_MODE)
   pGPIOx->CRH &= ~ (1 << (PINPOS[pinNumber]) |  1 << (PINPOS[pinNumber] +1) );
  else
   pGPIOx->CRH = (pinSpeed << (PINPOS[pinNumber]));
 }else{
  if(mode == INPUT_MODE)
   pGPIOx->CRL &= ~ (1 << (PINPOS[pinNumber]) |  1 << (PINPOS[pinNumber] +1) );
  else
   pGPIOx->CRL |= (pinSpeed << (PINPOS[pinNumber]));
 }

}
void config_write(GPIO_RegDef_t *pGPIOx, uint32_t pinNumber, uint8_t state)
{
	if(state)
	{
		pGPIOx->BSRR=(1<<pinNumber);

	}
	else
	{
		pGPIOx->BSRR=(1<<(pinNumber+16));
	}
}
void config_read(GPIO_RegDef_t *pGPIOx,uint32_t pinNumber)
{
	uint8_t value;
	value=(uint8_t)((pGPIOx->IDR>>pinNumber)&(0x00000001));
	return value;
}
void gpio_toggle(GPIO_RegDef_t *pGPIOx, uint32_t pinNumber){

 pGPIOx->ODR ^=(1<<pinNumber);

}

void configure_gpio_interrupt(GPIO_RegDef_t *pGPIOx, uint32_t pinNumber, edge_select edge){
 //GPIO_PCLK_EN_ALT_FUNC();
 RCC->APB2ENR |= (1<<0);

 if(pGPIOx == GPIOA){
  switch(pinNumber)
  {
   case 0:
    AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PA;
    break;
   case 1:
    AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PA;
    break;
   case 2:
    AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PA;
    break;
   case 3:
    AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PA;
    break;
   case 4:
    AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PA;
    break;
   case 5:
    AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PA;
    break;
   case 6:
    AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PA;
    break;
   case 7:
    AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PA;
    break;
   case 8:
    AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PA;
    break;
   case 9:
    AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PA;
    break;
   case 10:
    AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PA;
    break;
   case 11:
    AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PA;
    break;
   case 12:
    AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PA;
    break;
   case 13:
    AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PA;
    break;
   case 14:
    AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PA;
    break;
   case 15:
    AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PA;
    break;
   default:
    break;
  }
 }

 if(pGPIOx == GPIOB){
  switch(pinNumber)
  {
   case 0:
    AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PB;
    break;
   case 1:
    AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PB;
    break;
   case 2:
    AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PB;
    break;
   case 3:
    AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PB;
    break;
   case 4:
    AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PB;
    break;
   case 5:
    AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PB;
    break;
   case 6:
    AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PB;
    break;
   case 7:
    AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PB;
    break;
   case 8:
    AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PB;
    break;
   case 9:
    AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PB;
    break;
   case 10:
    AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PB;
    break;
   case 11:
    AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PB;
    break;
   case 12:
    AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PB;
    break;
   case 13:
    AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PB;
    break;
   case 14:
    AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PB;
    break;
   case 15:
    AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PB;
    break;
   default:
    break;
  }
 }

 if(pGPIOx == GPIOC){
  switch(pinNumber)
  {
   case 0:
    AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PC;
    break;
   case 1:
    AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PC;
    break;
   case 2:
    AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PC;
    break;
   case 3:
    AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PC;
    break;
   case 4:
    AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PC;
    break;
   case 5:
    AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PC;
    break;
   case 6:
    AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PC;
    break;
   case 7:
    AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PC;
    break;
   case 8:
    AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PC;
    break;
   case 9:
    AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PC;
    break;
   case 10:
    AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PC;
    break;
   case 11:
    AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PC;
    break;
   case 12:
    AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PC;
    break;
   case 13:
    AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PC;
    break;
   case 14:
    AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PC;
    break;
   case 15:
    AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PC;
    break;
   default:
    break;
  }
 }

 if(pGPIOx == GPIOD){
  switch(pinNumber)
  {
   case 0:
    AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PD;
    break;
   case 1:
    AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PD;
    break;
   case 2:
    AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PD;
    break;
   case 3:
    AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PD;
    break;
   case 4:
    AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PD;
    break;
   case 5:
    AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PD;
    break;
   case 6:
    AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PD;
    break;
   case 7:
    AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PD;
    break;
   case 8:
    AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PD;
    break;
   case 9:
    AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PD;
    break;
   case 10:
    AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PD;
    break;
   case 11:
    AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PD;
    break;
   case 12:
    AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PD;
    break;
   case 13:
    AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PD;
    break;
   case 14:
    AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PD;
    break;
   case 15:
    AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PD;
    break;
   default:
    break;
  }
 }

 if(edge == RISING_EDGE)
  EXTI->RTSR |= 1 << pinNumber;
 if(edge == FALLING_EDGE)
  EXTI->FTSR |= 1 << pinNumber;
 if(edge == RISING_FALLING_EDGE){
  EXTI->RTSR |= 1 << pinNumber;
  EXTI->FTSR |= 1 << pinNumber;
 }
}


void enable_gpio_interrupt(uint32_t pinNumber, IRQn_Type irqNumber)
{
	//enable interrupt in EXTI
	 EXTI->IMR |= 1 << pinNumber;
	 //enable interrupt in NVIC
	 NVIC_EnableIRQ(irqNumber);
}
void clear_gpio_interrupt(uint32_t pinNumber){

 EXTI->PR |= (1 << pinNumber);
}


