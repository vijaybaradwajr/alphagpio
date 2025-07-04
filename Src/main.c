/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>

#include "../Drivers/Inc/GPIO_Driver.h"



int main(void)
{
	 	gpio_init(GPIOA,ENABLE);
	 	gpio_config_pin(GPIOA,GPIO_PIN_NO_0 , GPIO_ANALOGMODE);
	 	gpio_init(GPIOA,DISABLE);
	 	return 0;
}
