/*
 * GPIO_Driver.h
 *
 *  Created on: Jun 22, 2025
 *      Author: vijay
 */

#ifndef GPIO_DRIVER_H_
#define GPIO_DRIVER_H_
#include "stm32f103c6xx.h"


//clock enable or disable API
void gpio_init(GPIO_PinConfig_t *pGPIOx, uint8_t EnOrDi);
void gpio_config_pin (GPIO_RegDef_t *pGPIOx, uint32_t pinNumber, uint32_t mode_type);
void gpio_config_pin_speed (GPIO_RegDef_t *pGPIOx, uint32_t pinNumber, uint32_t pinSpeed, uint32_t mode);
void gpio_write(GPIO_RegDef_t *pGPIOx, uint32_t pinNumber, uint8_t state);
void gpio_read(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
void gpio_toggle(GPIO_RegDef_t *pGPIOx, uint32_t pinNumber);
void configure_gpio_interrupt(GPIO_RegDef_t *pGPIOx, uint32_t pinNumber, edge_select edge);
void enable_gpio_interrupt(uint32_t pinNumber, IRQn_Type irqNumber);
void clear_gpio_interrupt(uint32_t pinNumber);

//HIGH BIT POSITIONS FOR CRH REGISTER CNFYG AND MODE
#define CNF_POS_BIT1    (PINPOS[pinNumber] + 2)
#define CNF_POS_BIT2    (PINPOS[pinNumber] + 3)

#endif /* GPIO_DRIVER_H_ */
